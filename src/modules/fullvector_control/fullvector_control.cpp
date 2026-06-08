/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fullvector_control_main.cpp
 *
 * Full-vector quadcopter controller with cascaded position/attitude PID loops
 * and direct normalized actuator output for motors and tilt servos.
 *
 * @author Mingyang Wang <3112311639@qq.com>
 */

#define MODULE_NAME "fullvector_control"

#include "fullvector_control.hpp"
#include <drivers/drv_hrt.h>
#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <px4_platform_common/log.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>

using namespace time_literals;
using namespace matrix;


FullvectorControl::FullvectorControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 构造时先读取一次参数，保证控制增益和机体物理参数都有初值。
	parameters_update(true);

	// 初始化内部状态缓存，后续会被 uORB 订阅到的估计器状态覆盖。
	_current_state.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.velocity = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.attitude = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_current_state.angular_velocity = Vector3f(0.0f, 0.0f, 0.0f);

	// 初始化命令缓存，实际目标点会在首次获得有效状态和后续 trajectory_setpoint 中写入。
	_current_command.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
}

FullvectorControl::~FullvectorControl()
{
	perf_free(_loop_perf);
}

bool FullvectorControl::init()
{
	// 调度一次 WorkQueue，之后 Run() 内部会周期性重新调度。
	ScheduleNow();
	return true;
}

void FullvectorControl::resetPidState()
{
	// 清空位置/速度环积分和前一拍误差，避免模式切换或参数更新后积分残留。
	_pos_error_int.zero();
	_vel_error_int.zero();
	_pid_state_initialized = false;

	// 清空姿态/角速度环积分和前一拍误差。
	_att_error_int.zero();
	_ang_vel_error_int.zero();
	_att_pid_state_initialized = false;
}

void FullvectorControl::publishSafeActuatorFallback()
{
	// 状态不可用或严重过期时发布安全输出：前四路电机停转，其余电机通道保持 NaN。
	actuator_motors_s motor_safe{};
	motor_safe.timestamp_sample = hrt_absolute_time();
	motor_safe.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_safe.control[i] = NAN;
	}

	motor_safe.control[0] = 0.0f;
	motor_safe.control[1] = 0.0f;
	motor_safe.control[2] = 0.0f;
	motor_safe.control[3] = 0.0f;
	_motor_speed_pub_raw.publish(motor_safe);

	// 倾转舵机回到中位，其余舵机通道保持 NaN。
	actuator_servos_s tilt_safe{};
	tilt_safe.timestamp_sample = hrt_absolute_time();
	tilt_safe.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		tilt_safe.control[i] = NAN;
	}

	tilt_safe.control[0] = 0.0f;
	tilt_safe.control[1] = 0.0f;
	tilt_safe.control[2] = 0.0f;
	tilt_safe.control[3] = 0.0f;
	_motor_tilt_pub_raw.publish(tilt_safe);
}

void FullvectorControl::parameters_update(bool force)
{
	// 检查参数是否更新；force=true 时用于构造阶段强制刷新一次。
	if (_parameter_update_sub.updated() || force) {
		// 读取并清除 parameter_update 通知。
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从 PX4 参数系统同步最新参数值到 ModuleParams 缓存。
		ModuleParams::updateParams();

		// 位置外环 PID 增益矩阵：行对应 x/y/z，列对应 P/I/D。
		gain_pos_pid.setZero();
		gain_pos_pid(0, 0) = _param_fv_pos_p_x.get();
		gain_pos_pid(0, 1) = _param_fv_pos_i_x.get();
		gain_pos_pid(0, 2) = _param_fv_pos_d_x.get();
		gain_pos_pid(1, 0) = _param_fv_pos_p_y.get();
		gain_pos_pid(1, 1) = _param_fv_pos_i_y.get();
		gain_pos_pid(1, 2) = _param_fv_pos_d_y.get();
		gain_pos_pid(2, 0) = _param_fv_pos_p_z.get();
		gain_pos_pid(2, 1) = _param_fv_pos_i_z.get();
		gain_pos_pid(2, 2) = _param_fv_pos_d_z.get();

		// 速度内环 PID 增益矩阵。
		gain_vel_pid.setZero();
		gain_vel_pid(0, 0) = _param_fv_vel_p_x.get();
		gain_vel_pid(0, 1) = _param_fv_vel_i_x.get();
		gain_vel_pid(0, 2) = _param_fv_vel_d_x.get();
		gain_vel_pid(1, 0) = _param_fv_vel_p_y.get();
		gain_vel_pid(1, 1) = _param_fv_vel_i_y.get();
		gain_vel_pid(1, 2) = _param_fv_vel_d_y.get();
		gain_vel_pid(2, 0) = _param_fv_vel_p_z.get();
		gain_vel_pid(2, 1) = _param_fv_vel_i_z.get();
		gain_vel_pid(2, 2) = _param_fv_vel_d_z.get();

		// 姿态外环 PID 增益矩阵。
		gain_att_pid.setZero();
		gain_att_pid(0, 0) = _param_fv_att_p_x.get();
		gain_att_pid(0, 1) = _param_fv_att_i_x.get();
		gain_att_pid(0, 2) = _param_fv_att_d_x.get();
		gain_att_pid(1, 0) = _param_fv_att_p_y.get();
		gain_att_pid(1, 1) = _param_fv_att_i_y.get();
		gain_att_pid(1, 2) = _param_fv_att_d_y.get();
		gain_att_pid(2, 0) = _param_fv_att_p_z.get();
		gain_att_pid(2, 1) = _param_fv_att_i_z.get();
		gain_att_pid(2, 2) = _param_fv_att_d_z.get();

		// 角速度内环 PID 增益矩阵。
		gain_ang_vel_pid.setZero();
		gain_ang_vel_pid(0, 0) = _param_fv_ang_vel_p_x.get();
		gain_ang_vel_pid(0, 1) = _param_fv_ang_vel_i_x.get();
		gain_ang_vel_pid(0, 2) = _param_fv_ang_vel_d_x.get();
		gain_ang_vel_pid(1, 0) = _param_fv_ang_vel_p_y.get();
		gain_ang_vel_pid(1, 1) = _param_fv_ang_vel_i_y.get();
		gain_ang_vel_pid(1, 2) = _param_fv_ang_vel_d_y.get();
		gain_ang_vel_pid(2, 0) = _param_fv_ang_vel_p_z.get();
		gain_ang_vel_pid(2, 1) = _param_fv_ang_vel_i_z.get();
		gain_ang_vel_pid(2, 2) = _param_fv_ang_vel_d_z.get();

		// 机体物理参数，用于执行器分配、推力/力矩估算和调试动力学积分。
		mass = _param_fv_mass.get();
		gravity = _param_fv_gravity.get();
		distance = _param_fv_motor_distance.get();
		K_F = _param_fv_K_F.get();
		K_M = _param_fv_K_M.get();

		// 惯量矩阵当前只使用对角项 Ixx/Iyy/Izz。
		inertia.setZero();
		inertia(0, 0) = _param_fv_inertia_xx.get();
		inertia(1, 1) = _param_fv_inertia_yy.get();
		inertia(2, 2) = _param_fv_inertia_zz.get();

		J_RP = _param_fv_J_RP.get();

		// 参数改变后重置 PID 状态，避免旧积分项和新增益混用。
		resetPidState();
	}
}

bool FullvectorControl::updateUAVState()
{
	_state_age_level = 0;

	// 读取本地位置和速度估计，仅在有效标志为真时更新内部状态。
	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&_position);

		if (_position.xy_valid && _position.z_valid) {
			_current_state.position = Vector3f(_position.x, _position.y, _position.z);
			_last_position_update = _position.timestamp;
		}

		if (_position.v_xy_valid && _position.v_z_valid) {
			_current_state.velocity = Vector3f(_position.vx, _position.vy, _position.vz);
			_last_velocity_update = _position.timestamp;
		}
	}

	// 读取姿态四元数。
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_attitude);
		_current_state.attitude = matrix::Quatf(_attitude.q);
		_last_attitude_update = _attitude.timestamp;
	}

	// 读取机体系角速度。
	if (_vehicle_angular_velocity_sub.updated()) {
		_vehicle_angular_velocity_sub.copy(&_angular_velocity);
		_current_state.angular_velocity = Vector3f(_angular_velocity.xyz[0],
								   _angular_velocity.xyz[1],
								   _angular_velocity.xyz[2]);
		_last_angular_velocity_update = _angular_velocity.timestamp;
	}

	// 所有必要状态都至少更新过一次后，才允许进入控制计算。
	if ((_last_position_update == 0) || (_last_velocity_update == 0)
	    || (_last_attitude_update == 0) || (_last_angular_velocity_update == 0)) {
		return false;
	}

	const hrt_abstime elapsed_position = hrt_elapsed_time(&_last_position_update);
	const hrt_abstime elapsed_velocity = hrt_elapsed_time(&_last_velocity_update);
	const hrt_abstime elapsed_attitude = hrt_elapsed_time(&_last_attitude_update);
	const hrt_abstime elapsed_ang_vel = hrt_elapsed_time(&_last_angular_velocity_update);

	// 区分“偏旧但可运行”和“严重过期必须失效保护”的状态。
	constexpr hrt_abstime stale_warn_timeout = 200_ms;
	constexpr hrt_abstime stale_fail_timeout = 500_ms;
	const bool aging = (elapsed_position > stale_warn_timeout)
			  || (elapsed_velocity > stale_warn_timeout)
			  || (elapsed_attitude > stale_warn_timeout)
			  || (elapsed_ang_vel > stale_warn_timeout);

	const bool stale_fail = (elapsed_position > stale_fail_timeout)
			       || (elapsed_velocity > stale_fail_timeout)
			       || (elapsed_attitude > stale_fail_timeout)
			       || (elapsed_ang_vel > stale_fail_timeout);

	if (aging) {
		static hrt_abstime last_stale_warn{0};

		// 限频打印状态老化告警，避免刷屏。
		if ((last_stale_warn == 0) || (hrt_elapsed_time(&last_stale_warn) > 1_s)) {
			PX4_WARN("state aging: pos=%llu vel=%llu att=%llu ang=%llu us",
				 (unsigned long long)elapsed_position,
				 (unsigned long long)elapsed_velocity,
				 (unsigned long long)elapsed_attitude,
				 (unsigned long long)elapsed_ang_vel);
			last_stale_warn = hrt_absolute_time();
		}
	}

	_state_age_level = stale_fail ? 2 : (aging ? 1 : 0);

	return true;
}


void FullvectorControl::Run()
{
	// 模块退出处理：停止调度并释放模块实例。
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 控制器按 5 ms 周期运行，即使本周期没有输出也保持调度。
	ScheduleDelayed(5_ms);

	parameters_update(false);

	// 读取飞控当前控制模式、导航状态和上层轨迹目标，用于判断是否允许本控制器输出。
	_vehicle_control_mode_sub.update(&_control_mode);
	_vehicle_status_sub.update(&_vehicle_status);
	_trajectory_setpoint_sub.update(&_trajectory_setpoint);

	const bool fv_enabled = (_param_fv_enable.get() == 1);
	const bool armed = _control_mode.flag_armed;
	const bool posctl_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL);
	const bool offboard_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	// 终止模式下必须停止本控制器输出，避免和 PX4 失效终止逻辑冲突。
	const bool termination_mode = _control_mode.flag_control_termination_enabled
				      || (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION);
	// 状态估计是否可用由 updateUAVState() 判断；是否进入 fullvector 闭环由飞行模式显式门控。
	const bool fullvector_mode_allowed = posctl_mode || offboard_mode;
	const bool module_active = fv_enabled && armed && !termination_mode;

	if (!module_active) {
		// 模块未启用、未解锁或终止模式时，不发布控制输出。
		// 控制器失活时清空 PID 状态，下一次激活从干净状态开始。
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		return;
	}

	if (!fullvector_mode_allowed) {
		// 非 fullvector 闭环模式下完全静默，让 PX4 原生控制器独占输出。
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	const bool debug_print_enabled = (_param_print_msg_a_en.get() != 0);
	const bool allow_debug_print = debug_print_enabled
					      && ((_last_debug_print_time == 0) || (hrt_elapsed_time(&_last_debug_print_time) > 200_ms));

	if (!_controller_was_active) {
		// 从未激活到激活的上升沿，重置时间和积分项。
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;
	}

	_controller_was_active = true;

	// 计算本次控制周期 dt，首次运行时使用名义 5 ms。
	if (_last_run_time == 0) {
		_dt = 0.005f;

	} else {
		_dt = (now - _last_run_time) / 1e6f;
	}

	_last_run_time = now;

	if (!PX4_ISFINITE(_dt) || (_dt <= FLT_EPSILON)) {
		return;
	}

	// 防止调度抖动导致微分项异常放大。
	constexpr float dt_clamp_s = 0.05f;
	constexpr float dt_reset_s = 0.1f;

	if (_dt > dt_reset_s) {
		resetPidState();
		_dt = 0.01f;

	} else if (_dt > dt_clamp_s) {
		_dt = dt_clamp_s;
	}

	// 状态尚未初始化时发布安全输出，避免用无效状态闭环控制。
	if (!updateUAVState()) {
		publishSafeActuatorFallback();
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;

		if (allow_debug_print) {
			PX4_WARN("state unavailable, publishing safe actuator fallback");
			_last_debug_print_time = now;
		}

		return;
	}

	// 状态严重过期时进入失效保护，不继续计算控制输出。
	if (_state_age_level >= 2) {
		publishSafeActuatorFallback();
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;

		if (allow_debug_print) {
			PX4_WARN("state stale-fail, publishing safe actuator fallback");
			_last_debug_print_time = now;
		}

		return;
	}

	// 使用估计器状态作为控制输入。
	const UAVStates &state_for_control = _current_state;

	if (!_command_initialized) {
		// 第一次收到有效状态后，将位置和姿态目标对齐到当前飞机状态，避免模式切换时目标阶跃。
		_current_command.position = _current_state.position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_current_command.Euler_angles = Vector3f(Eulerf(_current_state.attitude));
		_current_command.angular_velocity.zero();
		_command_initialized = true;
	}

	const bool trajectory_valid = (_trajectory_setpoint.timestamp != 0)
				      && (hrt_elapsed_time(&_trajectory_setpoint.timestamp) < 500_ms);
	float yaw_sp_target = _current_command.Euler_angles(2);

	if (trajectory_valid) {
		// POSCTL: FlightModeManager 会把遥控器输入转换成 trajectory_setpoint。
		// OFFBOARD: trajectory_setpoint 通常来自外部控制端。
		// 上层可能只填充部分轴的 setpoint，本控制器只采用有限值字段。
		// 对有限值才覆盖当前命令，NaN 表示该轴不由上层轨迹直接约束。
		for (int i = 0; i < 3; i++) {
			if (PX4_ISFINITE(_trajectory_setpoint.position[i])) {
				_current_command.position(i) = _trajectory_setpoint.position[i];
			}

			if (PX4_ISFINITE(_trajectory_setpoint.velocity[i])) {
				_current_command.velocity(i) = _trajectory_setpoint.velocity[i];
			}

			if (PX4_ISFINITE(_trajectory_setpoint.acceleration[i])) {
				_current_command.acceleration(i) = _trajectory_setpoint.acceleration[i];
			}
		}

		if (PX4_ISFINITE(_trajectory_setpoint.yaw)) {
			yaw_sp_target = matrix::wrap_pi(_trajectory_setpoint.yaw);
		}
	}

	// fullvector 平移主要依靠电机倾转实现，姿态目标保持水平，yaw 默认保持切入时航向。
	// 对姿态目标做限速，避免从自稳切到定点时产生姿态/航向阶跃。
	Vector3f attitude_sp_target = _current_command.Euler_angles;
	attitude_sp_target(0) = 0.0f;
	attitude_sp_target(1) = 0.0f;
	attitude_sp_target(2) = yaw_sp_target;
	_current_command.angular_velocity.zero();

	constexpr float attitude_sp_slew_rate = 1.0f; // rad/s
	const float attitude_sp_step = attitude_sp_slew_rate * _dt;

	for (int i = 0; i < 3; i++) {
		const float error = matrix::wrap_pi(attitude_sp_target(i) - _current_command.Euler_angles(i));
		_current_command.Euler_angles(i) = matrix::wrap_pi(_current_command.Euler_angles(i)
						+ math::constrain(error, -attitude_sp_step, attitude_sp_step));
	}

	perf_begin(_loop_perf);

	if (allow_debug_print) {
		PX4_INFO("debug value=%.3f", (double)_param_print_num_value.get());
	}

	// 读取 PX4 标准推力设定值，仅用于调试打印，不参与本控制器输出。
	vehicle_thrust_setpoint_s thrust_feedback{};
	const bool thrust_updated = _vehicle_thrust_setpoint_sub.update(&thrust_feedback);

	// 调试用途：读取当前 actuator_motors，便于观察和对比输出。
	actuator_motors_s actuator_motors{};
	const bool motors_updated = _actuator_motors_sub.update(&actuator_motors);

	if (allow_debug_print && (thrust_updated || motors_updated)) {
		if (thrust_updated) {
			PX4_INFO("[ThrustSub] thrust_sp=[%.3f, %.3f, %.3f] upward=%.3f",
			       (double)thrust_feedback.xyz[0],
			       (double)thrust_feedback.xyz[1],
			       (double)thrust_feedback.xyz[2],
			       (double)(-thrust_feedback.xyz[2]));
		}

		if (motors_updated) {
			PX4_INFO("[MotorOut] m1=%.3f m2=%.3f m3=%.3f m4=%.3f",
			       (double)actuator_motors.control[0],
			       (double)actuator_motors.control[1],
			       (double)actuator_motors.control[2],
			       (double)actuator_motors.control[3]);
		}
	}

	// 读取起飞状态和落地检测，配合调试日志判断当前飞行阶段。
	takeoff_status_s takeoff_status{};
	vehicle_land_detected_s land_detected{};
	_takeoff_status_sub.update(&takeoff_status);
	_vehicle_land_detected_sub.update(&land_detected);

	// 打印飞行阶段和基本状态，帮助判断控制器是否在预期状态下运行。
	if (allow_debug_print) {
		PX4_INFO("[FlightState] takeoff_state=%d landed=%d ground_contact=%d pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f]",
			 (int)takeoff_status.takeoff_state,
			 (int)land_detected.landed,
			 (int)land_detected.ground_contact,
			 (double)_current_state.position(0), (double)_current_state.position(1), (double)_current_state.position(2),
			 (double)_current_state.velocity(0), (double)_current_state.velocity(1), (double)_current_state.velocity(2));
		_last_debug_print_time = now;
	}

	// 串级控制流程：位置环 + 姿态环 -> 力/力矩分配。
	PositionControl(state_for_control, _current_command, _dt);
	AttitudeControl(state_for_control, _current_command, _dt);
	controlAllocation(state_for_control, _current_command);

	perf_end(_loop_perf);
}

void FullvectorControl::PositionControl(const UAVStates & state, const UAVCommand & command, const float dt)
{
	// dt 无效时不更新 PID，避免除零或 NaN 传播。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	// 位置误差：当前内部位置目标减估计位置。
	const Vector3f ep = command.position - state.position;

	if (!_pid_state_initialized) {
		// 首次运行时用当前误差初始化历史项，避免微分项跳变。
		_pos_error_prev = ep;
		_vel_error_prev.zero();
		_pid_state_initialized = true;
	}

	// 位置外环 PID：根据位置误差生成内部期望速度。
	const Vector3f dep = (ep - _pos_error_prev) / dt;
	_pos_error_int += ep * dt;

	// 位置环积分限幅，抑制积分饱和。
	for (int i = 0; i < 3; i++) {
		_pos_error_int(i) = math::constrain(_pos_error_int(i), -5.0f, 5.0f);
	}

	// 从矩阵中取出三个轴的 P/I/D 参数。
	const Vector3f pos_kp{gain_pos_pid(0, 0), gain_pos_pid(1, 0), gain_pos_pid(2, 0)};
	const Vector3f pos_ki{gain_pos_pid(0, 1), gain_pos_pid(1, 1), gain_pos_pid(2, 1)};
	const Vector3f pos_kd{gain_pos_pid(0, 2), gain_pos_pid(1, 2), gain_pos_pid(2, 2)};

	Vector3f v_sp = pos_kp.emult(ep) + pos_ki.emult(_pos_error_int) + pos_kd.emult(dep);

	// 限制期望速度，避免内环指令过大。
	for (int i = 0; i < 3; i++) {
		v_sp(i) = math::constrain(v_sp(i), -5.0f, 5.0f);
	}

	// 速度内环 PID：根据内部期望速度和估计速度的误差生成期望加速度。
	const Vector3f ev = v_sp - state.velocity;
	const Vector3f dev = (ev - _vel_error_prev) / dt;
	_vel_error_int += ev * dt;

	for (int i = 0; i < 3; i++) {
		_vel_error_int(i) = math::constrain(_vel_error_int(i), -3.0f, 3.0f);
	}

	const Vector3f vel_kp{gain_vel_pid(0, 0), gain_vel_pid(1, 0), gain_vel_pid(2, 0)};
	const Vector3f vel_ki{gain_vel_pid(0, 1), gain_vel_pid(1, 1), gain_vel_pid(2, 1)};
	const Vector3f vel_kd{gain_vel_pid(0, 2), gain_vel_pid(1, 2), gain_vel_pid(2, 2)};

	const Vector3f acc_cmd = vel_kp.emult(ev) + vel_ki.emult(_vel_error_int) + vel_kd.emult(dev);
	// 保存给后续电机倾转角和电机转速计算使用。
	_pos_acc_cmd = acc_cmd;

	// 更新历史误差，供下一周期微分项使用。
	_pos_error_prev = ep;
	_vel_error_prev = ev;

	// 发布位置控制器输出，便于日志查验或其他模块观察当前加速度指令。
	vehicle_local_position_setpoint_s position_controller_output{};
	position_controller_output.timestamp = hrt_absolute_time();
	position_controller_output.acceleration[0] = acc_cmd(0);
	position_controller_output.acceleration[1] = acc_cmd(1);
	position_controller_output.acceleration[2] = acc_cmd(2);
	_position_controller_output_pub.publish(position_controller_output);

	if ((_param_print_msg_a_en.get() != 0) && ((_last_debug_print_time == 0) || (hrt_elapsed_time(&_last_debug_print_time) > 200_ms))) {
		PX4_INFO("[PosCtrl] ep=[%.2f %.2f %.2f] acc_sp=[%.2f %.2f %.2f]",
			 (double)ep(0), (double)ep(1), (double)ep(2),
			 (double)acc_cmd(0), (double)acc_cmd(1), (double)acc_cmd(2));
	}
}

void FullvectorControl::AttitudeControl(const UAVStates & state, UAVCommand & command, const float dt)
{
	// dt 无效时不更新 PID。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	// 将当前四元数姿态转为欧拉角，与目标欧拉角做误差。
	const Vector3f euler_cur = Vector3f(Eulerf(state.attitude));
	const Vector3f euler_sp(command.Euler_angles(0), command.Euler_angles(1), command.Euler_angles(2));

	// 姿态角误差需要 wrap 到 [-pi, pi]，避免跨越 pi 时出现大跳变。
	Vector3f e_att = euler_sp - euler_cur;
	e_att(0) = matrix::wrap_pi(e_att(0));
	e_att(1) = matrix::wrap_pi(e_att(1));
	e_att(2) = matrix::wrap_pi(e_att(2));

	if (!_att_pid_state_initialized) {
		// 首次运行初始化微分历史项。
		_att_error_prev = e_att;
		_ang_vel_error_prev.zero();
		_att_pid_state_initialized = true;
	}

	// 姿态外环 PID：根据姿态误差生成期望角速度。
	const Vector3f de_att = (e_att - _att_error_prev) / dt;
	_att_error_int += e_att * dt;

	for (int i = 0; i < 3; i++) {
		_att_error_int(i) = math::constrain(_att_error_int(i), -1.0f, 1.0f);
	}

	const Vector3f att_kp{gain_att_pid(0, 0), gain_att_pid(1, 0), gain_att_pid(2, 0)};
	const Vector3f att_ki{gain_att_pid(0, 1), gain_att_pid(1, 1), gain_att_pid(2, 1)};
	const Vector3f att_kd{gain_att_pid(0, 2), gain_att_pid(1, 2), gain_att_pid(2, 2)};

	const Vector3f omega_sp = att_kp.emult(e_att) + att_ki.emult(_att_error_int) + att_kd.emult(de_att);
	// 将姿态外环生成的期望角速度写回 command，供调试或后续扩展使用。
	command.angular_velocity = omega_sp;

	// 角速度内环 PID：根据期望角速度和当前角速度生成期望角加速度。
	const Vector3f e_w = omega_sp - state.angular_velocity;
	const Vector3f de_w = (e_w - _ang_vel_error_prev) / dt;
	_ang_vel_error_int += e_w * dt;

	for (int i = 0; i < 3; i++) {
		_ang_vel_error_int(i) = math::constrain(_ang_vel_error_int(i), -3.0f, 3.0f);
	}

	const Vector3f w_kp{gain_ang_vel_pid(0, 0), gain_ang_vel_pid(1, 0), gain_ang_vel_pid(2, 0)};
	const Vector3f w_ki{gain_ang_vel_pid(0, 1), gain_ang_vel_pid(1, 1), gain_ang_vel_pid(2, 1)};
	const Vector3f w_kd{gain_ang_vel_pid(0, 2), gain_ang_vel_pid(1, 2), gain_ang_vel_pid(2, 2)};

	const Vector3f ang_acc_cmd = w_kp.emult(e_w) + w_ki.emult(_ang_vel_error_int) + w_kd.emult(de_w);
	// 保存给后续电机转速分配使用。
	_att_ang_acc_cmd = ang_acc_cmd;

	// 更新历史误差。
	_att_error_prev = e_att;
	_ang_vel_error_prev = e_w;

	// 发布姿态控制器的输出，便于观察控制链路。
	vehicle_angular_acceleration_setpoint_s attitude_controller_output{};
	attitude_controller_output.timestamp_sample = hrt_absolute_time();
	attitude_controller_output.timestamp = hrt_absolute_time();
	attitude_controller_output.xyz[0] = ang_acc_cmd(0);
	attitude_controller_output.xyz[1] = ang_acc_cmd(1);
	attitude_controller_output.xyz[2] = ang_acc_cmd(2);
	_attitude_controller_output_pub.publish(attitude_controller_output);

	if ((_param_print_msg_a_en.get() != 0) && ((_last_debug_print_time == 0) || (hrt_elapsed_time(&_last_debug_print_time) > 200_ms))) {
		PX4_INFO("[AttCtrl] e_att=[%.2f %.2f %.2f] ang_acc_sp=[%.2f %.2f %.2f]",
			 (double)e_att(0), (double)e_att(1), (double)e_att(2),
			 (double)ang_acc_cmd(0), (double)ang_acc_cmd(1), (double)ang_acc_cmd(2));
	}
}

void FullvectorControl::calculateMotorCommand(const UAVCommand & command)
{
	// 读取期望姿态角；当前电机倾转角主要使用 roll/pitch。
	const float phi_sp = command.Euler_angles(0);
	const float theta_sp = command.Euler_angles(1);
	const float psi_sp = command.Euler_angles(2);
	(void)psi_sp;

	// 使用前级位置环输出和姿态环输出。
	const Vector3f &acc_sp = _pos_acc_cmd;
	const Vector3f &ang_acc_sp = _att_ang_acc_cmd;
	constexpr float tilt_angle_max_rad = 0.52f;

	// 根据悬停推力估算基础电机角速度，K_F 太小时做保护。
	const float kf_safe = math::max(K_F, 1e-6f);
	const float mass_safe = math::max(mass, 1e-3f);
	const float gravity_safe = math::max(gravity, 1e-3f);
	const float distance_safe = math::max(distance, 1e-3f);
	const float arm_d = distance_safe / sqrtf(2.0f);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float base_thrust = sqrtf((mass_safe * gravity_safe) / (4.0f * kf_safe));

	//力到倾角的转换系数
	const float acc_to_tilt = sqrtf(2.0f) / (mass_safe * gravity_safe);

	// 根据姿态目标和水平加速度指令计算四个电机的倾转角偏置。
	alpha_offset1 =  sqrtf(2.0f)*phi_sp + sqrtf(2.0f)*theta_sp + acc_to_tilt * mass_safe * (acc_sp(0) - acc_sp(1)) / 4.0f;
	alpha_offset2 = -sqrtf(2.0f)*phi_sp - sqrtf(2.0f)*theta_sp - acc_to_tilt * mass_safe * (acc_sp(0) - acc_sp(1)) / 4.0f;
	alpha_offset3 = -sqrtf(2.0f)*phi_sp + sqrtf(2.0f)*theta_sp - acc_to_tilt * mass_safe * (acc_sp(0) + acc_sp(1)) / 4.0f;
	alpha_offset4 =  sqrtf(2.0f)*phi_sp - sqrtf(2.0f)*theta_sp + acc_to_tilt * mass_safe * (acc_sp(0) + acc_sp(1)) / 4.0f;

	// 限制倾转角，避免指令超过机构允许范围。
	alpha_offset1 = math::constrain(alpha_offset1, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset2 = math::constrain(alpha_offset2, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset3 = math::constrain(alpha_offset3, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset4 = math::constrain(alpha_offset4, -tilt_angle_max_rad, tilt_angle_max_rad);


	const auto signed_sqrt = [](float value) {
		return (value >= 0.0f) ? sqrtf(value) : -sqrtf(fabsf(value));
	};

	// 将期望角加速度和垂向加速度换算为各控制通道对应的电机角速度增量。
	const float w_roll = signed_sqrt((I_xx * ang_acc_sp(0)) / (4.0f * kf_safe * arm_d));
	const float w_pitch = signed_sqrt((I_yy * ang_acc_sp(1)) / (4.0f * kf_safe * arm_d));
	const float w_yaw = signed_sqrt((I_zz * ang_acc_sp(2)) / (4.0f * kf_safe * distance_safe));
	const float w_fz = signed_sqrt((mass_safe * (gravity_safe - acc_sp(2))) / (4.0f * kf_safe));

	// 将姿态角加速度和垂向加速度叠加到四个电机角速度命令。
	// 电机编号：1=右前，2=左后，3=左前，4=右后；偏航按 1/2 与 3/4 反向差动。
	motor_1 = -w_roll + w_pitch + w_yaw + w_fz;
	motor_2 =  w_roll - w_pitch + w_yaw + w_fz;
	motor_3 =  w_roll + w_pitch - w_yaw + w_fz;
	motor_4 = -w_roll - w_pitch - w_yaw + w_fz;

	// 限制电机角速度，防止负值或超过设定上限。
	constexpr float motor_speed_max = 20000.0f;
	motor_1 = math::constrain(motor_1, 0.0f, motor_speed_max);
	motor_2 = math::constrain(motor_2, 0.0f, motor_speed_max);
	motor_3 = math::constrain(motor_3, 0.0f, motor_speed_max);
	motor_4 = math::constrain(motor_4, 0.0f, motor_speed_max);

	// actuator_motors 期望归一化推力 [-1, 1]；内部 motor_* 仍按角速度计算，
	// 按 T ~ omega^2 映射到悬停油门附近，避免用物理角速度上限直接缩放导致输出接近 0。
	actuator_motors_s motor_speed{};
	motor_speed.timestamp_sample = hrt_absolute_time();
	motor_speed.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_speed.control[i] = NAN;
	}

	const float hover_omega = math::max(base_thrust, 1.0f);
	const float hover_throttle = math::constrain(_param_fv_hover_thr.get(), 0.05f, 0.95f);
	// omega 与推力近似满足平方关系，先相对悬停角速度归一化。
	const auto omega_to_normalized_thrust = [hover_omega, hover_throttle](float omega) {
		const float ratio = omega / hover_omega;
		return math::constrain(hover_throttle * ratio * ratio, 0.0f, 1.0f);
	};

	motor_speed.control[0] = omega_to_normalized_thrust(motor_1);
	motor_speed.control[1] = omega_to_normalized_thrust(motor_2);
	motor_speed.control[2] = omega_to_normalized_thrust(motor_3);
	motor_speed.control[3] = omega_to_normalized_thrust(motor_4);
	_motor_speed_pub_raw.publish(motor_speed);

	// actuator_servos 期望归一化位置 [-1, 1]，这里按最大倾转角归一化。
	actuator_servos_s motor_tilt{};
	motor_tilt.timestamp_sample = hrt_absolute_time();
	motor_tilt.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		motor_tilt.control[i] = NAN;
	}

	motor_tilt.control[0] = math::constrain(alpha_offset1 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[1] = math::constrain(alpha_offset2 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[2] = math::constrain(alpha_offset3 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[3] = math::constrain(alpha_offset4 / tilt_angle_max_rad, -1.0f, 1.0f);

	_motor_tilt_pub_raw.publish(motor_tilt);
}

void FullvectorControl::controlAllocation(const UAVStates & state, const UAVCommand & command)
{
	// 当前姿态用于把机体系推力转换到 NED 世界系，并进行调试用动力学积分。
	const Vector3f euler_cur = Vector3f(Eulerf(state.attitude));
	const float roll_rad = euler_cur(0);
	const float pitch_rad = euler_cur(1);
	const float yaw_rad = euler_cur(2);
	(void)roll_rad;
	(void)pitch_rad;
	(void)yaw_rad;

	// 先根据控制器输出计算电机角速度和倾转角。
	calculateMotorCommand(command);

	if (_param_print_msg_a_en.get() != 0) {
		static hrt_abstime last_ctrl_alloc_print_time{0};

		if ((last_ctrl_alloc_print_time == 0) || (hrt_elapsed_time(&last_ctrl_alloc_print_time) > 200_ms)) {
			PX4_INFO("[CtrlAlloc] motor_speed=[%.1f %.1f %.1f %.1f] alpha_rad=[%.3f %.3f %.3f %.3f]",
				 (double)motor_1, (double)motor_2, (double)motor_3, (double)motor_4,
				 (double)alpha_offset1, (double)alpha_offset2, (double)alpha_offset3, (double)alpha_offset4);
			last_ctrl_alloc_print_time = hrt_absolute_time();
		}
	}

	// 电机角速度平方用于推力和反扭矩计算。
	const float m1_sq = motor_1 * motor_1;
	const float m2_sq = motor_2 * motor_2;
	const float m3_sq = motor_3 * motor_3;
	const float m4_sq = motor_4 * motor_4;
	const float c = sqrtf(2.0f) * 0.5f;

	// 由四个倾转电机的推力分量合成机体系合力 Fx/Fy/Fz。
	const float Fx = + K_F * m1_sq * c * sinf(alpha_offset1)
			 - K_F * m2_sq * c * sinf(alpha_offset2)
			 - K_F * m3_sq * c * sinf(alpha_offset3)
			 + K_F * m4_sq * c * sinf(alpha_offset4);

	const float Fy = - K_F * m1_sq * c * sinf(alpha_offset1)
			 + K_F * m2_sq * c * sinf(alpha_offset2)
			 - K_F * m3_sq * c * sinf(alpha_offset3)
			 + K_F * m4_sq * c * sinf(alpha_offset4);

	const float Fz = + K_F * m1_sq * cosf(alpha_offset1)
			 + K_F * m2_sq * cosf(alpha_offset2)
			 + K_F * m3_sq * cosf(alpha_offset3)
			 + K_F * m4_sq * cosf(alpha_offset4);

	// 将机体系力转换为世界系加速度，NED 坐标下 z 方向含重力项。
	const float mass_safe = math::max(mass, 1e-3f);
	const float dv_x =- ((cosf(pitch_rad) * cosf(yaw_rad)) * Fx
			  + (cosf(yaw_rad) * sinf(pitch_rad) * sinf(roll_rad) - sinf(yaw_rad) * cosf(roll_rad)) * Fy
			  + (cosf(yaw_rad) * sinf(pitch_rad) * cosf(roll_rad) + sinf(yaw_rad) * sinf(roll_rad)) * Fz) / mass_safe;

	const float dv_y =- ((cosf(pitch_rad) * sinf(yaw_rad)) * Fx
			  + (sinf(yaw_rad) * sinf(pitch_rad) * sinf(roll_rad) + cosf(yaw_rad) * cosf(roll_rad)) * Fy
			  + (sinf(yaw_rad) * sinf(pitch_rad) * cosf(roll_rad) - cosf(yaw_rad) * sinf(roll_rad)) * Fz) / mass_safe;

	const float dv_z = gravity - ((-sinf(pitch_rad)) * Fx
			   + (sinf(roll_rad) * cosf(pitch_rad)) * Fy
			   + (cosf(roll_rad) * cosf(pitch_rad)) * Fz) / mass_safe;

	// 简单积分得到预测位置和速度；当前仅用于内部调试计算，暂不发布。
	const Vector3f acc_world(dv_x, dv_y, dv_z);
	const float dt = math::max(_dt, 0.0f);
	const Vector3f vel_integrated = state.velocity + acc_world * dt;
	const Vector3f pos_integrated = state.position + state.velocity * dt + 0.5f * acc_world * dt * dt;

	// 计算电机反扭矩和倾转推力通过力臂产生的机体系力矩。
	const float Qx = 0.0f;
	const float Qy = 0.0f;
	const float Qz = K_M * m1_sq * cosf(alpha_offset1)
			 + K_M * m2_sq * cosf(alpha_offset2)
			 - K_M * m3_sq * cosf(alpha_offset3)
			 - K_M * m4_sq * cosf(alpha_offset4);

	const float arm_d = distance / sqrtf(2.0f);
	const float tau_x = -K_F * m1_sq * arm_d * cosf(alpha_offset1)
			  + K_F * m2_sq * arm_d * cosf(alpha_offset2)
			  + K_F * m3_sq * arm_d * cosf(alpha_offset3)
			  - K_F * m4_sq * arm_d * cosf(alpha_offset4) + Qx;
	const float tau_y =  K_F * m1_sq * arm_d * cosf(alpha_offset1)
			  - K_F * m2_sq * arm_d * cosf(alpha_offset2)
			  + K_F * m3_sq * arm_d * cosf(alpha_offset3)
			  - K_F * m4_sq * arm_d * cosf(alpha_offset4) + Qy;
	const float tau_z = -K_F * m1_sq * distance * sinf(alpha_offset1)
			  - K_F * m2_sq * distance * sinf(alpha_offset2)
			  - K_F * m3_sq * distance * sinf(alpha_offset3)
			  - K_F * m4_sq * distance * sinf(alpha_offset4) + Qz;

	// 刚体转动动力学：由力矩、惯量和转子陀螺项估算机体系角加速度。
	const float p = state.angular_velocity(0);
	const float q = state.angular_velocity(1);
	const float r = state.angular_velocity(2);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float rotor_mix = (motor_1 + motor_2 - motor_3 - motor_4);

	const float dp = (tau_x + q * r * (I_yy - I_zz) + J_RP * q * rotor_mix) / I_xx;
	const float dq = (tau_y + p * r * (I_zz - I_xx) - J_RP * p * rotor_mix) / I_yy;
	const float dr = (tau_z + p * q * (I_xx - I_yy)) / I_zz;

	// 积分角加速度得到新的角速度。
	const Vector3f ang_acc_body(dp, dq, dr);
	const Vector3f omega_integrated = state.angular_velocity + ang_acc_body * dt;
	const float p_new = omega_integrated(0);
	const float q_new = omega_integrated(1);
	const float r_new = omega_integrated(2);

	// 将机体系角速度转换为欧拉角变化率，pitch 接近奇异点时做保护。
	const float cos_pitch = math::max(fabsf(cosf(pitch_rad)), 1e-4f);
	const float tan_pitch = sinf(pitch_rad) / cos_pitch;
	const float dphi = p_new + q_new * tan_pitch * sinf(roll_rad) + r_new * tan_pitch * cosf(roll_rad);
	const float dtheta = q_new * cosf(roll_rad) - r_new * sinf(roll_rad);
	const float dpsi = (q_new * sinf(roll_rad) + r_new * cosf(roll_rad)) / cos_pitch;

	// 积分欧拉角并转换回四元数；当前结果仅用于调试占位，避免影响估计器状态。
	const Vector3f euler_integrated = euler_cur + Vector3f(dphi, dtheta, dpsi) * dt;
	const Quatf q_integrated(Eulerf(euler_integrated(0), euler_integrated(1), euler_integrated(2)));
	(void)pos_integrated;
	(void)vel_integrated;
	(void)q_integrated;
	(void)omega_integrated;
	(void)ang_acc_body;
}

int FullvectorControl::task_spawn(int argc, char *argv[])
{
	// PX4 模块启动入口：创建控制器实例并挂到 WorkQueue。
	FullvectorControl *instance = new FullvectorControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	// 初始化失败时释放对象并返回错误。
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FullvectorControl::custom_command(int argc, char *argv[])
{
	// 当前模块没有自定义子命令，统一打印用法说明。
	return print_usage("unknown command");
}

int FullvectorControl::print_usage(const char *reason)
{
	// 输出模块说明和 PX4 标准 start/stop/status 用法。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	FullvectorControl module for testing geometric control algorithms.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fullvector_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fullvector_control_main(int argc, char *argv[])
{
	// C 链接入口，供 PX4 模块加载器按模块名调用。
	return FullvectorControl::main(argc, argv);
}
