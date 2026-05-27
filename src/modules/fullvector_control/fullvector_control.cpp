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
 * Full-vector quadcopter controller with position controller, attitude controller
 * and motor mixing calculations, using PID control.
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
	parameters_update(true);
	_current_state.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.velocity = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.attitude = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_current_state.angular_velocity = Vector3f(0.0f, 0.0f, 0.0f);

	// 初始化命令（默认使用参数后备目标）
	_current_command.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
}

FullvectorControl::~FullvectorControl()
{
	perf_free(_loop_perf);
}

bool FullvectorControl::init()
{
	ScheduleNow();
	return true;
}

void FullvectorControl::resetPidState()
{
	_pos_error_int.zero();
	_vel_error_int.zero();
	_pid_state_initialized = false;

	_att_error_int.zero();
	_ang_vel_error_int.zero();
	_att_pid_state_initialized = false;
}

void FullvectorControl::publishSafeActuatorFallback()
{
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
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();

		// 控制器PID参数矩阵：行=x/y/z，列=P/I/D
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

		//飞行器参数
		mass = _param_fv_mass.get();
		gravity = _param_fv_gravity.get();
		distance = _param_fv_motor_distance.get();

		//电机拉力系数和力矩系数
		K_F = _param_fv_K_F.get();
		K_M = _param_fv_K_M.get();

		//惯性矩阵初始化为对角矩阵，非对角项为0
		inertia.setZero();
		inertia(0, 0) = _param_fv_inertia_xx.get();
		inertia(1, 1) = _param_fv_inertia_yy.get();
		inertia(2, 2) = _param_fv_inertia_zz.get();

		//整个电机转子和螺旋桨绕转轴的总转动惯量
		J_RP = _param_fv_J_RP.get();

		_current_command.position = Vector3f(_param_fv_target_x.get(), _param_fv_target_y.get(), _param_fv_target_z.get());
		_current_command.Euler_angles = Vector3f(_param_fv_target_roll.get(), _param_fv_target_pitch.get(), _param_fv_target_yaw.get());

		// 参数更新后清空积分项，避免旧状态引入突变
		resetPidState();
	}

}

bool FullvectorControl::updateUAVState()
{
	_state_age_level = 0;

	// 1. 更新位置和速度
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

	// 2. 更新姿态
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_attitude);
		_current_state.attitude = matrix::Quatf(_attitude.q);
		_last_attitude_update = _attitude.timestamp;
	}

	// 3. 更新角速度
	if (_vehicle_angular_velocity_sub.updated()) {
		_vehicle_angular_velocity_sub.copy(&_angular_velocity);
		_current_state.angular_velocity = Vector3f(_angular_velocity.xyz[0],
								   _angular_velocity.xyz[1],
								   _angular_velocity.xyz[2]);
		_last_angular_velocity_update = _angular_velocity.timestamp;
	}

	// 所有状态都必须已初始化且不过期，避免混合不同时间的观测量
	if ((_last_position_update == 0) || (_last_velocity_update == 0)
	    || (_last_attitude_update == 0) || (_last_angular_velocity_update == 0)) {
		return false;
	}

	const hrt_abstime elapsed_position = hrt_elapsed_time(&_last_position_update);
	const hrt_abstime elapsed_velocity = hrt_elapsed_time(&_last_velocity_update);
	const hrt_abstime elapsed_attitude = hrt_elapsed_time(&_last_attitude_update);
	const hrt_abstime elapsed_ang_vel = hrt_elapsed_time(&_last_angular_velocity_update);

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
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// keep periodic execution even when control output is gated
	ScheduleDelayed(10_ms);

	parameters_update(false);

	_vehicle_control_mode_sub.update(&_control_mode);

	const bool fv_enabled = (_param_fv_enable.get() == 1);
	const bool armed = _control_mode.flag_armed;
	const bool controller_active = fv_enabled && armed;

	if (!controller_active) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		return;
	}

	if (!_controller_was_active) {
		resetPidState();
		_last_run_time = 0;
	}

	_controller_was_active = true;

	const hrt_abstime now = hrt_absolute_time();
	_current_command.position = Vector3f(_param_fv_target_x.get(), _param_fv_target_y.get(), _param_fv_target_z.get());
	_current_command.Euler_angles = Vector3f(_param_fv_target_roll.get(), _param_fv_target_pitch.get(), _param_fv_target_yaw.get());

	// 计算时间间隔
	if (_last_run_time == 0) {
		_dt = 0.01f;

	} else {
		_dt = (now - _last_run_time) / 1e6f;  // 转换为秒
	}

	_last_run_time = now;

	if (!PX4_ISFINITE(_dt) || (_dt <= FLT_EPSILON)) {
		return;
	}

	constexpr float dt_clamp_s = 0.05f;
	constexpr float dt_reset_s = 0.1f;

	// Guard derivative terms from scheduler jitter spikes.
	if (_dt > dt_reset_s) {
		resetPidState();
		_dt = 0.01f;

	} else if (_dt > dt_clamp_s) {
		_dt = dt_clamp_s;
	}

	const bool debug_print_enabled = (_param_print_msg_a_en.get() != 0);
	const bool allow_debug_print = debug_print_enabled
					      && ((_last_debug_print_time == 0) || (hrt_elapsed_time(&_last_debug_print_time) > 200_ms));

	// 关键：状态没有更新时不继续控制，避免使用过期状态闭环
	if (!updateUAVState()) {
		publishSafeActuatorFallback();

		if (allow_debug_print) {
			PX4_WARN("state unavailable, publishing safe actuator fallback");
			_last_debug_print_time = now;
		}

		return;
	}

	if (_state_age_level >= 2) {
		publishSafeActuatorFallback();

		if (allow_debug_print) {
			PX4_WARN("state stale-fail, publishing safe actuator fallback");
			_last_debug_print_time = now;
		}

		return;
	}

	// Keep estimator state as control source even in aging mode to avoid recursive model drift.
	const UAVStates &state_for_control = _current_state;

	perf_begin(_loop_perf);


	if (allow_debug_print) {
		PX4_INFO("debug value=%.3f", (double)_param_print_num_value.get());
	}

	vehicle_thrust_setpoint_s thrust_feedback{};
	const bool thrust_updated = _vehicle_thrust_setpoint_sub.update(&thrust_feedback);

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

	// Diagnostic: check takeoff and land detection status
	takeoff_status_s takeoff_status{};
	vehicle_land_detected_s land_detected{};
	_takeoff_status_sub.update(&takeoff_status);
	_vehicle_land_detected_sub.update(&land_detected);

	if (allow_debug_print) {
		PX4_INFO("[FlightState] takeoff_state=%d landed=%d ground_contact=%d pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f]",
			 (int)takeoff_status.takeoff_state,
			 (int)land_detected.landed,
			 (int)land_detected.ground_contact,
			 (double)_current_state.position(0), (double)_current_state.position(1), (double)_current_state.position(2),
			 (double)_current_state.velocity(0), (double)_current_state.velocity(1), (double)_current_state.velocity(2));
		_last_debug_print_time = now;
	}

	// 1. 执行位置控制
	PositionControl(state_for_control, _current_command, _dt);

	// 2. 执行姿态控制
	AttitudeControl(state_for_control, _current_command, _dt);

	// 3. 执行力与力矩分配计算函数（其中包括电机角速度与偏移角计算）
	controlAllocation(state_for_control, _current_command);

	 perf_end(_loop_perf);

}

//位置控制器函数（串级PID控制器）
void FullvectorControl::PositionControl(const UAVStates & state, const UAVCommand & command, const float dt)
{
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	// 位置外环：根据位置误差生成期望速度
	const Vector3f ep = command.position - state.position;

	if (!_pid_state_initialized) {
		_pos_error_prev = ep;
		_vel_error_prev.zero();
		_pid_state_initialized = true;
	}

	const Vector3f dep = (ep - _pos_error_prev) / dt;
	_pos_error_int += ep * dt;

	// 积分限幅，防止积分饱和
	for (int i = 0; i < 3; i++) {
		_pos_error_int(i) = math::constrain(_pos_error_int(i), -5.0f, 5.0f);
	}

	const Vector3f pos_kp{gain_pos_pid(0, 0), gain_pos_pid(1, 0), gain_pos_pid(2, 0)};
	const Vector3f pos_ki{gain_pos_pid(0, 1), gain_pos_pid(1, 1), gain_pos_pid(2, 1)};
	const Vector3f pos_kd{gain_pos_pid(0, 2), gain_pos_pid(1, 2), gain_pos_pid(2, 2)};

	Vector3f v_sp = pos_kp.emult(ep) + pos_ki.emult(_pos_error_int) + pos_kd.emult(dep);

	// 期望速度限幅
	for (int i = 0; i < 3; i++) {
		v_sp(i) = math::constrain(v_sp(i), -5.0f, 5.0f);
	}

	// 速度内环：根据速度误差生成期望加速度
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
	_pos_acc_cmd = acc_cmd;

	_pos_error_prev = ep;
	_vel_error_prev = ev;

	// 仅发布位置串级PID输出
	vehicle_local_position_setpoint_s position_controller_output{};
	position_controller_output.timestamp_sample = hrt_absolute_time();
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

//姿态控制器函数（串级PID控制器）
void FullvectorControl::AttitudeControl(const UAVStates & state, UAVCommand & command, const float dt)
{
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	// 当前姿态由旋转矩阵转为欧拉角，顺序为 roll/pitch/yaw
	const Vector3f euler_cur = Vector3f(Eulerf(state.attitude));
	const Vector3f euler_sp(command.Euler_angles(0), command.Euler_angles(1), command.Euler_angles(2));

	// 姿态外环：根据姿态误差生成期望角速度
	Vector3f e_att = euler_sp - euler_cur;
	e_att(0) = math::wrap_pi(e_att(0));
	e_att(1) = math::wrap_pi(e_att(1));
	e_att(2) = math::wrap_pi(e_att(2));

	if (!_att_pid_state_initialized) {
		_att_error_prev = e_att;
		_ang_vel_error_prev.zero();
		_att_pid_state_initialized = true;
	}

	const Vector3f de_att = (e_att - _att_error_prev) / dt;
	_att_error_int += e_att * dt;

	for (int i = 0; i < 3; i++) {
		_att_error_int(i) = math::constrain(_att_error_int(i), -1.0f, 1.0f);
	}

	const Vector3f att_kp{gain_att_pid(0, 0), gain_att_pid(1, 0), gain_att_pid(2, 0)};
	const Vector3f att_ki{gain_att_pid(0, 1), gain_att_pid(1, 1), gain_att_pid(2, 1)};
	const Vector3f att_kd{gain_att_pid(0, 2), gain_att_pid(1, 2), gain_att_pid(2, 2)};

	const Vector3f omega_sp = att_kp.emult(e_att) + att_ki.emult(_att_error_int) + att_kd.emult(de_att);
	command.angular_velocity = omega_sp;

	// 角速度内环：根据角速度误差生成期望角加速度
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
	_att_ang_acc_cmd = ang_acc_cmd;

	_att_error_prev = e_att;
	_ang_vel_error_prev = e_w;

	// 仅发布姿态串级PID输出
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

//电机角度偏移及角速度计算函数
void FullvectorControl::calculateMotorCommand(const UAVCommand & command)
{
	// 读取期望姿态角
	const float phi_sp = command.Euler_angles(0);
	const float theta_sp = command.Euler_angles(1);
	const float psi_sp = command.Euler_angles(2);
	(void)psi_sp;

	// 调用位置控制器和姿态控制器输出
	const Vector3f &acc_sp = _pos_acc_cmd;
	const Vector3f &ang_acc_sp = _att_ang_acc_cmd;

	// 计算电机角度偏移
	alpha_offset1 =  sqrt(2.0f)*phi_sp + sqrt(2.0f)*theta_sp - (acc_sp(0) - acc_sp(1)) / 4.0f;
	alpha_offset2 = -sqrt(2.0f)*phi_sp - sqrt(2.0f)*theta_sp + (acc_sp(0) - acc_sp(1)) / 4.0f;
	alpha_offset3 = -sqrt(2.0f)*phi_sp + sqrt(2.0f)*theta_sp + (acc_sp(0) + acc_sp(1)) / 4.0f;
	alpha_offset4 =  sqrt(2.0f)*phi_sp - sqrt(2.0f)*theta_sp - (acc_sp(0) + acc_sp(1)) / 4.0f;

	// 电机偏转角限幅（rad）
	alpha_offset1 = math::constrain(alpha_offset1, -0.52f, 0.52f);
	alpha_offset2 = math::constrain(alpha_offset2, -0.52f, 0.52f);
	alpha_offset3 = math::constrain(alpha_offset3, -0.52f, 0.52f);
	alpha_offset4 = math::constrain(alpha_offset4, -0.52f, 0.52f);

	// 计算悬停基础推力对应的电机旋转角速度
	const float kf_safe = math::max(K_F, 1e-6f);
	const float base_thrust = sqrtf((mass * gravity) / (4.0f * kf_safe));
	(void)base_thrust;

	//计算电机推力对应的电机旋转角速度
	motor_1 = base_thrust - ang_acc_sp(0) + ang_acc_sp(1) + ang_acc_sp(2) + acc_sp(2);
	motor_2 = base_thrust + ang_acc_sp(0) - ang_acc_sp(1) + ang_acc_sp(2) + acc_sp(2);
	motor_3 = base_thrust + ang_acc_sp(0) + ang_acc_sp(1) - ang_acc_sp(2) + acc_sp(2);
	motor_4 = base_thrust - ang_acc_sp(0) - ang_acc_sp(1) - ang_acc_sp(2) + acc_sp(2);

	// 电机旋转角速度限幅
	constexpr float motor_speed_max = 20000.0f;
	motor_1 = math::constrain(motor_1, 0.0f, motor_speed_max);
	motor_2 = math::constrain(motor_2, 0.0f, motor_speed_max);
	motor_3 = math::constrain(motor_3, 0.0f, motor_speed_max);
	motor_4 = math::constrain(motor_4, 0.0f, motor_speed_max);

	// 发布电机旋转角速度（raw omega，非归一化）
	actuator_motors_s motor_speed{};
	motor_speed.timestamp_sample = hrt_absolute_time();
	motor_speed.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_speed.control[i] = NAN;
	}
	motor_speed.control[0] = motor_1;
	motor_speed.control[1] = motor_2;
	motor_speed.control[2] = motor_3;
	motor_speed.control[3] = motor_4;
	_motor_speed_pub_raw.publish(motor_speed);

	// 发布电机偏转角（raw rad，直接发布弧度值）
	actuator_servos_s motor_tilt{};
	motor_tilt.timestamp_sample = hrt_absolute_time();
	motor_tilt.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		motor_tilt.control[i] = NAN;
	}

	motor_tilt.control[0] = alpha_offset1;
	motor_tilt.control[1] = alpha_offset2;
	motor_tilt.control[2] = alpha_offset3;
	motor_tilt.control[3] = alpha_offset4;

	_motor_tilt_pub_raw.publish(motor_tilt);

}

//控制分配函数
void FullvectorControl::controlAllocation(const UAVStates & state, const UAVCommand & command)
{
	// 在控制分配函数内读取当前姿态角（rad）
	const Vector3f euler_cur = Vector3f(Eulerf(state.attitude));
	const float roll_rad = euler_cur(0);
	const float pitch_rad = euler_cur(1);
	const float yaw_rad = euler_cur(2);
	(void)roll_rad;
	(void)pitch_rad;
	(void)yaw_rad;

	// 控制分配阶段调用 raw 执行器命令计算（omega + rad）
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

	// 计算三个方向的力
	const float m1_sq = motor_1 * motor_1;
	const float m2_sq = motor_2 * motor_2;
	const float m3_sq = motor_3 * motor_3;
	const float m4_sq = motor_4 * motor_4;
	const float c = sqrtf(2.0f) * 0.5f;

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

	// 计算世界系加速度
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

	// 加速度两次积分得到位置：a -> v -> p
	const Vector3f acc_world(dv_x, dv_y, dv_z);
	const float dt = math::max(_dt, 0.0f);
	const Vector3f vel_integrated = state.velocity + acc_world * dt;
	const Vector3f pos_integrated = state.position + state.velocity * dt + 0.5f * acc_world * dt * dt;

	// 计算机体扭矩
	// 反扭距（x,y方向相互抵消）
	const float Qx = 0.0f;
	const float Qy = 0.0f;
	const float Qz = K_M * m1_sq * cosf(alpha_offset1)
			 + K_M * m2_sq * cosf(alpha_offset2)
			 - K_M * m3_sq * cosf(alpha_offset3)
			 - K_M * m4_sq * cosf(alpha_offset4);

	// 拉力作用下机体扭矩
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
			  + K_F * m2_sq * distance * sinf(alpha_offset2)
			  + K_F * m3_sq * distance * sinf(alpha_offset3)
			  - K_F * m4_sq * distance * sinf(alpha_offset4) + Qz;

	// 计算姿态动力学方程，忽略舵机转动扭矩，只考虑陀螺力矩和机体扭矩
	const float p = state.angular_velocity(0);
	const float q = state.angular_velocity(1);
	const float r = state.angular_velocity(2);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float rotor_mix = (motor_1 - motor_2 + motor_3 - motor_4);

	const float dp = (tau_x + q * r * (I_yy - I_zz) + J_RP * q * rotor_mix) / I_xx;
	const float dq = (tau_y + p * r * (I_zz - I_xx) - J_RP * p * rotor_mix) / I_yy;
	const float dr = (tau_z + p * q * (I_xx - I_yy)) / I_zz;

	// 角加速度积分得到角速度
	const Vector3f ang_acc_body(dp, dq, dr);
	const Vector3f omega_integrated = state.angular_velocity + ang_acc_body * dt;
	const float p_new = omega_integrated(0);
	const float q_new = omega_integrated(1);
	const float r_new = omega_integrated(2);

	// 角速度通过欧拉角转换关系得到角度
	const float cos_pitch = math::max(fabsf(cosf(pitch_rad)), 1e-4f);
	const float tan_pitch = sinf(pitch_rad) / cos_pitch;
	const float dphi = p_new + q_new * tan_pitch * sinf(roll_rad) + r_new * tan_pitch * cosf(roll_rad);
	const float dtheta = q_new * cosf(roll_rad) - r_new * sinf(roll_rad);
	const float dpsi = (q_new * sinf(roll_rad) + r_new * cosf(roll_rad)) / cos_pitch;

	//更新飞行器状态中的姿态角（欧拉角）和四元数
	const Vector3f euler_integrated = euler_cur + Vector3f(dphi, dtheta, dpsi) * dt;
	const Quatf q_integrated(Eulerf(euler_integrated(0), euler_integrated(1), euler_integrated(2)));

	// 发布积分后的状态量到对应uORB消息，供后续状态更新链路使用
	const hrt_abstime now = hrt_absolute_time();

	vehicle_local_position_s local_position_out = _position;
	local_position_out.timestamp_sample = now;
	local_position_out.timestamp = now;
	local_position_out.xy_valid = true;
	local_position_out.z_valid = true;
	local_position_out.v_xy_valid = true;
	local_position_out.v_z_valid = true;
	local_position_out.x = pos_integrated(0);
	local_position_out.y = pos_integrated(1);
	local_position_out.z = pos_integrated(2);
	local_position_out.vx = vel_integrated(0);
	local_position_out.vy = vel_integrated(1);
	local_position_out.vz = vel_integrated(2);
	local_position_out.z_deriv = vel_integrated(2);
	local_position_out.ax = acc_world(0);
	local_position_out.ay = acc_world(1);
	local_position_out.az = acc_world(2);
	local_position_out.heading = euler_integrated(2);
	local_position_out.heading_good_for_control = true;
	_vehicle_local_position_pub.publish(local_position_out);

	vehicle_attitude_s attitude_out = _attitude;
	attitude_out.timestamp_sample = now;
	attitude_out.timestamp = now;
	attitude_out.q[0] = q_integrated(0);
	attitude_out.q[1] = q_integrated(1);
	attitude_out.q[2] = q_integrated(2);
	attitude_out.q[3] = q_integrated(3);
	_vehicle_attitude_pub.publish(attitude_out);

	vehicle_angular_velocity_s angular_velocity_out = _angular_velocity;
	angular_velocity_out.timestamp_sample = now;
	angular_velocity_out.timestamp = now;
	angular_velocity_out.xyz[0] = omega_integrated(0);
	angular_velocity_out.xyz[1] = omega_integrated(1);
	angular_velocity_out.xyz[2] = omega_integrated(2);
	angular_velocity_out.xyz_derivative[0] = ang_acc_body(0);
	angular_velocity_out.xyz_derivative[1] = ang_acc_body(1);
	angular_velocity_out.xyz_derivative[2] = ang_acc_body(2);
	_vehicle_angular_velocity_pub.publish(angular_velocity_out);

}

int FullvectorControl::task_spawn(int argc, char *argv[])
{
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

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FullvectorControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FullvectorControl::print_usage(const char *reason)
{
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
	return FullvectorControl::main(argc, argv);
}
