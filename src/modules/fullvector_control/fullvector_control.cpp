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
 * @file fullvector_control.cpp
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
#include <px4_platform_common/log.h>
#include <commander/px4_custom_mode.h>

using namespace time_literals;
using namespace matrix;

FullvectorControl::FullvectorControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化参数、状态和目标缓存。
	parameters_update(true);

	_current_state.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.velocity = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.attitude = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_current_state.angular_velocity = Vector3f(0.0f, 0.0f, 0.0f);

	_current_command.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
}

//模块退出时释放控制循环的性能监测计数器资源
FullvectorControl::~FullvectorControl()
{
	perf_free(_loop_perf);
}

bool FullvectorControl::init()
{
	// 启动控制循环。
	ScheduleNow();
	return true;
}

void FullvectorControl::resetPositionPidState()
{
	// 重置位置和速度环状态。
	_pos_error_int.zero();
	_vel_error_int.zero();
	_pid_state_initialized = false;

	//解除三轴位置锁定
	for (bool &axis_locked : _position_axis_locked) {
		axis_locked = false;
	}

	//清除相对位姿模式记录+高度保持及垂直控制缓存
	_position_outer_uses_relative_pose = false;
	_posctl_z_hold_active = false;
	_vertical_velocity_feedback = 0.0f;
	_vertical_velocity_integral_acceleration = 0.0f;
	// 清除位置通道的分配残差和饱和记录。
	_position_anti_windup_active = false;
	_allocation_accel_residual.zero();

	for (bool &saturated : _position_allocation_saturated) {
		saturated = false;
	}
}

void FullvectorControl::resetAttitudePidState()
{
	// 重置姿态、角速度和航向目标。
	_att_error_int.zero();
	_ang_vel_error_int.zero();
	_att_pid_state_initialized = false;

	//清除相对姿态控制模式记录+丢弃旧航向目标
	_attitude_outer_uses_relative_pose = false;
	_posctl_yaw_sp_initialized = false;
	_manual_yaw_sp_initialized = false;
	_manual_yaw_stick_active = false;
	// 清除姿态通道的分配残差和饱和记录。
	_attitude_anti_windup_active = false;
	_allocation_ang_acc_residual.zero();

	for (bool &saturated : _attitude_allocation_saturated) {
		saturated = false;
	}
}

void FullvectorControl::resetPidState()
{
	resetPositionPidState();
	resetAttitudePidState();
}

void FullvectorControl::publishSafeActuatorFallback()
{
	// 电机停转，倾转舵机回中。
	actuator_motors_s motor_safe{};
	motor_safe.timestamp_sample = hrt_absolute_time();
	motor_safe.timestamp = hrt_absolute_time();

	// NaN 表示不接管其余执行器通道。
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
	_last_actuator_output_valid = false;
}

void FullvectorControl::publishNeutralTiltServos()
{
	// 仅让倾转舵机回中，电机交给原生控制器。
	actuator_servos_s tilt_neutral{};

	tilt_neutral.timestamp_sample = hrt_absolute_time();
	tilt_neutral.timestamp = tilt_neutral.timestamp_sample;

	// 前 4 路舵机设为 0.0f，其余通道保持 NaN。
	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		tilt_neutral.control[i] = NAN;
	}

	for (int i = 0; i < 4 && i < actuator_servos_s::NUM_CONTROLS; i++) {
		tilt_neutral.control[i] = 0.0f;
	}

	_motor_tilt_pub_raw.publish(tilt_neutral);
	_last_actuator_output_valid = false;
}

void FullvectorControl::publishFullvectorControlStatus(bool fullvector_active, bool native_requested,
		bool rc_switch_valid, float rc_switch_value)
{
	// 创建并初始化状态消息，发布控制权和相对位姿状态。
	fullvector_control_status_s status{};

	// 记录当前时间戳和控制权及RC开关状态。
	status.timestamp = hrt_absolute_time();
	status.fullvector_active = fullvector_active;
	status.native_requested = native_requested;
	status.rc_switch_valid = rc_switch_valid;
	status.rc_switch_value = rc_switch_valid ? rc_switch_value : NAN;

	//记录相对位姿控制状态及目标信息
	status.relative_pose_valid = _target_relative_pose_valid;
	status.relative_pose_active = _relative_pose_active;
	status.relative_pose_loss_hold = _relative_pose_loss_hold;
	status.relative_pose_hold_timed_out = _relative_pose_hold_timed_out;
	status.relative_attitude_mode = static_cast<uint8_t>(math::constrain(_param_fv_rel_att_mode.get(),
					int32_t{0}, int32_t{1}));
	status.target_id = _target_pose_target_id;
	status.target_pose_timestamp_sample = _target_pose_timestamp_sample;
	status.target_pose_time_offset = _target_pose_time_offset;
	status.target_pose_receive_age = _target_pose_receive_age;
	status.relative_pose_loss_duration = _relative_pose_loss_duration;
	status.relative_pose_reject_reason = _relative_pose_reject_reason;
	status.relative_pose_reject_count = _relative_pose_reject_count;

	// 计算相对位姿样本年龄。
	if ((_target_pose_timestamp_sample > 0)
	    && (_target_pose_timestamp_sample <= status.timestamp)) {
		status.target_pose_age = status.timestamp - _target_pose_timestamp_sample;
	}

	//记录定点模式下垂向保持控制诊断量
	status.posctl_z_hold_active = _posctl_z_hold_active;
	status.vertical_velocity_feedback = _vertical_velocity_feedback;
	status.vertical_velocity_integral_acceleration = _vertical_velocity_integral_acceleration;
	// 记录分配器饱和、实际能力和 anti-windup 状态。
	status.allocation_saturation_flags = _allocation_saturation_flags;
	status.allocation_differential_scale = _allocation_differential_scale;
	status.position_anti_windup_active = _position_anti_windup_active;
	status.attitude_anti_windup_active = _attitude_anti_windup_active;

	for (int i = 0; i < 3; i++) {
		status.acceleration_setpoint[i] = _pos_acc_cmd(i);
		status.acceleration_achieved[i] = _allocation_accel_achieved(i);
		status.angular_acceleration_setpoint[i] = _att_ang_acc_cmd(i);
		status.angular_acceleration_achieved[i] = _allocation_ang_acc_achieved(i);
	}

	//发布状态消息
	_fullvector_control_status_pub.publish(status);
}

// 对接失联保持超时后，限频请求切回 POSCTL。
void FullvectorControl::requestPositionControlFallback(hrt_abstime now)
{
	constexpr hrt_abstime retry_interval = 500_ms;

	if ((_last_fallback_request != 0) && ((now - _last_fallback_request) < retry_interval)) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.param1 = 1.0f; // 启用 PX4 自定义主模式字段。
	command.param2 = static_cast<float>(PX4_CUSTOM_MAIN_MODE_POSCTL);
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.source_system = _vehicle_status.system_id;
	command.target_system = _vehicle_status.system_id;
	command.source_component = _vehicle_status.component_id;
	command.target_component = _vehicle_status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_fallback_request = now;
}

// 检查相对位姿标志、数值范围和相邻样本跳变。
bool FullvectorControl::validateRelativePoseSample(const target_relative_pose_s &candidate, hrt_abstime now)
{
	// 先保存原始样本时间信息，便于记录被拒绝样本。
	uint8_t reject_reason = fullvector_control_status_s::REL_POSE_REJECT_NONE;
	_target_pose_timestamp_sample = candidate.timestamp_sample;
	_target_pose_target_id = candidate.target_id;
	_target_pose_time_offset = (candidate.timestamp_sample > 0) ?
				   static_cast<int64_t>(candidate.timestamp_sample) - static_cast<int64_t>(now) : 0;

	// 检查发布端有效标志。
	if (!candidate.position_valid || !candidate.orientation_valid) {
		reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_FLAGS;
	}

	// 拒绝非有限位置或四元数。
	const Vector3f candidate_position(candidate.position);
	const bool position_finite = candidate_position.isAllFinite();
	const bool quaternion_finite = PX4_ISFINITE(candidate.q[0]) && PX4_ISFINITE(candidate.q[1])
				       && PX4_ISFINITE(candidate.q[2]) && PX4_ISFINITE(candidate.q[3]);

	if (!position_finite || !quaternion_finite) {
		reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_NONFINITE;
	}

	// 四元数必须接近单位模长。
	const float quaternion_norm_sq = candidate.q[0] * candidate.q[0] + candidate.q[1] * candidate.q[1]
					 + candidate.q[2] * candidate.q[2] + candidate.q[3] * candidate.q[3];

	if (!quaternion_finite || (fabsf(quaternion_norm_sq - 1.0f) >= 0.05f)) {
		reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_QUATERNION;
	}

	const bool basic_valid = reject_reason == fullvector_control_status_s::REL_POSE_REJECT_NONE;

	// 基础检查通过后再执行目标锁定和运动学跳变门控。
	if (basic_valid && (_last_accepted_target_pose_update != 0)) {
		const float sample_dt = math::constrain((now - _last_accepted_target_pose_update) / 1e6f, 0.0f, 1.0f);

		if (_relative_pose_session_active && (candidate.target_id != _target_relative_pose.target_id)) {
			reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_TARGET_CHANGE;

		} else if (candidate.target_id == _target_relative_pose.target_id) {
			const float position_limit = math::max(_param_fv_rel_pos_jump.get(), 0.02f)
						   + math::max(_param_fv_rel_velocity_gate.get(), 0.1f) * sample_dt;

			if ((candidate_position - Vector3f(_target_relative_pose.position)).norm() > position_limit) {
				reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_POSITION_JUMP;
			}

			const float quaternion_dot = fabsf(candidate.q[0] * _target_relative_pose.q[0]
							 + candidate.q[1] * _target_relative_pose.q[1]
							 + candidate.q[2] * _target_relative_pose.q[2]
							 + candidate.q[3] * _target_relative_pose.q[3]);
			const float attitude_jump = 2.0f * acosf(math::constrain(quaternion_dot, 0.0f, 1.0f));
			const float attitude_limit = math::max(_param_fv_rel_angle_jump.get(), 0.02f)
						   + math::max(_param_fv_rel_rate_gate.get(), 0.1f) * sample_dt;

			if (attitude_jump > attitude_limit) {
				reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_ATTITUDE_JUMP;
			}
		}
	}

	// 累计拒绝原因；只有完全通过的样本才替换控制输入。
	_relative_pose_reject_reason = reject_reason;

	if (reject_reason != fullvector_control_status_s::REL_POSE_REJECT_NONE) {
		_relative_pose_reject_count++;
		return false;
	}

	_target_relative_pose = candidate;
	_last_accepted_target_pose_update = now;
	return true;
}

// 使用受限输出与请求输出的差值反算修正积分状态。
bool FullvectorControl::applyAntiWindup(Vector3f &integral_state, const Vector3f &output_residual,
					const Vector3f &integral_gain, const bool saturated[3], float dt)
{
	const float anti_windup_gain = math::constrain(_param_fv_aw_gain.get(), 0.0f, 10.0f);
	bool active = false;

	if (!PX4_ISFINITE(dt) || (dt <= FLT_EPSILON) || (anti_windup_gain <= FLT_EPSILON)) {
		return false;
	}

	// 仅在对应轴饱和且积分增益有效时执行反算。
	for (int i = 0; i < 3; i++) {
		if (saturated[i] && PX4_ISFINITE(output_residual(i)) && (fabsf(integral_gain(i)) > FLT_EPSILON)) {
			integral_state(i) += anti_windup_gain * output_residual(i) / integral_gain(i) * dt;
			active = true;
		}
	}

	return active;
}

bool FullvectorControl::evaluateNativeControllerRequest(float &rc_switch_value, bool &rc_switch_valid)
{
	// 读取 AUX 拨杆并判断原生控制器请求。
	rc_switch_value = NAN;
	rc_switch_valid = false;

	// 未启用或手动输入无效时保持 fullvector 控制权。
	if (!_param_fv_rc_sw_en.get()) {
		return false;
	}

	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	if (!_manual_control_setpoint.valid) {
		return false;
	}

	const int channel = math::constrain(_param_fv_rc_sw_ch.get(), int32_t{1}, int32_t{6});
	float value = NAN;

	// 将参数通道号映射到 AUX1～AUX6。
	switch (channel) {
	case 1: value = _manual_control_setpoint.aux1; break;

	case 2: value = _manual_control_setpoint.aux2; break;

	case 3: value = _manual_control_setpoint.aux3; break;

	case 4: value = _manual_control_setpoint.aux4; break;

	case 5: value = _manual_control_setpoint.aux5; break;

	case 6: value = _manual_control_setpoint.aux6; break;

	default: break;
	}

	if (!PX4_ISFINITE(value)) {
		return false;
	}

	rc_switch_value = math::constrain(value, -1.0f, 1.0f);
	rc_switch_valid = true;

	const bool high_position = rc_switch_value > _param_fv_rc_sw_thr.get();

	// 反向参数用于适配不同的拨杆方向。
	return _param_fv_rc_sw_rev.get() ? !high_position : high_position;
}

bool FullvectorControl::publishLastActuatorCommand()
{
	//检查上一拍输出是否有效
	if (!_last_actuator_output_valid) {
		return false;
	}

	// 刷新时间戳并重发上一拍输出。
	const hrt_abstime now = hrt_absolute_time();
	_last_motor_output.timestamp_sample = now;
	_last_motor_output.timestamp = now;
	_last_tilt_output.timestamp_sample = now;
	_last_tilt_output.timestamp = now;

	_motor_speed_pub_raw.publish(_last_motor_output);
	_motor_tilt_pub_raw.publish(_last_tilt_output);
	return true;
}

void FullvectorControl::parameters_update(bool force)
{
	// 同步参数并重置 PID 历史。
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();

		// 位置外环：位置误差到期望速度。
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

		// 速度内环：速度误差到期望加速度。
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

		// 姿态外环：欧拉角误差到期望角速度。
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

		// 角速度内环：角速度误差到期望角加速度。
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

		// 更新飞行器物理参数。
		mass = _param_fv_mass.get();
		gravity = _param_fv_gravity.get();
		distance = _param_fv_motor_distance.get();
		K_F = _param_fv_K_F.get();
		K_M = _param_fv_K_M.get();

		// 更新惯性矩阵。
		inertia.setZero();
		inertia(0, 0) = _param_fv_inertia_xx.get();
		inertia(1, 1) = _param_fv_inertia_yy.get();
		inertia(2, 2) = _param_fv_inertia_zz.get();

		J_RP = _param_fv_J_RP.get();

		// 重置 PID 历史状态。
		resetPidState();
	}
}

bool FullvectorControl::updateUAVState()
{
	_state_age_level = 0;
	_position_state_age_level = 0;
	_attitude_state_age_level = 0;

	// 更新位置和速度。
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

	updateAttitudeAndAngularVelocity();

	// 姿态和角速度是最小闭环状态。
	if ((_last_attitude_update == 0) || (_last_angular_velocity_update == 0)) {
		return false;
	}

	// 位置和速度是可选闭环状态，缺失时仅使用姿态环。
	const bool position_available = (_last_position_update != 0) && (_last_velocity_update != 0);
	const hrt_abstime elapsed_attitude = hrt_elapsed_time(&_last_attitude_update);
	const hrt_abstime elapsed_ang_vel = hrt_elapsed_time(&_last_angular_velocity_update);

	// 按消息年龄划分告警、保持和失效等级。
	constexpr hrt_abstime stale_warn_timeout = 200_ms;
	constexpr hrt_abstime stale_hold_timeout = 500_ms;
	constexpr hrt_abstime stale_fail_timeout = 1500_ms;
	const hrt_abstime elapsed_position = position_available ? hrt_elapsed_time(&_last_position_update) :
					     (stale_fail_timeout + 1);
	const hrt_abstime elapsed_velocity = position_available ? hrt_elapsed_time(&_last_velocity_update) :
					     (stale_fail_timeout + 1);
	const bool position_aging = (elapsed_position > stale_warn_timeout) || (elapsed_velocity > stale_warn_timeout);
	const bool position_stale = (elapsed_position > stale_hold_timeout) || (elapsed_velocity > stale_hold_timeout);
	const bool position_fail = (elapsed_position > stale_fail_timeout) || (elapsed_velocity > stale_fail_timeout);
	const bool attitude_aging = (elapsed_attitude > stale_warn_timeout) || (elapsed_ang_vel > stale_warn_timeout);
	const bool attitude_stale = (elapsed_attitude > stale_hold_timeout) || (elapsed_ang_vel > stale_hold_timeout);
	const bool attitude_fail = (elapsed_attitude > stale_fail_timeout) || (elapsed_ang_vel > stale_fail_timeout);
	const bool aging = position_aging || attitude_aging;

	if (aging) {
		static hrt_abstime last_stale_warn{0};

		// 限频报告状态老化。
		if ((last_stale_warn == 0) || (hrt_elapsed_time(&last_stale_warn) > 1_s)) {
			PX4_WARN("state aging: pos=%llu vel=%llu att=%llu ang=%llu us",
				 (unsigned long long)elapsed_position,
				 (unsigned long long)elapsed_velocity,
				 (unsigned long long)elapsed_attitude,
				 (unsigned long long)elapsed_ang_vel);
			last_stale_warn = hrt_absolute_time();
		}
	}

	// 分开记录位置和姿态等级，支持独立降级。
	_position_state_age_level = position_fail ? 3 : (position_stale ? 2 : (position_aging ? 1 : 0));
	_attitude_state_age_level = attitude_fail ? 3 : (attitude_stale ? 2 : (attitude_aging ? 1 : 0));
	_state_age_level = (_position_state_age_level > _attitude_state_age_level) ? _position_state_age_level :
			   _attitude_state_age_level;

	return true;
}

//STAB（姿态稳定）模式状态更新函数
bool FullvectorControl::updateAttitudeStateOnly()
{
	_state_age_level = 0;
	_position_state_age_level = 0;
	_attitude_state_age_level = 0;

	// 更新姿态和角速度。
	updateAttitudeAndAngularVelocity();

	if ((_last_attitude_update == 0) || (_last_angular_velocity_update == 0)) {
		return false;
	}

	// 复用完整状态检查的老化阈值。
	const hrt_abstime elapsed_attitude = hrt_elapsed_time(&_last_attitude_update);
	const hrt_abstime elapsed_ang_vel = hrt_elapsed_time(&_last_angular_velocity_update);
	constexpr hrt_abstime stale_warn_timeout = 200_ms;
	constexpr hrt_abstime stale_hold_timeout = 500_ms;
	constexpr hrt_abstime stale_fail_timeout = 1500_ms;
	const bool aging = (elapsed_attitude > stale_warn_timeout) || (elapsed_ang_vel > stale_warn_timeout);
	const bool stale_hold = (elapsed_attitude > stale_hold_timeout) || (elapsed_ang_vel > stale_hold_timeout);
	const bool stale_fail = (elapsed_attitude > stale_fail_timeout) || (elapsed_ang_vel > stale_fail_timeout);

	if (aging) {
		static hrt_abstime last_stale_warn{0};

		// 限频报告姿态状态老化。
		if ((last_stale_warn == 0) || (hrt_elapsed_time(&last_stale_warn) > 1_s)) {
			PX4_WARN("attitude state aging: att=%llu ang=%llu us",
				 (unsigned long long)elapsed_attitude,
				 (unsigned long long)elapsed_ang_vel);
			last_stale_warn = hrt_absolute_time();
		}
	}

	_attitude_state_age_level = stale_fail ? 3 : (stale_hold ? 2 : (aging ? 1 : 0));
	_state_age_level = _attitude_state_age_level;

	return true;
}

void FullvectorControl::updateAttitudeAndAngularVelocity()
{
	// 更新姿态消息并将姿态四元数转换为欧拉角。
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_attitude);
		_current_state.attitude = Quatf(_attitude.q);
		_current_state.Euler_angles = Vector3f(Eulerf(_current_state.attitude));
		_last_attitude_update = _attitude.timestamp;
	}

	// 更新角速度消息（机体 FRD 坐标系）。
	if (_vehicle_angular_velocity_sub.updated()) {
		_vehicle_angular_velocity_sub.copy(&_angular_velocity);
		_current_state.angular_velocity = Vector3f(_angular_velocity.xyz[0],
						  _angular_velocity.xyz[1],
						  _angular_velocity.xyz[2]);
		_last_angular_velocity_update = _angular_velocity.timestamp;
	}
}

bool FullvectorControl::shouldLogFaultWarning(hrt_abstime now)
{
	constexpr hrt_abstime warning_interval = 1_s;

	if ((_last_fault_warning_time == 0) || ((now - _last_fault_warning_time) > warning_interval)) {
		_last_fault_warning_time = now;
		return true;
	}

	return false;
}

void FullvectorControl::Run()
{
	// 响应模块退出请求。
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 以 5 ms 周期调度控制循环。
	ScheduleDelayed(5_ms);

	parameters_update(false);
	const hrt_abstime now = hrt_absolute_time();

	// 更新模式和轨迹输入。
	_vehicle_control_mode_sub.update(&_control_mode);
	_vehicle_status_sub.update(&_vehicle_status);
	_trajectory_setpoint_sub.update(&_trajectory_setpoint);

	// 相对位姿有效性检查。
	const hrt_abstime relative_pose_timeout = static_cast<hrt_abstime>(
				math::max(_param_fv_rel_loss_t.get(), 0.1f) * 1_s);

	// 使用候选缓存，避免异常消息覆盖上一份有效控制输入。
	target_relative_pose_s relative_pose_candidate{};

	if (_target_relative_pose_sub.update(&relative_pose_candidate)) {
		_last_target_relative_pose_update = now;
		_target_relative_pose_valid = validateRelativePoseSample(relative_pose_candidate, now);
	}

	_target_pose_receive_age = (_last_target_relative_pose_update > 0) ? now - _last_target_relative_pose_update : 0;

	// 接收端长时间无消息时强制使相对位姿失效；显式无效或异常样本会立即使其失效。
	if ((_last_target_relative_pose_update == 0)
	    || (_target_pose_receive_age > relative_pose_timeout)) {
		_target_relative_pose_valid = false;
		_relative_pose_reject_reason = fullvector_control_status_s::REL_POSE_REJECT_RX_TIMEOUT;
	}

	float rc_switch_value = NAN;
	bool rc_switch_valid = false;
	// 在输出前完成控制权仲裁。
	const bool native_requested = evaluateNativeControllerRequest(rc_switch_value, rc_switch_valid);

	// 汇总参数、解锁状态和飞行模式门控。
	const bool fv_enabled = (_param_fv_enable.get() == 1);
	const bool armed = _control_mode.flag_armed;
	const bool posctl_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL);
	const bool offboard_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	// 固定本周期的相对位姿状态快照。
	const bool relative_pose_was_active = _relative_pose_active;
	_relative_pose_active = offboard_mode && _target_relative_pose_valid && !_relative_pose_hold_timed_out;
	_relative_pose_just_lost = offboard_mode && relative_pose_was_active && !_relative_pose_active;

	// 记录相对位姿会话以触发失联保持。
	if (!offboard_mode) {
		_relative_pose_session_active = false;
		_relative_pose_hold_initialized = false;
		_relative_pose_hold_timed_out = false;
		_relative_pose_loss_started = 0;
		_relative_pose_loss_duration = 0;
		_last_fallback_request = 0;

	} else if (_relative_pose_active) {
		_relative_pose_session_active = true;
		_relative_pose_loss_started = 0;
		_relative_pose_loss_duration = 0;
	}

	if (_relative_pose_just_lost) {
		_relative_pose_hold_initialized = false;
		_relative_pose_loss_started = now;
	}

	// 对接会话失联后累计保持时间，并在超时后锁存中止状态。
	if (offboard_mode && _relative_pose_session_active && !_relative_pose_active) {
		if (_relative_pose_loss_started == 0) {
			_relative_pose_loss_started = now;
		}

		_relative_pose_loss_duration = now - _relative_pose_loss_started;
		const hrt_abstime maximum_hold_time = static_cast<hrt_abstime>(
				math::max(_param_fv_rel_hold_t.get(), 0.5f) * 1_s);

		if (_relative_pose_loss_duration > maximum_hold_time) {
			_relative_pose_hold_timed_out = true;
		}
	}

	_relative_pose_loss_hold = offboard_mode && _relative_pose_session_active && !_relative_pose_active
				   && !_relative_pose_hold_timed_out;

	if (offboard_mode && _relative_pose_hold_timed_out) {
		requestPositionControlFallback(now);
	}
	const bool stabilized_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB);
	const bool termination_mode = _control_mode.flag_control_termination_enabled
				      || (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION);
	const bool fullvector_mode_allowed = posctl_mode || offboard_mode || stabilized_mode;
	const bool module_active = fv_enabled && armed && !termination_mode;
	const bool fullvector_active = module_active && fullvector_mode_allowed && !native_requested;

	// 离开模式时丢弃对应航向目标。
	if (!posctl_mode) {
		_posctl_yaw_sp_initialized = false;
	}

	if (!stabilized_mode) {
		_manual_yaw_sp_initialized = false;
		_manual_yaw_stick_active = false;
	}

	// 未激活时释放控制权并清除历史状态。
	if (!module_active) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		_last_actuator_output_valid = false;
		publishFullvectorControlStatus(false, native_requested, rc_switch_valid, rc_switch_value);
		return;
	}

	// 非支持模式下保持静默。
	if (!fullvector_mode_allowed) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		_last_actuator_output_valid = false;
		publishFullvectorControlStatus(false, native_requested, rc_switch_valid, rc_switch_value);
		return;
	}

	// RC 切换时舵机回中并交还控制权。
	if (native_requested) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		publishNeutralTiltServos();
		_controller_was_active = false;
		_command_initialized = false;
		_last_actuator_output_valid = false;
		publishFullvectorControlStatus(false, true, rc_switch_valid, rc_switch_value);
		return;
	}

	// 声明 fullvector 接管执行器。
	publishFullvectorControlStatus(fullvector_active, false, rc_switch_valid, rc_switch_value);

	// 激活沿从干净状态启动。
	if (!_controller_was_active) {
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;
	}

	_controller_was_active = true;

	// 计算并约束控制周期。
	if (_last_run_time == 0) {
		_dt = 0.005f;

	} else {
		_dt = (now - _last_run_time) / 1e6f;
	}

	_last_run_time = now;

	if (!PX4_ISFINITE(_dt) || (_dt <= FLT_EPSILON)) {
		return;
	}

	constexpr float dt_clamp_s = 0.05f;
	constexpr float dt_reset_s = 0.1f;

	// 长周期重置 PID，中等抖动只限制 dt。
	if (_dt > dt_reset_s) {
		resetPidState();
		_dt = 0.01f;

	} else if (_dt > dt_clamp_s) {
		_dt = dt_clamp_s;
	}

	// 状态数据失效保护。
	const bool state_valid = stabilized_mode ? updateAttitudeStateOnly() : updateUAVState();

	// 缺少最小闭环状态时发布安全输出。
	if (!state_valid) {
		publishSafeActuatorFallback();
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("state unavailable, publishing safe actuator fallback");
		}

		return;
	}

	// 姿态严重过期时进入失效保护。
	if (_attitude_state_age_level >= 3) {
		publishSafeActuatorFallback();
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("attitude stale-fail, publishing safe actuator fallback");
		}

		return;
	}

	// 姿态短时过期时保持上一拍输出。
	if (_attitude_state_age_level >= 2) {
		if (!publishLastActuatorCommand()) {
			publishSafeActuatorFallback();
		}

		resetPidState();
		_last_run_time = 0;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("attitude stale, holding last actuator command");
		}

		return;
	}

	const UAVStates &state_for_control = _current_state;

	// 相对位姿控制和失联保持。
	if (_relative_pose_active) {
		_relative_attitude = Quatf(_target_relative_pose.q).normalized();
		_relative_euler = Vector3f(Eulerf(_relative_attitude));
		// 将目标机 FRD 中的相对位置误差旋转到 NED。
		_rotation_ned_target = Dcmf(state_for_control.attitude) * Dcmf(_relative_attitude).T();
	}

	// 首次接管时对齐当前状态，避免目标阶跃。
	if (!_command_initialized) {
		_current_command.position = _current_state.position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_current_command.Euler_angles = _current_state.Euler_angles;
		_current_command.angular_velocity.zero();
		_command_initialized = true;
	}

	// 相对目标丢失时锁定当前位置和航向。
	if ((_relative_pose_loss_hold || _relative_pose_hold_timed_out) && !_relative_pose_hold_initialized) {
		_relative_pose_hold_position = state_for_control.position;
		_relative_pose_hold_yaw = state_for_control.Euler_angles(2);
		// 保留速度环和角速度环积分，由后续误差源切换只清理发生变化的外环状态。
		_relative_pose_hold_initialized = true;
	}

	// 持续输出相对目标丢失时的锁定目标。
	if (_relative_pose_loss_hold || _relative_pose_hold_timed_out) {
		_current_command.position = _relative_pose_hold_position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_current_command.Euler_angles = Vector3f(0.0f, 0.0f, _relative_pose_hold_yaw);
		_current_command.angular_velocity.zero();
	}

	// STAB、失联保持或超过 500 ms 的轨迹目标不参与控制。
	const bool trajectory_valid = !stabilized_mode && !_relative_pose_loss_hold && !_relative_pose_hold_timed_out
				      && (_trajectory_setpoint.timestamp != 0)
				      && (hrt_elapsed_time(&_trajectory_setpoint.timestamp) < 500_ms);

	const float current_yaw = _current_state.Euler_angles(2);

	// 进入 POSCTL 时捕获当前航向。
	if (posctl_mode && !_posctl_yaw_sp_initialized) {
		_posctl_yaw_sp = current_yaw;
		_posctl_yaw_sp_initialized = true;
	}

	float yaw_sp_target = posctl_mode ? _posctl_yaw_sp : _current_command.Euler_angles(2);
	bool posctl_yaw_rate_active = false;
	_current_command.angular_velocity.zero();

	// 将轨迹目标写入内部命令。
	if (trajectory_valid) {
		// POSCTL 保留 NaN 以按轴解锁位置环。
		for (int i = 0; i < 3; i++) {
			if (posctl_mode) {
				_current_command.position(i) = _trajectory_setpoint.position[i];

			} else if (PX4_ISFINITE(_trajectory_setpoint.position[i])) {
				_current_command.position(i) = _trajectory_setpoint.position[i];
			}

			_current_command.velocity(i) = PX4_ISFINITE(_trajectory_setpoint.velocity[i]) ?
						       _trajectory_setpoint.velocity[i] : 0.0f;

			_current_command.acceleration(i) = PX4_ISFINITE(_trajectory_setpoint.acceleration[i]) ?
							   _trajectory_setpoint.acceleration[i] : 0.0f;
		}

		// POSCTL 在 yaw 角速度与航向保持之间切换。
		if (posctl_mode) {
			const float yaw_rate_sp = PX4_ISFINITE(_trajectory_setpoint.yawspeed) ?
						  _trajectory_setpoint.yawspeed : 0.0f;
			posctl_yaw_rate_active = fabsf(yaw_rate_sp) > FLT_EPSILON;

			// yaw 杆活动时使用角速度控制。
			if (posctl_yaw_rate_active) {
				_posctl_yaw_sp = current_yaw;
				_current_command.angular_velocity(2) = yaw_rate_sp;

			} else if (PX4_ISFINITE(_trajectory_setpoint.yaw)) {
				_posctl_yaw_sp = matrix::wrap_pi(_trajectory_setpoint.yaw);
			}

			yaw_sp_target = _posctl_yaw_sp;

		} else if (PX4_ISFINITE(_trajectory_setpoint.yaw)) {
			yaw_sp_target = matrix::wrap_pi(_trajectory_setpoint.yaw);
		}
	}

	// 平移由电机倾转完成，机体姿态目标保持水平。
	Vector3f attitude_sp_target = _current_command.Euler_angles;
	attitude_sp_target(0) = 0.0f;
	attitude_sp_target(1) = 0.0f;
	attitude_sp_target(2) = yaw_sp_target;

	if (stabilized_mode) {
		// 从摇杆生成 STAB 姿态和加速度目标。
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// STAB 模式下的摇杆输入约束。
		constexpr float max_manual_tilt_rad = 0.52f; // about 30 deg
		constexpr float max_manual_yaw_rate = 2.0f; // rad/s
		constexpr float max_manual_xy_accel = 2.0f; // m/s^2

		// 无效摇杆使用安全默认值。
		const float roll_stick = PX4_ISFINITE(_manual_control_setpoint.roll) ?
					 math::constrain(_manual_control_setpoint.roll, -1.0f, 1.0f) : 0.0f;
		const float pitch_stick = PX4_ISFINITE(_manual_control_setpoint.pitch) ?
					  math::constrain(_manual_control_setpoint.pitch, -1.0f, 1.0f) : 0.0f;
		const float yaw_stick = PX4_ISFINITE(_manual_control_setpoint.yaw) ?
					math::constrain(_manual_control_setpoint.yaw, -1.0f, 1.0f) : 0.0f;
		const float throttle_stick = PX4_ISFINITE(_manual_control_setpoint.throttle) ?
					     math::constrain(_manual_control_setpoint.throttle, -1.0f, 1.0f) : -1.0f;
		const float throttle = (throttle_stick + 1.0f) * 0.5f;
		const float hover_throttle = math::constrain(_param_fv_hover_thr.get(), 0.05f, 0.95f);

		// 低油门直接进入安全输出。
		constexpr float manual_throttle_idle_threshold = 0.02f;

		if (throttle <= manual_throttle_idle_threshold) {
			publishSafeActuatorFallback();
			resetPidState();
			_last_run_time = 0;
			_command_initialized = false;
			return;
		}

		// 按 PX4 符号约定生成 roll/pitch 目标。
		attitude_sp_target(0) = roll_stick * max_manual_tilt_rad;
		attitude_sp_target(1) = -pitch_stick * max_manual_tilt_rad;

		// 积分 yaw 杆并限制航向误差。
		constexpr float manual_yaw_deadband = 0.03f;
		constexpr float max_manual_yaw_error = 0.6f; // rad
		const bool yaw_stick_active = fabsf(yaw_stick) > manual_yaw_deadband;
		const float yaw_rate_cmd = yaw_stick_active ? yaw_stick * max_manual_yaw_rate : 0.0f;

		if (!_manual_yaw_sp_initialized) {
			_manual_yaw_sp = current_yaw;
			_manual_yaw_sp_initialized = true;
		}

		if (yaw_stick_active) {
			_manual_yaw_sp = matrix::wrap_pi(_manual_yaw_sp + yaw_rate_cmd * _dt);

		} else if (_manual_yaw_stick_active) {
			// yaw 杆刚释放时捕获当前航向，并只清理一次旧的角速度控制残留。
			_manual_yaw_sp = current_yaw;
			_att_error_int(2) = 0.0f;
			_att_error_prev(2) = matrix::wrap_pi(_manual_yaw_sp - current_yaw);
			_ang_vel_error_int(2) = 0.0f;
			_ang_vel_error_prev(2) = -_current_state.angular_velocity(2);
		}

		// 松杆后保持释放瞬间的航向，不再让目标持续跟随当前 yaw。
		_manual_yaw_stick_active = yaw_stick_active;

		const float yaw_error_from_current = matrix::wrap_pi(_manual_yaw_sp - current_yaw);

		if (fabsf(yaw_error_from_current) > max_manual_yaw_error) {
			_manual_yaw_sp = matrix::wrap_pi(current_yaw
							 + math::constrain(yaw_error_from_current,
									 -max_manual_yaw_error,
									 max_manual_yaw_error));
		}

		attitude_sp_target(2) = _manual_yaw_sp;
		_current_command.angular_velocity(2) = yaw_rate_cmd;

		// 将机体系摇杆加速度转换到 NED。
		_current_command.position = _current_state.position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_pos_acc_cmd.zero();
		const float forward_accel = pitch_stick * max_manual_xy_accel;
		const float right_accel = roll_stick * max_manual_xy_accel;
		_pos_acc_cmd(0) = cosf(current_yaw) * forward_accel - sinf(current_yaw) * right_accel;
		_pos_acc_cmd(1) = sinf(current_yaw) * forward_accel + cosf(current_yaw) * right_accel;

		// 反算 NED 垂向加速度以匹配归一化油门。
		_pos_acc_cmd(2) = gravity * (1.0f - throttle / hover_throttle);
	}

	// 限制姿态目标变化率。
	constexpr float attitude_sp_slew_rate = 1.0f; // rad/s
	const float attitude_sp_step = attitude_sp_slew_rate * _dt;

	// STAB yaw 和 POSCTL yaw rate 绕过姿态限速。
	for (int i = 0; i < 3; i++) {
		if (stabilized_mode && (i == 2)) {
			_current_command.Euler_angles(i) = attitude_sp_target(i);
			continue;
		}

		if (posctl_mode && (i == 2) && posctl_yaw_rate_active) {
			_current_command.Euler_angles(i) = current_yaw;
			continue;
		}

		const float error = matrix::wrap_pi(attitude_sp_target(i) - _current_command.Euler_angles(i));
		_current_command.Euler_angles(i) = matrix::wrap_pi(_current_command.Euler_angles(i)
						   + math::constrain(error, -attitude_sp_step, attitude_sp_step));
	}

	perf_begin(_loop_perf);

	// 位置失效时仅保留姿态闭环。
	const bool run_position_control = !stabilized_mode && (_position_state_age_level < 2);

	if (run_position_control) {
		PositionControl(state_for_control, _current_command, _dt);

	} else if (!stabilized_mode) {
		resetPositionPidState();
		_pos_acc_cmd.zero();

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("position stale, skipping position control");
		}
	}

	AttitudeControl(state_for_control, _current_command, _dt, stabilized_mode);
	controlAllocation(state_for_control, _current_command);

	perf_end(_loop_perf);
}

void FullvectorControl::PositionControl(const UAVStates &state, const UAVCommand &command, const float dt)
{
	// 检查时间步长有效性。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	//确定当前位置控制模式
	const bool use_relative_pose = _relative_pose_active;
	const bool docking_position_guard = use_relative_pose || _relative_pose_loss_hold || _relative_pose_hold_timed_out;
	const bool posctl_mode = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL;
	// 每周期重新统计位置通道是否触发积分反算。
	_position_anti_windup_active = false;

	//初始化局部变量
	Vector3f ep{};
	bool position_axis_locked[3] {};
	bool initialize_velocity_error = false;

	// 选择相对或 NED 位置误差。
	if (use_relative_pose) {
		const Vector3f relative_position(_target_relative_pose.position);
		const Vector3f relative_position_sp(_param_fv_rel_pos_x.get(),
						    _param_fv_rel_pos_y.get(),
						    _param_fv_rel_pos_z.get());
		const Vector3f relative_error_target = relative_position_sp - relative_position;

		ep = _rotation_ned_target * relative_error_target;

		for (bool &axis_locked : position_axis_locked) {
			axis_locked = true;
		}

	} else {
		// 有限位置目标启用位置环，NaN 目标保留速度控制。
		for (int i = 0; i < 3; i++) {
			position_axis_locked[i] = PX4_ISFINITE(command.position(i));

			if (position_axis_locked[i]) {
				ep(i) = command.position(i) - state.position(i);
			}
		}
	}

	// POSCTL 锁高时使用融合的 Z 速度反馈。
	const bool posctl_z_hold_active = posctl_mode && !use_relative_pose && position_axis_locked[2];
	const bool posctl_z_just_locked = posctl_z_hold_active && !_position_axis_locked[2];
	_posctl_z_hold_active = posctl_z_hold_active;
	_vertical_velocity_feedback = state.velocity(2);

	// POSCTL 锁高时融合同源 Z 位置导数。
	if (posctl_z_hold_active && PX4_ISFINITE(_position.z_deriv)) {
		const float z_derivative_weight = math::constrain(_param_fv_z_vel_blend.get(), 0.0f, 1.0f);
		_vertical_velocity_feedback = (1.0f - z_derivative_weight) * state.velocity(2)
					      + z_derivative_weight * _position.z_deriv;
	}

	// 切换误差源时只清除位置外环积分并同步微分历史，保留速度内环的悬停补偿。
	if (use_relative_pose != _position_outer_uses_relative_pose) {
		_pos_error_int.zero();
		_pos_error_prev = ep;
		initialize_velocity_error = true;
		_position_outer_uses_relative_pose = use_relative_pose;

		for (int i = 0; i < 3; i++) {
			_position_axis_locked[i] = position_axis_locked[i];
		}
	}

	// 首次运行时抑制微分冲击。
	if (!_pid_state_initialized) {
		_pos_error_prev = ep;
		initialize_velocity_error = true;
		_pid_state_initialized = true;

		for (int i = 0; i < 3; i++) {
			_position_axis_locked[i] = position_axis_locked[i];
		}
	}

	// 逐轴处理位置锁定切换。
	for (int i = 0; i < 3; i++) {
		if (!position_axis_locked[i]) {
			_pos_error_int(i) = 0.0f;
			_pos_error_prev(i) = 0.0f;

		} else if (!_position_axis_locked[i]) {
			_pos_error_int(i) = 0.0f;
			_pos_error_prev(i) = ep(i);
		}
	}

	// 计算位置误差的微分和积分。
	const Vector3f dep = (ep - _pos_error_prev) / dt;
	_pos_error_int += ep * dt;

	// 限制积分状态，抑制长时间位置偏差积累。
	for (int i = 0; i < 3; i++) {
		_pos_error_int(i) = math::constrain(_pos_error_int(i), -5.0f, 5.0f);
	}

	// 位置 PID 增益。
	const Vector3f pos_kp{gain_pos_pid(0, 0), gain_pos_pid(1, 0), gain_pos_pid(2, 0)};
	const Vector3f pos_ki{gain_pos_pid(0, 1), gain_pos_pid(1, 1), gain_pos_pid(2, 1)};
	const Vector3f pos_kd{gain_pos_pid(0, 2), gain_pos_pid(1, 2), gain_pos_pid(2, 2)};

	// 生成期望速度，相对控制禁用轨迹速度前馈。
	const Vector3f velocity_ff = use_relative_pose ? Vector3f{} : command.velocity;
	Vector3f v_sp = velocity_ff + pos_kp.emult(ep) + pos_ki.emult(_pos_error_int) + pos_kd.emult(dep);
	// 保留限幅前目标，用于计算位置外环反算残差。
	const Vector3f v_sp_unconstrained = v_sp;

	// 常规模式逐轴限制期望速度。
	for (int i = 0; i < 3; i++) {
		v_sp(i) = math::constrain(v_sp(i), -5.0f, 5.0f);
	}

	// 对接阶段限制水平速度模长。
	if (docking_position_guard) {
		const float horizontal_velocity_limit = math::max(_param_fv_rel_vxy_max.get(), FLT_EPSILON);
		const float horizontal_velocity_norm = v_sp.xy().norm();

		if (horizontal_velocity_norm > horizontal_velocity_limit) {
			const float scale = horizontal_velocity_limit / horizontal_velocity_norm;
			v_sp(0) *= scale;
			v_sp(1) *= scale;
		}
	}

	// 根据速度限幅残差修正位置外环积分。
	bool velocity_setpoint_saturated[3] {};
	const Vector3f velocity_setpoint_residual = v_sp - v_sp_unconstrained;

	for (int i = 0; i < 3; i++) {
		velocity_setpoint_saturated[i] = fabsf(velocity_setpoint_residual(i)) > FLT_EPSILON;
	}

	_position_anti_windup_active |= applyAntiWindup(_pos_error_int, velocity_setpoint_residual,
					       pos_ki, velocity_setpoint_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_pos_error_int(i) = math::constrain(_pos_error_int(i), -5.0f, 5.0f);
	}

	// 速度 PID 与加速度前馈共同生成期望加速度。
	Vector3f velocity_feedback = state.velocity;
	// Z 轴使用 POSCTL 选择后的融合反馈。
	velocity_feedback(2) = _vertical_velocity_feedback;
	const Vector3f ev = v_sp - velocity_feedback;

	// 首次运行时抑制微分冲击。
	if (initialize_velocity_error) {
		_vel_error_prev = ev;
	}

	// 锁高切入时同步 Z 轴微分历史。
	if (posctl_z_just_locked) {
		_vel_error_prev(2) = ev(2);
	}

	const Vector3f dev = (ev - _vel_error_prev) / dt;

	// 锁高快速制动时冻结 Z 轴积分。
	_vel_error_int(0) += ev(0) * dt;
	_vel_error_int(1) += ev(1) * dt;
	constexpr float vertical_braking_velocity = 0.15f;
	const bool allow_vertical_integrator = !posctl_mode || !posctl_z_hold_active
					       || (fabsf(_vertical_velocity_feedback) < vertical_braking_velocity);

	if (allow_vertical_integrator) {
		_vel_error_int(2) += ev(2) * dt;
	}

	// 统一限制速度积分状态在有限范围。
	for (int i = 0; i < 3; i++) {
		_vel_error_int(i) = math::constrain(_vel_error_int(i), -3.0f, 3.0f);
	}

	// 速度 PID 增益。
	const Vector3f vel_kp{gain_vel_pid(0, 0), gain_vel_pid(1, 0), gain_vel_pid(2, 0)};
	Vector3f vel_ki{gain_vel_pid(0, 1), gain_vel_pid(1, 1), gain_vel_pid(2, 1)};
	const Vector3f vel_kd{gain_vel_pid(0, 2), gain_vel_pid(1, 2), gain_vel_pid(2, 2)};
	// 保存 POSCTL 垂向积分状态上限，反算后继续执行同一限制。
	float posctl_vertical_integral_state_limit = 3.0f;

	// POSCTL模式单独限制 Z 轴积分带宽和贡献。
	if (posctl_mode) {
		// 地面站可在 0～1 之间完整调整定点垂向积分带宽。
		vel_ki(2) *= math::constrain(_param_fv_pc_z_i_scale.get(), 0.0f, 1.0f);

		// 垂向积分只用于补偿慢变悬停偏差，限制其最大加速度贡献，快速高度误差由 P 环处理。
		constexpr float posctl_vertical_integral_acceleration_max = 0.6f;
		const float z_integral_acceleration_limit = math::constrain(_param_fv_z_int_max.get(), 0.1f,
							      posctl_vertical_integral_acceleration_max);

		if (fabsf(vel_ki(2)) > FLT_EPSILON) {
			posctl_vertical_integral_state_limit = z_integral_acceleration_limit / fabsf(vel_ki(2));

			if (posctl_z_just_locked) {
				// 松杆重新锁高时只保留有限悬停补偿，避免把手动升降阶段的积分饱和带入制动。
				constexpr float lock_capture_integral_ratio = 0.5f;
				const float lock_capture_state_limit = lock_capture_integral_ratio
								 * posctl_vertical_integral_state_limit;
				_vel_error_int(2) = math::constrain(_vel_error_int(2),
							    -lock_capture_state_limit, lock_capture_state_limit);
			}

			_vel_error_int(2) = math::constrain(_vel_error_int(2),
							    -posctl_vertical_integral_state_limit,
							    posctl_vertical_integral_state_limit);

		} else {
			_vel_error_int(2) = 0.0f;
		}
	}

	// 使用上一周期的分配残差修正速度环积分。
	_position_anti_windup_active |= applyAntiWindup(_vel_error_int, _allocation_accel_residual,
					       vel_ki, _position_allocation_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_vel_error_int(i) = math::constrain(_vel_error_int(i), -3.0f, 3.0f);
	}

	if (posctl_mode) {
		_vel_error_int(2) = math::constrain(_vel_error_int(2),
						-posctl_vertical_integral_state_limit,
						posctl_vertical_integral_state_limit);
	}

	// 选择加速度前馈，相对位姿控制禁用轨迹加速度前馈。
	const Vector3f acceleration_ff = use_relative_pose ? Vector3f{} : command.acceleration;
	Vector3f acc_cmd = acceleration_ff + vel_kp.emult(ev) + vel_ki.emult(_vel_error_int) + vel_kd.emult(dev);
	// 保留限幅前目标，用于计算速度内环反算残差。
	const Vector3f acc_cmd_unconstrained = acc_cmd;

	// 对接阶段限制水平加速度模长。
	if (docking_position_guard) {
		const float horizontal_acceleration_limit = math::max(_param_fv_rel_axy_max.get(), FLT_EPSILON);
		const float horizontal_acceleration_norm = acc_cmd.xy().norm();

		if (horizontal_acceleration_norm > horizontal_acceleration_limit) {
			const float scale = horizontal_acceleration_limit / horizontal_acceleration_norm;
			acc_cmd(0) *= scale;
			acc_cmd(1) *= scale;
		}
	}

	// 根据加速度限幅残差修正速度内环积分。
	bool acceleration_setpoint_saturated[3] {};
	const Vector3f acceleration_setpoint_residual = acc_cmd - acc_cmd_unconstrained;

	for (int i = 0; i < 3; i++) {
		acceleration_setpoint_saturated[i] = fabsf(acceleration_setpoint_residual(i)) > FLT_EPSILON;
	}

	_position_anti_windup_active |= applyAntiWindup(_vel_error_int, acceleration_setpoint_residual,
					       vel_ki, acceleration_setpoint_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_vel_error_int(i) = math::constrain(_vel_error_int(i), -3.0f, 3.0f);
	}

	if (posctl_mode) {
		_vel_error_int(2) = math::constrain(_vel_error_int(2),
						-posctl_vertical_integral_state_limit,
						posctl_vertical_integral_state_limit);
	}

	_vertical_velocity_integral_acceleration = vel_ki(2) * _vel_error_int(2);

	// 保存位置环输出和 PID 历史。
	_pos_acc_cmd = acc_cmd;

	_pos_error_prev = ep;
	_vel_error_prev = ev;

	// 保存各轴位置锁定状态。
	for (int i = 0; i < 3; i++) {
		_position_axis_locked[i] = position_axis_locked[i];
	}

	// 发布位置环目标和模式切换反馈。
	vehicle_local_position_setpoint_s position_controller_output{};
	position_controller_output.timestamp = hrt_absolute_time();
	position_controller_output.x = command.position(0);
	position_controller_output.y = command.position(1);
	position_controller_output.z = command.position(2);
	position_controller_output.vx = v_sp(0);
	position_controller_output.vy = v_sp(1);
	position_controller_output.vz = v_sp(2);
	position_controller_output.acceleration[0] = acc_cmd(0);
	position_controller_output.acceleration[1] = acc_cmd(1);
	position_controller_output.acceleration[2] = acc_cmd(2);
	position_controller_output.thrust[0] = NAN;
	position_controller_output.thrust[1] = NAN;
	position_controller_output.thrust[2] = NAN;
	position_controller_output.yaw = command.Euler_angles(2);
	position_controller_output.yawspeed = command.angular_velocity(2);
	_position_controller_output_pub.publish(position_controller_output);
}

void FullvectorControl::AttitudeControl(const UAVStates &state, UAVCommand &command, const float dt,
					bool stabilized_mode)
{
	// 检查时间步长有效性。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	// 确定当前姿态控制模式。
	const bool use_relative_pose = _relative_pose_active;
	const bool docking_attitude_guard = use_relative_pose || _relative_pose_loss_hold || _relative_pose_hold_timed_out;
	const bool full_relative_attitude = _param_fv_rel_att_mode.get() == 1;
	// 每周期重新统计姿态通道是否触发积分反算。
	_attitude_anti_windup_active = false;
	// 初始化局部变量。
	Vector3f euler_cur;
	Vector3f euler_sp;
	bool initialize_angular_velocity_error = false;

	// 选择绝对或相对姿态误差源。
	if (use_relative_pose) {
		if (full_relative_attitude) {
			// 旧模式跟踪三轴视觉相对姿态。
			euler_cur = _relative_euler;
			euler_sp = Vector3f(_param_fv_rel_roll.get(),
					    _param_fv_rel_pitch.get(),
					    _param_fv_rel_yaw.get());

		} else {
			// 默认由 IMU 保持 roll/pitch，仅由视觉对齐 yaw。
			euler_cur = state.Euler_angles;
			euler_sp = command.Euler_angles;
			euler_cur(2) = _relative_euler(2);
			euler_sp(2) = _param_fv_rel_yaw.get();
		}

	} else {
		euler_cur = state.Euler_angles;
		euler_sp = command.Euler_angles;
	}

	// 定点模式判定。
	const Vector3f angular_velocity_ff = command.angular_velocity;
	const bool posctl_mode = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL;
	const bool posctl_yaw_rate_active = posctl_mode && (fabsf(angular_velocity_ff(2)) > FLT_EPSILON);
	const bool stabilized_yaw_rate_active = stabilized_mode && (fabsf(angular_velocity_ff(2)) > FLT_EPSILON);
	const bool yaw_rate_control_active = posctl_yaw_rate_active || stabilized_yaw_rate_active;
	const bool manual_yaw_control = stabilized_mode || posctl_mode;

	// 计算姿态角误差，将姿态误差约束到最短角路径。
	Vector3f e_att = euler_sp - euler_cur;
	e_att(0) = matrix::wrap_pi(e_att(0));
	e_att(1) = matrix::wrap_pi(e_att(1));
	e_att(2) = matrix::wrap_pi(e_att(2));

	// 切换误差源时只清除发生变化的姿态外环积分，保留角速度内环的稳态力矩补偿。
	if (use_relative_pose != _attitude_outer_uses_relative_pose) {
		if (full_relative_attitude) {
			_att_error_int.zero();

		} else {
			// 默认相对姿态模式只切换 yaw 误差源，roll/pitch 仍由 IMU 保持。
			_att_error_int(2) = 0.0f;
		}

		_att_error_prev = e_att;
		initialize_angular_velocity_error = true;
		_attitude_outer_uses_relative_pose = use_relative_pose;
	}

	// 首次运行时抑制微分冲击。
	if (!_att_pid_state_initialized) {
		_att_error_prev = e_att;
		initialize_angular_velocity_error = true;
		_att_pid_state_initialized = true;
	}

	// 计算姿态误差微分。
	Vector3f de_att = (e_att - _att_error_prev) / dt;

	// 手动 yaw 角速度切入时抑制姿态微分脉冲。
	if (yaw_rate_control_active) {
		de_att(2) = 0.0f;
	}

	// yaw 杆活动时禁用姿态积分，松杆航向保持期间允许积分消除静态误差。
	if (yaw_rate_control_active) {
		_att_error_int(2) = 0.0f;
	}

	// 计算姿态 PID 积分。
	_att_error_int += e_att * dt;

	// yaw 杆活动时禁用姿态积分。
	if (yaw_rate_control_active) {
		_att_error_int(2) = 0.0f;
	}

	// 姿态积分逐轴限幅，避免持续角度误差饱和。
	for (int i = 0; i < 3; i++) {
		_att_error_int(i) = math::constrain(_att_error_int(i), -1.0f, 1.0f);
	}

	// 姿态 PID 增益。
	const Vector3f att_kp{gain_att_pid(0, 0), gain_att_pid(1, 0), gain_att_pid(2, 0)};
	const Vector3f att_ki{gain_att_pid(0, 1), gain_att_pid(1, 1), gain_att_pid(2, 1)};
	const Vector3f att_kd{gain_att_pid(0, 2), gain_att_pid(1, 2), gain_att_pid(2, 2)};

	// 姿态 PID 生成期望角速度。
	Vector3f omega_sp = att_kp.emult(e_att) + att_ki.emult(_att_error_int) + att_kd.emult(de_att);

	// 视觉全姿态模式缩放三轴，默认模式只缩放 yaw。
	if (use_relative_pose) {
		const float relative_attitude_gain = math::constrain(_param_fv_rel_att_gain.get(), 0.05f, 1.0f);

		if (full_relative_attitude) {
			omega_sp *= relative_attitude_gain;

		} else {
			omega_sp(2) *= relative_attitude_gain;
		}
	}

	// 叠加手动 yaw 角速度前馈。
	if (manual_yaw_control) {
		omega_sp(2) += angular_velocity_ff(2);
	}

	// 保留限幅前目标，用于计算姿态外环反算残差。
	const Vector3f omega_sp_unconstrained = omega_sp;

	// 对接阶段限制期望角速度。
	if (docking_attitude_guard) {
		const float rate_limit = math::max(_param_fv_rel_rate_max.get(), 0.1f);

		for (int i = 0; i < 3; i++) {
			omega_sp(i) = math::constrain(omega_sp(i), -rate_limit, rate_limit);
		}
	}

	if (manual_yaw_control) {
		constexpr float max_manual_yaw_rate = 2.0f; // rad/s
		omega_sp(2) = math::constrain(omega_sp(2), -max_manual_yaw_rate, max_manual_yaw_rate);
	}

	// 根据角速度限幅残差修正姿态外环积分。
	bool angular_velocity_setpoint_saturated[3] {};
	const Vector3f angular_velocity_setpoint_residual = omega_sp - omega_sp_unconstrained;

	for (int i = 0; i < 3; i++) {
		angular_velocity_setpoint_saturated[i] = fabsf(angular_velocity_setpoint_residual(i)) > FLT_EPSILON;
	}

	if (yaw_rate_control_active) {
		angular_velocity_setpoint_saturated[2] = false;
	}

	_attitude_anti_windup_active |= applyAntiWindup(_att_error_int, angular_velocity_setpoint_residual,
					       att_ki, angular_velocity_setpoint_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_att_error_int(i) = math::constrain(_att_error_int(i), -1.0f, 1.0f);
	}

	if (yaw_rate_control_active) {
		_att_error_int(2) = 0.0f;
	}

	command.angular_velocity = omega_sp;

	// 计算角速度误差。
	const Vector3f e_w = omega_sp - state.angular_velocity;

	// 误差源切换后同步角速度微分历史。
	if (initialize_angular_velocity_error) {
		_ang_vel_error_prev = e_w;
	}

	// 计算角速度误差微分和积分。
	const Vector3f de_w = (e_w - _ang_vel_error_prev) / dt;
	_ang_vel_error_int += e_w * dt;

	// 角速度积分逐轴限幅。
	for (int i = 0; i < 3; i++) {
		_ang_vel_error_int(i) = math::constrain(_ang_vel_error_int(i), -3.0f, 3.0f);
	}

	// 定义手动 Yaw 控制保护常量。
	constexpr float yaw_rate_integral_limit = 0.3f;
	constexpr float max_manual_yaw_accel = 4.0f; // rad/s^2

	// 限制手动 yaw 的积分修正量。
	if (manual_yaw_control) {
		_ang_vel_error_int(2) = math::constrain(_ang_vel_error_int(2),
							-yaw_rate_integral_limit,
							yaw_rate_integral_limit);
	}

	// 角速度 PID 增益。
	const Vector3f w_kp{gain_ang_vel_pid(0, 0), gain_ang_vel_pid(1, 0), gain_ang_vel_pid(2, 0)};
	const Vector3f w_ki{gain_ang_vel_pid(0, 1), gain_ang_vel_pid(1, 1), gain_ang_vel_pid(2, 1)};
	const Vector3f w_kd{gain_ang_vel_pid(0, 2), gain_ang_vel_pid(1, 2), gain_ang_vel_pid(2, 2)};

	// 使用上一周期的分配残差修正角速度环积分。
	_attitude_anti_windup_active |= applyAntiWindup(_ang_vel_error_int, _allocation_ang_acc_residual,
					       w_ki, _attitude_allocation_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_ang_vel_error_int(i) = math::constrain(_ang_vel_error_int(i), -3.0f, 3.0f);
	}

	if (manual_yaw_control) {
		_ang_vel_error_int(2) = math::constrain(_ang_vel_error_int(2),
						-yaw_rate_integral_limit, yaw_rate_integral_limit);
	}

	// 角速度 PID 生成期望角加速度。
	Vector3f ang_acc_cmd = w_kp.emult(e_w) + w_ki.emult(_ang_vel_error_int) + w_kd.emult(de_w);
	// 保留限幅前目标，用于计算角速度内环反算残差。
	const Vector3f ang_acc_cmd_unconstrained = ang_acc_cmd;

	// 手动 yaw 仅限制积分和瞬时角加速度，不在小误差下周期性清积分。
	if (manual_yaw_control) {
		// 限制手动 yaw 对电机差动的瞬时需求。
		ang_acc_cmd(2) = math::constrain(ang_acc_cmd(2), -max_manual_yaw_accel, max_manual_yaw_accel);
	}

	// 对接阶段限制期望角加速度。
	if (docking_attitude_guard) {
		const float acceleration_limit = math::max(_param_fv_rel_acc_max.get(), 0.5f);

		for (int i = 0; i < 3; i++) {
			ang_acc_cmd(i) = math::constrain(ang_acc_cmd(i), -acceleration_limit, acceleration_limit);
		}
	}

	// 根据角加速度限幅残差修正角速度环积分。
	bool angular_acceleration_setpoint_saturated[3] {};
	const Vector3f angular_acceleration_setpoint_residual = ang_acc_cmd - ang_acc_cmd_unconstrained;

	for (int i = 0; i < 3; i++) {
		angular_acceleration_setpoint_saturated[i] = fabsf(angular_acceleration_setpoint_residual(i)) > FLT_EPSILON;
	}

	_attitude_anti_windup_active |= applyAntiWindup(_ang_vel_error_int, angular_acceleration_setpoint_residual,
					       w_ki, angular_acceleration_setpoint_saturated, dt);

	for (int i = 0; i < 3; i++) {
		_ang_vel_error_int(i) = math::constrain(_ang_vel_error_int(i), -3.0f, 3.0f);
	}

	if (manual_yaw_control) {
		_ang_vel_error_int(2) = math::constrain(_ang_vel_error_int(2),
						-yaw_rate_integral_limit, yaw_rate_integral_limit);
	}

	// 保存姿态环输出和 PID 历史。
	_att_ang_acc_cmd = ang_acc_cmd;
	_att_error_prev = e_att;
	_ang_vel_error_prev = e_w;

	// 发布姿态环目标和模式切换反馈。
	vehicle_angular_acceleration_setpoint_s attitude_controller_output{};
	attitude_controller_output.timestamp_sample = hrt_absolute_time();
	attitude_controller_output.timestamp = hrt_absolute_time();
	attitude_controller_output.xyz[0] = ang_acc_cmd(0);
	attitude_controller_output.xyz[1] = ang_acc_cmd(1);
	attitude_controller_output.xyz[2] = ang_acc_cmd(2);
	_attitude_controller_output_pub.publish(attitude_controller_output);
}

void FullvectorControl::calculateMotorCommand(const UAVStates &state, const UAVCommand &command)
{
	// 每周期重新统计分配饱和位和差动缩放比例。
	_allocation_saturation_flags = fullvector_control_status_s::ALLOC_SAT_NONE;
	_allocation_differential_scale = 1.0f;

	// 读取期望 Roll/Pitch
	const float phi_sp = command.Euler_angles(0);
	const float theta_sp = command.Euler_angles(1);

	// 读取位置环和姿态环输出
	const Vector3f &acc_sp_ned = _pos_acc_cmd;
	const Vector3f &ang_acc_sp = _att_ang_acc_cmd;

	// 读取倾转舵机的最大倾转角
	const float tilt_angle_max_rad = math::max(_param_fv_tilt_max.get(), 0.01f);
	// 读取 yaw 补偿最大倾转角和 yaw 电机混合权重
	const float yaw_tilt_max_rad = math::constrain(_param_fv_yaw_tilt_max.get(), 0.0f, tilt_angle_max_rad);
	const float yaw_motor_mix_weight = math::constrain(_param_fv_yaw_mix_wt.get(), 0.0f, 1.0f);

	// 读取当前 Yaw 并将 NED 水平加速度转换到机体 FRD。
	const float current_yaw = state.Euler_angles(2);
	const float cos_yaw = cosf(current_yaw);
	const float sin_yaw = sinf(current_yaw);
	const float acc_sp_body_x = cos_yaw * acc_sp_ned(0) + sin_yaw * acc_sp_ned(1);
	const float acc_sp_body_y = -sin_yaw * acc_sp_ned(0) + cos_yaw * acc_sp_ned(1);

	// 保护物理参数并计算悬停角速度平方。
	const float kf_safe = math::max(K_F, 1e-6f);
	const float mass_safe = math::max(mass, 1e-3f);
	const float gravity_safe = math::max(gravity, 1e-3f);
	const float distance_safe = math::max(distance, 1e-3f);
	const float arm_d = distance_safe / sqrtf(2.0f);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float hover_omega_sq = (mass_safe * gravity_safe) / (4.0f * kf_safe);
	const float hover_throttle = math::constrain(_param_fv_hover_thr.get(), 0.05f, 0.95f);
	const float motor_omega_sq_max = hover_omega_sq / hover_throttle;
	// 计算力到倾角的转换系数。
	const float acc_to_tilt = sqrtf(2.0f) / (mass_safe * gravity_safe);

	// 按 X 型布局由姿态和水平加速度生成四路基础倾转角。
	const float alpha_base1 =  sqrtf(2.0f) * theta_sp + sqrtf(2.0f) * phi_sp
				   - acc_to_tilt * mass_safe * (acc_sp_body_x - acc_sp_body_y) / 4.0f;
	const float alpha_base2 = -sqrtf(2.0f) * theta_sp - sqrtf(2.0f) * phi_sp
				  + acc_to_tilt * mass_safe * (acc_sp_body_x - acc_sp_body_y) / 4.0f;
	const float alpha_base3 = -sqrtf(2.0f) * theta_sp + sqrtf(2.0f) * phi_sp
				  + acc_to_tilt * mass_safe * (acc_sp_body_x + acc_sp_body_y) / 4.0f;
	const float alpha_base4 =  sqrtf(2.0f) * theta_sp - sqrtf(2.0f) * phi_sp
				   - acc_to_tilt * mass_safe * (acc_sp_body_x + acc_sp_body_y) / 4.0f;
	// 在角速度平方域分配推力和力矩，避免角速度增量再次平方产生交叉项和零点抖动。
	float omega_sq_roll = (I_xx * ang_acc_sp(0)) / (4.0f * kf_safe * arm_d);
	float omega_sq_pitch = (I_yy * ang_acc_sp(1)) / (4.0f * kf_safe * arm_d);
	const float tau_z_sp = I_zz * ang_acc_sp(2);
	// K_M 无效时关闭电机 yaw 分量，并把 yaw 力矩全部交给公共倾转通道。
	const bool motor_yaw_available = PX4_ISFINITE(K_M) && (K_M > 1e-9f);
	const float yaw_motor_torque_weight = motor_yaw_available ? yaw_motor_mix_weight : 0.0f;
	const float km_safe = motor_yaw_available ? K_M : 1.0f;
	float omega_sq_yaw = yaw_motor_torque_weight * tau_z_sp / (4.0f * km_safe);
	const float collective_omega_sq_unconstrained =
		mass_safe * (gravity_safe - acc_sp_ned(2)) / (4.0f * kf_safe);
	const float collective_omega_sq = math::constrain(collective_omega_sq_unconstrained, 0.0f,
					  motor_omega_sq_max);

	// 记录集体推力触及下限或上限。
	if (collective_omega_sq_unconstrained < 0.0f) {
		_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_COLLECTIVE_LOW;

	} else if (collective_omega_sq_unconstrained > motor_omega_sq_max) {
		_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_COLLECTIVE_HIGH;
	}

	// 对接阶段在角速度平方域限制三轴差动，保留力矩方向并避免总推力被过度侵占。
	if (_relative_pose_active || _relative_pose_loss_hold || _relative_pose_hold_timed_out) {
		const float differential_ratio = math::constrain(_param_fv_rel_motor_diff.get(), 0.05f, 1.0f);
		const float differential_limit = differential_ratio * math::max(collective_omega_sq, 1.0f);
		const float differential_sum = fabsf(omega_sq_roll) + fabsf(omega_sq_pitch) + fabsf(omega_sq_yaw);

		if (differential_sum > differential_limit) {
			const float differential_scale = differential_limit / differential_sum;
			omega_sq_roll *= differential_scale;
			omega_sq_pitch *= differential_scale;
			omega_sq_yaw *= differential_scale;
			_allocation_differential_scale *= differential_scale;
			_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_DIFFERENTIAL;
		}
	}

	// 生成四路角速度平方差动，编号顺序保持右前、左后、左前、右后。
	float differential_1 = -omega_sq_roll + omega_sq_pitch + omega_sq_yaw;
	float differential_2 =  omega_sq_roll - omega_sq_pitch + omega_sq_yaw;
	float differential_3 =  omega_sq_roll + omega_sq_pitch - omega_sq_yaw;
	float differential_4 = -omega_sq_roll - omega_sq_pitch - omega_sq_yaw;

	// 在不改变集体推力的前提下等比例压缩差动，避免任一路超出归一化电机范围。
	const float differential_min = math::min(math::min(differential_1, differential_2),
						 math::min(differential_3, differential_4));
	const float differential_max = math::max(math::max(differential_1, differential_2),
						 math::max(differential_3, differential_4));
	float differential_scale = 1.0f;

	if (differential_min < 0.0f) {
		differential_scale = math::min(differential_scale, collective_omega_sq / -differential_min);
	}

	if (differential_max > 0.0f) {
		differential_scale = math::min(differential_scale,
					       (motor_omega_sq_max - collective_omega_sq) / differential_max);
	}

	differential_scale = math::constrain(differential_scale, 0.0f, 1.0f);

	// 记录电机范围导致的二次差动缩放。
	if (differential_scale < 1.0f - FLT_EPSILON) {
		_allocation_differential_scale *= differential_scale;
		_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_DIFFERENTIAL;
	}

	differential_1 *= differential_scale;
	differential_2 *= differential_scale;
	differential_3 *= differential_scale;
	differential_4 *= differential_scale;

	const float motor_1_sq = math::constrain(collective_omega_sq + differential_1, 0.0f, motor_omega_sq_max);
	const float motor_2_sq = math::constrain(collective_omega_sq + differential_2, 0.0f, motor_omega_sq_max);
	const float motor_3_sq = math::constrain(collective_omega_sq + differential_3, 0.0f, motor_omega_sq_max);
	const float motor_4_sq = math::constrain(collective_omega_sq + differential_4, 0.0f, motor_omega_sq_max);

	// 保留角速度形式供后续动力学估算使用。
	motor_1 = sqrtf(motor_1_sq);
	motor_2 = sqrtf(motor_2_sq);
	motor_3 = sqrtf(motor_3_sq);
	motor_4 = sqrtf(motor_4_sq);

	// 四个舵机叠加同向公共倾转，以水平分力补偿电机实际未分配的 yaw 力矩。
	const float motor_sq_sum = math::max(motor_1_sq + motor_2_sq + motor_3_sq + motor_4_sq, 1.0f);
	const float tau_z_motor_actual = motor_yaw_available ? 4.0f * K_M * omega_sq_yaw * differential_scale : 0.0f;
	const float tau_z_tilt_sp = tau_z_sp - tau_z_motor_actual;
	// 正公共倾角在当前几何模型中产生负 yaw 力矩，因此公共倾角使用反号。
	const float alpha_yaw_unconstrained = -tau_z_tilt_sp / (kf_safe * distance_safe * motor_sq_sum);
	const float alpha_yaw = math::constrain(alpha_yaw_unconstrained, -yaw_tilt_max_rad, yaw_tilt_max_rad);

	// 记录 yaw 公共倾转触及独立上限。
	if (fabsf(alpha_yaw - alpha_yaw_unconstrained) > FLT_EPSILON) {
		_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_YAW_TILT;
	}

	// 叠加公共倾转角并统一限幅。
	const float alpha_unconstrained1 = alpha_base1 + alpha_yaw;
	const float alpha_unconstrained2 = alpha_base2 + alpha_yaw;
	const float alpha_unconstrained3 = alpha_base3 + alpha_yaw;
	const float alpha_unconstrained4 = alpha_base4 + alpha_yaw;
	alpha_offset1 = math::constrain(alpha_unconstrained1, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset2 = math::constrain(alpha_unconstrained2, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset3 = math::constrain(alpha_unconstrained3, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset4 = math::constrain(alpha_unconstrained4, -tilt_angle_max_rad, tilt_angle_max_rad);

	// 任一舵机触及机械角度上限时记录倾转饱和。
	if ((fabsf(alpha_offset1 - alpha_unconstrained1) > FLT_EPSILON)
	    || (fabsf(alpha_offset2 - alpha_unconstrained2) > FLT_EPSILON)
	    || (fabsf(alpha_offset3 - alpha_unconstrained3) > FLT_EPSILON)
	    || (fabsf(alpha_offset4 - alpha_unconstrained4) > FLT_EPSILON)) {
		_allocation_saturation_flags |= fullvector_control_status_s::ALLOC_SAT_TILT;
	}

	// 创建电机输出消息，设置电机消息采样和发布时间并初始化为 NaN。
	actuator_motors_s motor_speed{};
	motor_speed.timestamp_sample = hrt_absolute_time();
	motor_speed.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_speed.control[i] = NAN;
	}

	// 角速度平方与归一化推力线性对应，避免重复平方引入非线性放大。
	const auto omega_sq_to_normalized_thrust = [hover_omega_sq, hover_throttle](float omega_sq) {
		return math::constrain(hover_throttle * omega_sq / hover_omega_sq, 0.0f, 1.0f);
	};

	// 发布电机角速度平方转换后的归一化推力。
	motor_speed.control[0] = omega_sq_to_normalized_thrust(motor_1_sq);
	motor_speed.control[1] = omega_sq_to_normalized_thrust(motor_2_sq);
	motor_speed.control[2] = omega_sq_to_normalized_thrust(motor_3_sq);
	motor_speed.control[3] = omega_sq_to_normalized_thrust(motor_4_sq);

	// 创建倾转舵机输出消息，设置舵机消息采样和发布时间并初始化为 NaN。
	actuator_servos_s motor_tilt{};
	motor_tilt.timestamp_sample = hrt_absolute_time();
	motor_tilt.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		motor_tilt.control[i] = NAN;
	}

	// 将四路倾转角归一化到 [-1, 1] 范围，供舵机控制。
	motor_tilt.control[0] = math::constrain(alpha_offset1 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[1] = math::constrain(alpha_offset2 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[2] = math::constrain(alpha_offset3 / tilt_angle_max_rad, -1.0f, 1.0f);
	motor_tilt.control[3] = math::constrain(alpha_offset4 / tilt_angle_max_rad, -1.0f, 1.0f);

	// 缓存并发布执行器输出。
	_last_motor_output = motor_speed;
	_last_tilt_output = motor_tilt;
	_last_actuator_output_valid = true;

	_motor_speed_pub_raw.publish(motor_speed);
	_motor_tilt_pub_raw.publish(motor_tilt);
}

void FullvectorControl::controlAllocation(const UAVStates &state, const UAVCommand &command)
{
	// 读取当前欧拉角。
	const Vector3f &euler_cur = state.Euler_angles;
	const float roll_rad = euler_cur(0);
	const float pitch_rad = euler_cur(1);
	const float yaw_rad = euler_cur(2);
	// 计算并发布真实执行器指令。
	calculateMotorCommand(state, command);

	// 计算四路电机角速度平方。
	const float m1_sq = motor_1 * motor_1;
	const float m2_sq = motor_2 * motor_2;
	const float m3_sq = motor_3 * motor_3;
	const float m4_sq = motor_4 * motor_4;
	// 计算 45° 方向投影系数。
	const float c = sqrtf(2.0f) * 0.5f;

	// 计算机体系下的推力分量。
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

	// 计算 NED 坐标系下加速度。
	const float mass_safe = math::max(mass, 1e-3f);
	const float dv_x = - ((cosf(pitch_rad) * cosf(yaw_rad)) * Fx
			      + (cosf(yaw_rad) * sinf(pitch_rad) * sinf(roll_rad) - sinf(yaw_rad) * cosf(roll_rad)) * Fy
			      + (cosf(yaw_rad) * sinf(pitch_rad) * cosf(roll_rad) + sinf(yaw_rad) * sinf(roll_rad)) * Fz) / mass_safe;

	const float dv_y = - ((cosf(pitch_rad) * sinf(yaw_rad)) * Fx
			      + (sinf(yaw_rad) * sinf(pitch_rad) * sinf(roll_rad) + cosf(yaw_rad) * cosf(roll_rad)) * Fy
			      + (sinf(yaw_rad) * sinf(pitch_rad) * cosf(roll_rad) - cosf(yaw_rad) * sinf(roll_rad)) * Fz) / mass_safe;

	const float dv_z = gravity - ((-sinf(pitch_rad)) * Fx
				      + (sinf(roll_rad) * cosf(pitch_rad)) * Fy
				      + (cosf(roll_rad) * cosf(pitch_rad)) * Fz) / mass_safe;

	// 一步预测位置和速度。
	const Vector3f acc_world(dv_x, dv_y, dv_z);
	const float dt = math::max(_dt, 0.0f);
	const Vector3f vel_integrated = state.velocity + acc_world * dt;
	const Vector3f pos_integrated = state.position + state.velocity * dt + 0.5f * acc_world * dt * dt;

	// 合成旋翼反扭矩
	const float Qx = 0.0f;
	const float Qy = 0.0f;
	const float Qz = K_M * m1_sq * cosf(alpha_offset1)
			 + K_M * m2_sq * cosf(alpha_offset2)
			 - K_M * m3_sq * cosf(alpha_offset3)
			 - K_M * m4_sq * cosf(alpha_offset4);

	// 计算机体系下的力矩。
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

	// 读取角速度及转动惯量。
	const float p = state.angular_velocity(0);
	const float q = state.angular_velocity(1);
	const float r = state.angular_velocity(2);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float rotor_mix = (motor_1 + motor_2 - motor_3 - motor_4);

	// 计算机体系下的角加速度。
	const float dp = (tau_x + q * r * (I_yy - I_zz) + J_RP * q * rotor_mix) / I_xx;
	const float dq = (tau_y + p * r * (I_zz - I_xx) - J_RP * p * rotor_mix) / I_yy;
	const float dr = (tau_z + p * q * (I_xx - I_yy)) / I_zz;

	// 一步预测机体系角速度。
	const Vector3f ang_acc_body(dp, dq, dr);
	// 保存实际可实现控制量及其相对请求值的残差。
	_allocation_accel_achieved = acc_world;
	_allocation_ang_acc_achieved = ang_acc_body;
	_allocation_accel_residual = _allocation_accel_achieved - _pos_acc_cmd;
	_allocation_ang_acc_residual = _allocation_ang_acc_achieved - _att_ang_acc_cmd;

	const bool collective_saturated = (_allocation_saturation_flags
					   & (fullvector_control_status_s::ALLOC_SAT_COLLECTIVE_LOW
					      | fullvector_control_status_s::ALLOC_SAT_COLLECTIVE_HIGH)) != 0;
	const bool differential_saturated = (_allocation_saturation_flags
					     & fullvector_control_status_s::ALLOC_SAT_DIFFERENTIAL) != 0;
	const bool tilt_saturated = (_allocation_saturation_flags & fullvector_control_status_s::ALLOC_SAT_TILT) != 0;
	const bool yaw_tilt_saturated = (_allocation_saturation_flags
					 & fullvector_control_status_s::ALLOC_SAT_YAW_TILT) != 0;

	// 只在饱和确实阻碍当前请求方向时启用对应轴反算。
	const auto saturation_opposes_request = [](float request, float residual) {
		return (fabsf(request) > FLT_EPSILON) && (request * residual < 0.0f);
	};

	_position_allocation_saturated[0] = tilt_saturated
					    && saturation_opposes_request(_pos_acc_cmd(0), _allocation_accel_residual(0));
	_position_allocation_saturated[1] = tilt_saturated
					    && saturation_opposes_request(_pos_acc_cmd(1), _allocation_accel_residual(1));
	_position_allocation_saturated[2] = (collective_saturated || tilt_saturated)
					    && saturation_opposes_request(_pos_acc_cmd(2), _allocation_accel_residual(2));
	_attitude_allocation_saturated[0] = (differential_saturated || tilt_saturated)
					    && saturation_opposes_request(_att_ang_acc_cmd(0), _allocation_ang_acc_residual(0));
	_attitude_allocation_saturated[1] = (differential_saturated || tilt_saturated)
					    && saturation_opposes_request(_att_ang_acc_cmd(1), _allocation_ang_acc_residual(1));
	_attitude_allocation_saturated[2] = (differential_saturated || yaw_tilt_saturated)
					    && saturation_opposes_request(_att_ang_acc_cmd(2), _allocation_ang_acc_residual(2));

	const Vector3f omega_integrated = state.angular_velocity + ang_acc_body * dt;
	const float p_new = omega_integrated(0);
	const float q_new = omega_integrated(1);
	const float r_new = omega_integrated(2);

	// 转换欧拉角速率并保护 pitch 奇异点。
	const float cos_pitch = math::max(fabsf(cosf(pitch_rad)), 1e-4f);
	const float tan_pitch = sinf(pitch_rad) / cos_pitch;
	const float dphi = p_new + q_new * tan_pitch * sinf(roll_rad) + r_new * tan_pitch * cosf(roll_rad);
	const float dtheta = q_new * cosf(roll_rad) - r_new * sinf(roll_rad);
	const float dpsi = (q_new * sinf(roll_rad) + r_new * cosf(roll_rad)) / cos_pitch;

	// 一步预测姿态；这些局部结果仅校核动力学模型，不进入闭环或 uORB 输出。
	const Vector3f euler_integrated = euler_cur + Vector3f(dphi, dtheta, dpsi) * dt;
	const Quatf q_integrated(Eulerf(euler_integrated(0), euler_integrated(1), euler_integrated(2)));
	(void)pos_integrated;
	(void)vel_integrated;
	(void)q_integrated;
	(void)omega_integrated;
}

int FullvectorControl::task_spawn(int argc, char *argv[])
{
	// 创建模块并挂载到 WorkQueue。
	FullvectorControl *instance = new FullvectorControl();

	// 检查实例是否创建成功。
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	// 初始化失败时释放实例。
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

// 处理未知命令。
int FullvectorControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// 打印模块使用说明。
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

// 声明模块入口函数。
extern "C" __EXPORT int fullvector_control_main(int argc, char *argv[])
{
	return FullvectorControl::main(argc, argv);
}
