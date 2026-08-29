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

// 初始化模块参数及内部状态。
FullvectorControl::FullvectorControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化参数、状态与目标缓存。
	parameters_update(true);

	_current_state.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.velocity = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.attitude = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_current_state.angular_velocity = Vector3f(0.0f, 0.0f, 0.0f);

	_current_command.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.Euler_angles = Vector3f(0.0f, 0.0f, 0.0f);
}

// 释放控制循环性能计数器。
FullvectorControl::~FullvectorControl()
{
	perf_free(_loop_perf);
}

// 初始化工作队列并启动控制循环。
bool FullvectorControl::init()
{
	ScheduleNow();
	return true;
}

// 清理位置与速度控制历史。
void FullvectorControl::resetPositionPidState()
{
	// 清零位置环与速度环历史。
	_pos_error_int.zero();
	_vel_error_int.zero();
	_pid_state_initialized = false;

	// 解除三轴位置锁定。
	for (bool &axis_locked : _position_axis_locked) {
		axis_locked = false;
	}

	// 清除相对位姿控制记录。
	_position_outer_uses_relative_pose = false;
}

// 清理姿态与角速度控制历史。
void FullvectorControl::resetAttitudePidState()
{
	// 清零姿态环、角速度环与航向目标。
	_att_error_int.zero();
	_ang_vel_error_int.zero();
	_att_pid_state_initialized = false;

	// 清除姿态误差源和旧航向目标。
	_attitude_error_source = AttitudeErrorSource::Absolute;
	_posctl_yaw_sp_initialized = false;
	_manual_yaw_sp_initialized = false;
	_manual_yaw_stick_active = false;
}

// 清理全部 PID 历史。
void FullvectorControl::resetPidState()
{
	resetPositionPidState();
	resetAttitudePidState();
	_actuator_saturated = false;
}

// 结束并清理当前对接会话。
void FullvectorControl::resetRelativePoseSession()
{
	_relative_pose_state_machine.reset();
	_relative_pose_active = false;
	_relative_pose_hold_initialized = false;
	_relative_pose_hold_position.zero();
	_relative_pose_hold_yaw = 0.0f;
	_relative_attitude = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_relative_euler.zero();
	_rotation_ned_target.setIdentity();
	_last_accepted_target_pose_update = 0;
	_relative_pose_loss_duration = 0;
	_last_fallback_request = 0;
}

// 发布电机停转、舵机回中的安全输出。
void FullvectorControl::publishSafeActuatorFallback()
{
	// 电机停转，倾转舵机回中。
	actuator_motors_s motor_safe{};
	motor_safe.timestamp_sample = hrt_absolute_time();
	motor_safe.timestamp = hrt_absolute_time();

	// NaN 表示不接管对应通道。
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_safe.control[i] = NAN;
	}

	motor_safe.control[0] = 0.0f;
	motor_safe.control[1] = 0.0f;
	motor_safe.control[2] = 0.0f;
	motor_safe.control[3] = 0.0f;
	_motor_speed_pub_raw.publish(motor_safe);

	// 前四路倾转舵机回中。
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

// 舵机回中，并释放电机控制权。
void FullvectorControl::publishNeutralTiltServos()
{
	// 舵机回中，电机交还原生控制器。
	actuator_servos_s tilt_neutral{};

	tilt_neutral.timestamp_sample = hrt_absolute_time();
	tilt_neutral.timestamp = tilt_neutral.timestamp_sample;

	// 前四路舵机回中，其余通道不接管。
	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
		tilt_neutral.control[i] = NAN;
	}

	for (int i = 0; i < 4 && i < actuator_servos_s::NUM_CONTROLS; i++) {
		tilt_neutral.control[i] = 0.0f;
	}

	_motor_tilt_pub_raw.publish(tilt_neutral);
	_last_actuator_output_valid = false;
}

// 发布控制权、输入健康度与对接状态。
void FullvectorControl::publishFullvectorControlStatus(bool fullvector_active, bool native_requested,
		bool rc_switch_valid, float rc_switch_value)
{
	// 汇总控制权与相对位姿状态。
	fullvector_control_status_s status{};

	// 记录时间戳、控制权与 RC 开关状态。
	status.timestamp = hrt_absolute_time();
	status.fullvector_active = fullvector_active;
	status.native_requested = native_requested;
	status.rc_switch_valid = rc_switch_valid;
	status.rc_switch_value = rc_switch_valid ? rc_switch_value : NAN;

	// 记录对接状态与目标信息。
	status.relative_pose_valid = _target_relative_pose_valid;
	status.relative_pose_active = _relative_pose_active;
	const RelativePoseStateMachine::State relative_pose_state = _relative_pose_state_machine.state();
	status.relative_pose_loss_hold = relative_pose_state == RelativePoseStateMachine::State::LossHold;
	status.relative_pose_hold_timed_out = relative_pose_state == RelativePoseStateMachine::State::Aborted;
	status.target_id = _target_pose_target_id;
	status.target_pose_timestamp_sample = _target_pose_timestamp_sample;
	status.target_pose_time_offset = _target_pose_time_offset;
	status.target_pose_receive_age = _target_pose_receive_age;
	status.relative_pose_loss_duration = _relative_pose_loss_duration;
	status.relative_pose_reject_reason = _relative_pose_reject_reason;
	status.relative_pose_reject_count = _relative_pose_reject_count;

	// 计算最近接收和采样时间差。
	if ((_target_pose_timestamp_sample > 0)
	    && (_target_pose_timestamp_sample <= status.timestamp)) {
		status.target_pose_age = status.timestamp - _target_pose_timestamp_sample;
	}

	// 发布诊断状态。
	_fullvector_control_status_pub.publish(status);
}

// 发布旁路控制诊断快照；该消息不参与控制、仲裁或执行器输出。
void FullvectorControl::publishFullvectorControlDiagnostics(uint8_t output_state, uint8_t fallback_reason)
{
	fullvector_control_diagnostics_s diagnostics{};
	const hrt_abstime now = hrt_absolute_time();
	const auto sample_age = [now](hrt_abstime timestamp) {
		return ((timestamp > 0) && (timestamp <= now)) ? now - timestamp : UINT64_MAX;
	};

	diagnostics.timestamp_sample = now;
	diagnostics.timestamp = now;
	diagnostics.output_state = output_state;
	diagnostics.fallback_reason = fallback_reason;
	diagnostics.control_setpoint_valid = _diagnostic_control_setpoint_valid;
	diagnostics.position_control_active = _diagnostic_position_control_active;
	diagnostics.state_age_level = _state_age_level;
	diagnostics.position_state_age_level = _position_state_age_level;
	diagnostics.attitude_state_age_level = _attitude_state_age_level;
	diagnostics.position_age = sample_age(_last_position_update);
	diagnostics.velocity_age = sample_age(_last_velocity_update);
	diagnostics.attitude_age = sample_age(_last_attitude_update);
	diagnostics.angular_velocity_age = sample_age(_last_angular_velocity_update);
	diagnostics.acceleration_limited = _diagnostic_acceleration_limited;
	diagnostics.jerk_limited = _diagnostic_jerk_limited;
	diagnostics.attitude_error_source = _diagnostic_attitude_error_source;
	diagnostics.actuator_saturated = _actuator_saturated;
	diagnostics.motor_upper_saturation_mask = _diagnostic_motor_upper_saturation_mask;
	diagnostics.motor_lower_saturation_mask = _diagnostic_motor_lower_saturation_mask;
	diagnostics.servo_saturation_mask = _diagnostic_servo_saturation_mask;

	for (int i = 0; i < 3; i++) {
		diagnostics.acceleration_sp_raw[i] = _diagnostic_acceleration_sp_raw(i);
		diagnostics.acceleration_sp_limited[i] = _diagnostic_acceleration_sp_limited(i);
		diagnostics.acceleration_sp_final[i] = _diagnostic_acceleration_sp_final(i);
		diagnostics.attitude_sp[i] = _diagnostic_attitude_sp(i);
		diagnostics.attitude_error[i] = _diagnostic_attitude_error(i);
		diagnostics.angular_velocity_sp[i] = _diagnostic_angular_velocity_sp(i);
		diagnostics.angular_velocity_error[i] = _diagnostic_angular_velocity_error(i);
	}

	_fullvector_control_diagnostics_pub.publish(diagnostics);
}

// 按固定频率输出控制与对接摘要。
void FullvectorControl::printControlDebug(hrt_abstime now)
{
	// 参数关闭时不产生调试日志。
	if (!_param_fv_debug_enable.get()) {
		return;
	}

	constexpr hrt_abstime debug_print_interval = 200_ms;

	if ((_last_debug_print_time != 0) && ((now - _last_debug_print_time) < debug_print_interval)) {
		return;
	}

	_last_debug_print_time = now;
	PX4_INFO("[FV] dock=%s acc=[%.2f %.2f %.2f] ang_acc=[%.2f %.2f %.2f]",
		 RelativePoseStateMachine::name(_relative_pose_state_machine.state()),
		 (double)_pos_acc_cmd(0), (double)_pos_acc_cmd(1), (double)_pos_acc_cmd(2),
		 (double)_att_ang_acc_cmd(0), (double)_att_ang_acc_cmd(1), (double)_att_ang_acc_cmd(2));
}

// 对接失联超时后，限频请求切回 POSCTL。
void FullvectorControl::requestPositionControlFallback(hrt_abstime now)
{
	constexpr hrt_abstime retry_interval = 500_ms;

	// 避免重复发送模式切换命令。
	if ((_last_fallback_request != 0) && ((now - _last_fallback_request) < retry_interval)) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.param1 = 1.0f; // 使用 PX4 自定义主模式。
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

// 检查相对位姿标志、数值与相邻样本跳变。
bool FullvectorControl::validateRelativePoseSample(const target_relative_pose_s &candidate, hrt_abstime now)
{
	// 保留原始时间信息，便于诊断拒绝样本。
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

	// 四元数模长应接近 1。
	const float quaternion_norm_sq = candidate.q[0] * candidate.q[0] + candidate.q[1] * candidate.q[1]
					 + candidate.q[2] * candidate.q[2] + candidate.q[3] * candidate.q[3];

	if (!quaternion_finite || (fabsf(quaternion_norm_sq - 1.0f) >= 0.05f)) {
		reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_QUATERNION;
	}

	const bool basic_valid = reject_reason == fullvector_control_status_s::REL_POSE_REJECT_NONE;

	// 基础检查通过后再检查目标锁定与运动学跳变。
	if (basic_valid && (_last_accepted_target_pose_update != 0)) {
		const float sample_dt = math::constrain((now - _last_accepted_target_pose_update) / 1e6f, 0.0f, 1.0f);

		const bool session_active = _relative_pose_state_machine.state() != RelativePoseStateMachine::State::Idle;

		// 对接期间锁定目标 ID。
		if (session_active && (candidate.target_id != _target_relative_pose.target_id)) {
			reject_reason |= fullvector_control_status_s::REL_POSE_REJECT_TARGET_CHANGE;

		} else if (candidate.target_id == _target_relative_pose.target_id) {
			// 根据采样间隔放宽位置与姿态跳变阈值。
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

	// 仅用完整通过检查的样本更新控制输入。
	_relative_pose_reject_reason = reject_reason;

	if (reject_reason != fullvector_control_status_s::REL_POSE_REJECT_NONE) {
		_relative_pose_reject_count++;
		return false;
	}

	_target_relative_pose = candidate;
	_last_accepted_target_pose_update = now;
	return true;
}

// 读取 RC 开关并判断是否交还原生控制器。
bool FullvectorControl::evaluateNativeControllerRequest(float &rc_switch_value, bool &rc_switch_valid)
{
	rc_switch_value = NAN;
	rc_switch_valid = false;

	// 未启用切换或输入无效时保持 fullvector 控制权。
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

	// 反向参数适配不同拨杆方向。
	return _param_fv_rc_sw_rev.get() ? !high_position : high_position;
}

// 状态短时过期时重发上一拍有效输出。
bool FullvectorControl::publishLastActuatorCommand()
{
	// 仅重发有效的上一拍输出。
	if (!_last_actuator_output_valid) {
		return false;
	}

	// 更新时间戳后重发。
	const hrt_abstime now = hrt_absolute_time();
	_last_motor_output.timestamp_sample = now;
	_last_motor_output.timestamp = now;
	_last_tilt_output.timestamp_sample = now;
	_last_tilt_output.timestamp = now;

	_motor_speed_pub_raw.publish(_last_motor_output);
	_motor_tilt_pub_raw.publish(_last_tilt_output);
	return true;
}

// 更新参数、控制增益与飞行器物理量。
void FullvectorControl::parameters_update(bool force)
{
	// 参数变化后同步增益并清理 PID 历史。
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();

		// 位置误差生成期望速度。
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

		// 速度误差生成期望加速度。
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

		// 姿态误差生成期望角速度。
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

		// 角速度误差生成期望角加速度。
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

		// 更新质量、推力与执行器参数。
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

		// 参数切换后清理旧控制历史。
		resetPidState();
	}
}

// 更新完整状态并评估各状态量的新鲜度。
bool FullvectorControl::updateUAVState()
{
	_state_age_level = 0;
	_position_state_age_level = 0;
	_attitude_state_age_level = 0;

	// 更新位置与速度状态。
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

	// 姿态与角速度是最小闭环状态。
	if ((_last_attitude_update == 0) || (_last_angular_velocity_update == 0)) {
		return false;
	}

	// 位置状态缺失时仅保留姿态闭环。
	const bool position_available = (_last_position_update != 0) && (_last_velocity_update != 0);
	const hrt_abstime elapsed_attitude = hrt_elapsed_time(&_last_attitude_update);
	const hrt_abstime elapsed_ang_vel = hrt_elapsed_time(&_last_angular_velocity_update);

	// 按消息年龄划分正常、保持和失效等级。
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

	// 分别记录位置与姿态等级，支持独立降级。
	// 等级：0 正常，1 告警，2 保持，3 失效。
	_position_state_age_level = position_fail ? 3 : (position_stale ? 2 : (position_aging ? 1 : 0));
	_attitude_state_age_level = attitude_fail ? 3 : (attitude_stale ? 2 : (attitude_aging ? 1 : 0));
	_state_age_level = (_position_state_age_level > _attitude_state_age_level) ? _position_state_age_level :
			   _attitude_state_age_level;

	return true;
}

// 更新 STAB 模式所需的姿态与角速度。
bool FullvectorControl::updateAttitudeStateOnly()
{
	_state_age_level = 0;
	_position_state_age_level = 0;
	_attitude_state_age_level = 0;

	// 更新姿态与角速度。
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

	// STAB 仅按姿态状态划分降级等级。
	_attitude_state_age_level = stale_fail ? 3 : (stale_hold ? 2 : (aging ? 1 : 0));
	_state_age_level = _attitude_state_age_level;

	return true;
}

// 将 uORB 姿态数据写入内部状态。
void FullvectorControl::updateAttitudeAndAngularVelocity()
{
	// 更新姿态并转换为欧拉角。
	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_attitude);
		_current_state.attitude = Quatf(_attitude.q);
		_current_state.Euler_angles = Vector3f(Eulerf(_current_state.attitude));
		_last_attitude_update = _attitude.timestamp;
	}

	// 更新机体 FRD 系角速度。
	if (_vehicle_angular_velocity_sub.updated()) {
		_vehicle_angular_velocity_sub.copy(&_angular_velocity);
		_current_state.angular_velocity = Vector3f(_angular_velocity.xyz[0],
						  _angular_velocity.xyz[1],
						  _angular_velocity.xyz[2]);
		_last_angular_velocity_update = _angular_velocity.timestamp;
	}
}

// 对故障告警执行统一限频。
bool FullvectorControl::shouldLogFaultWarning(hrt_abstime now)
{
	constexpr hrt_abstime warning_interval = 1_s;

	if ((_last_fault_warning_time == 0) || ((now - _last_fault_warning_time) > warning_interval)) {
		_last_fault_warning_time = now;
		return true;
	}

	return false;
}

// 执行单拍输入更新、状态机、控制计算与输出。
void FullvectorControl::Run()
{
	// 响应模块退出请求。
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 按 5 ms 周期调度下一拍。
	ScheduleDelayed(5_ms);

	parameters_update(false);
	const hrt_abstime now = hrt_absolute_time();

	// 每周期重新建立旁路诊断快照，不改变任何控制状态。
	_diagnostic_control_setpoint_valid = false;
	_diagnostic_position_control_active = false;
	_diagnostic_acceleration_limited = false;
	_diagnostic_jerk_limited = false;
	_diagnostic_motor_upper_saturation_mask = 0;
	_diagnostic_motor_lower_saturation_mask = 0;
	_diagnostic_servo_saturation_mask = 0;

	// 更新飞行模式与轨迹目标。
	_vehicle_control_mode_sub.update(&_control_mode);
	_vehicle_status_sub.update(&_vehicle_status);
	_trajectory_setpoint_sub.update(&_trajectory_setpoint);

	// 校验主 IMU；飞行中发生主传感器切换时清理旧传感器对应的控制历史。
	if (_sensor_selection_sub.update(&_sensor_selection)) {
		const bool selection_initialized = (_selected_accel_device_id != 0) && (_selected_gyro_device_id != 0);
		const bool selection_changed = selection_initialized
					       && ((_sensor_selection.accel_device_id != _selected_accel_device_id)
						   || (_sensor_selection.gyro_device_id != _selected_gyro_device_id));

		if (selection_changed) {
			PX4_WARN("primary IMU changed: acc %lu->%lu, gyro %lu->%lu",
				 (unsigned long)_selected_accel_device_id, (unsigned long)_sensor_selection.accel_device_id,
				 (unsigned long)_selected_gyro_device_id, (unsigned long)_sensor_selection.gyro_device_id);
			resetPidState();
			_command_initialized = false;
			_last_run_time = 0;
			_last_control_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
			_posctl_acceleration_previous_valid = false;

		} else if (!selection_initialized && (_sensor_selection.accel_device_id != 0)
			   && (_sensor_selection.gyro_device_id != 0)) {
			PX4_INFO("primary IMU: acc %lu, gyro %lu",
				 (unsigned long)_sensor_selection.accel_device_id,
				 (unsigned long)_sensor_selection.gyro_device_id);
		}

		_selected_accel_device_id = _sensor_selection.accel_device_id;
		_selected_gyro_device_id = _sensor_selection.gyro_device_id;
	}

	// 相对位姿接收超时阈值。
	const hrt_abstime relative_pose_timeout = static_cast<hrt_abstime>(
				math::max(_param_fv_rel_loss_t.get(), 0.1f) * 1_s);

	// 候选样本通过检查后才覆盖上一有效输入。
	target_relative_pose_s relative_pose_candidate{};

	if (_target_relative_pose_sub.update(&relative_pose_candidate)) {
		_last_target_relative_pose_update = now;
		_target_relative_pose_valid = validateRelativePoseSample(relative_pose_candidate, now);
	}

	_target_pose_receive_age = (_last_target_relative_pose_update > 0) ? now - _last_target_relative_pose_update : 0;

	// 接收超时、显式无效或异常样本均标记为无效。
	if ((_last_target_relative_pose_update == 0)
	    || (_target_pose_receive_age > relative_pose_timeout)) {
		_target_relative_pose_valid = false;
		_relative_pose_reject_reason = fullvector_control_status_s::REL_POSE_REJECT_RX_TIMEOUT;
	}

	float rc_switch_value = NAN;
	bool rc_switch_valid = false;
	// 输出前完成控制权仲裁。
	const bool native_requested = evaluateNativeControllerRequest(rc_switch_value, rc_switch_valid);

	// 汇总使能、解锁与飞行模式门控。
	const bool fv_enabled = (_param_fv_enable.get() == 1);
	const bool armed = _control_mode.flag_armed;
	const bool posctl_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL);
	const bool offboard_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
	const bool stabilized_mode = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB);
	const bool termination_mode = _control_mode.flag_control_termination_enabled
				      || (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION);
	const bool fullvector_mode_allowed = posctl_mode || offboard_mode || stabilized_mode;
	const bool module_active = fv_enabled && armed && !termination_mode;
	const bool fullvector_active = module_active && fullvector_mode_allowed && !native_requested;

	// 仅在 fullvector 接管 OFFBOARD 时运行对接状态机。
	using RelativePoseState = RelativePoseStateMachine::State;
	const RelativePoseState previous_relative_pose_state = _relative_pose_state_machine.state();

	if (!offboard_mode || !fullvector_active) {
		resetRelativePoseSession();

	} else {
		// 保持超时负责安全退出，滞回时间抑制视觉状态抖动。
		const hrt_abstime maximum_hold_time = static_cast<hrt_abstime>(
				math::max(_param_fv_rel_hold_t.get(), 0.5f) * 1_s);
		const hrt_abstime validity_debounce = static_cast<hrt_abstime>(
				math::constrain(_param_fv_rel_debounce_t.get(), 0.0f, 0.5f) * 1_s);
		const RelativePoseState relative_pose_state = _relative_pose_state_machine.update(
					true, true, _target_relative_pose_valid, now, maximum_hold_time, validity_debounce);

		// 控制环统一使用状态机输出，避免重复判定。
		_relative_pose_active = relative_pose_state == RelativePoseState::Tracking;
		_relative_pose_loss_duration = _relative_pose_state_machine.lossDuration(now);

		// 首次进入保持态时重新捕获保持目标。
		if ((relative_pose_state == RelativePoseState::LossHold)
		    && (previous_relative_pose_state != RelativePoseState::LossHold)) {
			_relative_pose_hold_initialized = false;
		}

		// 超时中止后请求退出 OFFBOARD。
		if (relative_pose_state == RelativePoseState::Aborted) {
			requestPositionControlFallback(now);
		}
	}

	// 离开模式时清除对应航向目标。
	if (!posctl_mode) {
		_posctl_yaw_sp_initialized = false;
	}

	if (!stabilized_mode) {
		_manual_yaw_sp_initialized = false;
		_manual_yaw_stick_active = false;
	}

	// 模块未激活时释放控制权并清理历史。
	if (!module_active) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		_last_actuator_output_valid = false;
		publishFullvectorControlStatus(false, native_requested, rc_switch_valid, rc_switch_value);
		const uint8_t fallback_reason = !fv_enabled ?
						fullvector_control_diagnostics_s::FALLBACK_REASON_DISABLED :
						(!armed ? fullvector_control_diagnostics_s::FALLBACK_REASON_DISARMED :
						 fullvector_control_diagnostics_s::FALLBACK_REASON_TERMINATION);
		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_NONE, fallback_reason);
		return;
	}

	// 非支持模式不发布执行器控制。
	if (!fullvector_mode_allowed) {
		if (_controller_was_active) {
			resetPidState();
			_last_run_time = 0;
		}

		_controller_was_active = false;
		_command_initialized = false;
		_last_actuator_output_valid = false;
		publishFullvectorControlStatus(false, native_requested, rc_switch_valid, rc_switch_value);
		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_NONE,
				fullvector_control_diagnostics_s::FALLBACK_REASON_UNSUPPORTED_MODE);
		return;
	}

	// RC 请求交权时舵机回中。
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
		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_NEUTRAL_TILT,
				fullvector_control_diagnostics_s::FALLBACK_REASON_NATIVE_REQUEST);
		return;
	}

	// 声明 fullvector 已接管执行器。
	publishFullvectorControlStatus(fullvector_active, false, rc_switch_valid, rc_switch_value);

	// 接管首拍从干净状态启动。
	if (!_controller_was_active) {
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;
		_last_control_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
		_posctl_acceleration_previous_valid = false;
	}

	_controller_was_active = true;

	// 计算控制周期并限制异常值。
	if (_last_run_time == 0) {
		_dt = 0.005f;

	} else {
		_dt = (now - _last_run_time) / 1e6f;
	}

	_last_run_time = now;

	if (!PX4_ISFINITE(_dt) || (_dt <= FLT_EPSILON)) {
		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_NONE,
				fullvector_control_diagnostics_s::FALLBACK_REASON_INVALID_DT);
		return;
	}

	constexpr float dt_clamp_s = 0.05f;
	constexpr float dt_reset_s = 0.1f;

	// 长间隔重置 PID，短时调度抖动仅限制 dt。
	if (_dt > dt_reset_s) {
		resetPidState();
		_dt = 0.01f;

	} else if (_dt > dt_clamp_s) {
		_dt = dt_clamp_s;
	}

	// 根据状态新鲜度执行降级保护。
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

		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_SAFE_FALLBACK,
				fullvector_control_diagnostics_s::FALLBACK_REASON_STATE_UNAVAILABLE);

		return;
	}

	// 姿态严重过期时进入安全保护。
	if (_attitude_state_age_level >= 3) {
		publishSafeActuatorFallback();
		resetPidState();
		_last_run_time = 0;
		_command_initialized = false;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("attitude stale-fail, publishing safe actuator fallback");
		}

		publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_SAFE_FALLBACK,
				fullvector_control_diagnostics_s::FALLBACK_REASON_ATTITUDE_STALE);

		return;
	}

	// 姿态短时过期时保持上一拍输出。
	if (_attitude_state_age_level >= 2) {
		const bool held_last_command = publishLastActuatorCommand();

		if (!held_last_command) {
			publishSafeActuatorFallback();
		}

		resetPidState();
		_last_run_time = 0;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("attitude stale, holding last actuator command");
		}

		publishFullvectorControlDiagnostics(held_last_command ?
				fullvector_control_diagnostics_s::OUTPUT_STATE_HOLD_LAST :
				fullvector_control_diagnostics_s::OUTPUT_STATE_SAFE_FALLBACK,
				fullvector_control_diagnostics_s::FALLBACK_REASON_ATTITUDE_STALE);

		return;
	}

	const UAVStates &state_for_control = _current_state;
	const RelativePoseState relative_pose_state = _relative_pose_state_machine.state();
	const bool relative_pose_holding = (relative_pose_state == RelativePoseState::LossHold)
					   || (relative_pose_state == RelativePoseState::Aborted);

	// 每拍只计算一次相对姿态和目标系到 NED 的旋转。
	if (_relative_pose_active) {
		_relative_attitude = Quatf(_target_relative_pose.q).normalized();
		_relative_euler = Vector3f(Eulerf(_relative_attitude));
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

	// STAB 切入 POSCTL 时只重置位置/速度环，并从当前飞行状态开始接管。
	// 姿态和角速度环保持连续，避免切模破坏已经稳定的内环状态。
	const bool entering_posctl = posctl_mode
				      && (_last_control_nav_state != vehicle_status_s::NAVIGATION_STATE_POSCTL);
	if (entering_posctl) {
		resetPositionPidState();
		_current_command.position = state_for_control.position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_posctl_yaw_sp = state_for_control.Euler_angles(2);
		_posctl_yaw_sp_initialized = true;
		_posctl_acceleration_previous = _pos_acc_cmd;
		_posctl_acceleration_previous_valid = true;
	}

	if (!posctl_mode) {
		_posctl_acceleration_previous_valid = false;
	}

	_last_control_nav_state = _vehicle_status.nav_state;

	// 进入保持态时锁定当前位置与航向。
	if (relative_pose_holding && !_relative_pose_hold_initialized) {
		_relative_pose_hold_position = state_for_control.position;
		_relative_pose_hold_yaw = state_for_control.Euler_angles(2);
		// 保留内环积分，误差源切换仅清理外环历史。
		_relative_pose_hold_initialized = true;
	}

	// 保持态持续使用锁定目标。
	if (relative_pose_holding) {
		_current_command.position = _relative_pose_hold_position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_current_command.Euler_angles = Vector3f(0.0f, 0.0f, _relative_pose_hold_yaw);
		_current_command.angular_velocity.zero();
	}

	// STAB、保持态及超时轨迹不使用轨迹目标。
	const bool trajectory_valid = !stabilized_mode && !relative_pose_holding
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

	// 将有效轨迹目标写入内部命令。
	if (trajectory_valid) {
		// POSCTL 保留 NaN，用于按轴解锁位置环。
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

		// POSCTL 在 yaw 角速度控制与航向保持间切换。
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

	// 平移由电机倾转完成，机体目标保持水平。
	Vector3f attitude_sp_target = _current_command.Euler_angles;
	attitude_sp_target(0) = 0.0f;
	attitude_sp_target(1) = 0.0f;
	attitude_sp_target(2) = yaw_sp_target;

	if (stabilized_mode) {
		// 由摇杆生成 STAB 姿态与加速度目标。
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// STAB 摇杆输入上限。
		constexpr float max_manual_tilt_rad = 0.52f; // 约 30°。
		constexpr float max_manual_yaw_rate = 2.0f; // rad/s。
		constexpr float max_manual_xy_accel = 2.0f; // m/s²。

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
			publishFullvectorControlDiagnostics(fullvector_control_diagnostics_s::OUTPUT_STATE_SAFE_FALLBACK,
					fullvector_control_diagnostics_s::FALLBACK_REASON_MANUAL_THROTTLE_IDLE);
			return;
		}

		// 按 PX4 符号约定生成 roll/pitch 目标。
		attitude_sp_target(0) = roll_stick * max_manual_tilt_rad;
		attitude_sp_target(1) = -pitch_stick * max_manual_tilt_rad;

		// 积分 yaw 输入并限制航向误差。
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
			// yaw 杆释放时捕获航向并清理角速度残留。
			_manual_yaw_sp = current_yaw;
			_att_error_int(2) = 0.0f;
			_att_error_prev(2) = matrix::wrap_pi(_manual_yaw_sp - current_yaw);
			_ang_vel_error_int(2) = 0.0f;
			_ang_vel_error_prev(2) = -_current_state.angular_velocity(2);
		}

		// 松杆后保持释放瞬间的航向。
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

		// 将机体系加速度指令转换到 NED。
		_current_command.position = _current_state.position;
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
		_pos_acc_cmd.zero();
		const float forward_accel = pitch_stick * max_manual_xy_accel;
		const float right_accel = roll_stick * max_manual_xy_accel;
		_pos_acc_cmd(0) = cosf(current_yaw) * forward_accel - sinf(current_yaw) * right_accel;
		_pos_acc_cmd(1) = sinf(current_yaw) * forward_accel + cosf(current_yaw) * right_accel;

		// 由归一化油门反算 NED 垂向加速度。
		_pos_acc_cmd(2) = gravity * (1.0f - throttle / hover_throttle);
		_diagnostic_acceleration_sp_raw = _pos_acc_cmd;
		_diagnostic_acceleration_sp_limited = _pos_acc_cmd;
		_diagnostic_acceleration_sp_final = _pos_acc_cmd;
	}

	// 限制姿态目标变化率。
	constexpr float attitude_sp_slew_rate = 1.0f; // rad/s。
	const float attitude_sp_step = attitude_sp_slew_rate * _dt;

	// STAB yaw 与 POSCTL yaw rate 不经过姿态限速。
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
	_diagnostic_position_control_active = run_position_control;

	if (run_position_control) {
		PositionControl(state_for_control, _current_command, _dt);
		const Vector3f acceleration_before_jerk_limit = _pos_acc_cmd;

		if (posctl_mode) {
			// 对 POSCTL 最终位置环输出限制变化率，避免轨迹更新或估计噪声直接形成执行器阶跃。
			const float jerk_max = math::max(_param_fv_jerk_max.get(), 0.0f); // m/s^3

			if (_posctl_acceleration_previous_valid) {
				Vector2f horizontal_delta(_pos_acc_cmd(0) - _posctl_acceleration_previous(0),
							  _pos_acc_cmd(1) - _posctl_acceleration_previous(1));
				const float acceleration_step_max = jerk_max * _dt;
				const float horizontal_delta_norm = horizontal_delta.norm();

				if (horizontal_delta_norm > acceleration_step_max) {
					horizontal_delta *= acceleration_step_max / horizontal_delta_norm;
				}

				_pos_acc_cmd(0) = _posctl_acceleration_previous(0) + horizontal_delta(0);
				_pos_acc_cmd(1) = _posctl_acceleration_previous(1) + horizontal_delta(1);
				_pos_acc_cmd(2) = _posctl_acceleration_previous(2) + math::constrain(
							  _pos_acc_cmd(2) - _posctl_acceleration_previous(2),
							  -acceleration_step_max, acceleration_step_max);
			}

			_posctl_acceleration_previous = _pos_acc_cmd;
			_posctl_acceleration_previous_valid = true;
		}

		_diagnostic_acceleration_sp_final = _pos_acc_cmd;
		_diagnostic_jerk_limited = (_pos_acc_cmd - acceleration_before_jerk_limit).norm_squared() > FLT_EPSILON;

	} else if (!stabilized_mode) {
		resetPositionPidState();
		_pos_acc_cmd.zero();
		_diagnostic_acceleration_sp_raw.zero();
		_diagnostic_acceleration_sp_limited.zero();
		_diagnostic_acceleration_sp_final.zero();
		_posctl_acceleration_previous_valid = false;

		if (shouldLogFaultWarning(now)) {
			PX4_WARN("position stale, skipping position control");
		}
	}

	// 姿态控制后生成并发布执行器指令。
	AttitudeControl(state_for_control, _current_command, _dt, stabilized_mode);
	controlAllocation(state_for_control, _current_command);

	const bool attitude_only_output = !stabilized_mode && !run_position_control;
	const uint8_t diagnostic_fallback_reason = attitude_only_output ?
			fullvector_control_diagnostics_s::FALLBACK_REASON_POSITION_STALE :
			(relative_pose_state == RelativePoseState::Aborted ?
			 fullvector_control_diagnostics_s::FALLBACK_REASON_RELATIVE_POSE_ABORTED :
			 fullvector_control_diagnostics_s::FALLBACK_REASON_NONE);
	publishFullvectorControlDiagnostics(attitude_only_output ?
			fullvector_control_diagnostics_s::OUTPUT_STATE_ATTITUDE_ONLY :
			fullvector_control_diagnostics_s::OUTPUT_STATE_NORMAL,
			diagnostic_fallback_reason);

	// 执行器已经饱和时泄放积分，防止下一拍继续沿饱和方向累积。
	if (_actuator_saturated) {
		const float integral_decay = math::constrain(1.0f - 2.0f * _dt, 0.0f, 1.0f);
		_pos_error_int *= integral_decay;
		_vel_error_int *= integral_decay;
		_att_error_int *= integral_decay;
		_ang_vel_error_int *= integral_decay;
	}

	printControlDebug(now);

	perf_end(_loop_perf);
}

// 由位置与速度误差生成 NED 加速度指令。
void FullvectorControl::PositionControl(const UAVStates &state, const UAVCommand &command, const float dt)
{
	// dt 无效时不更新 PID，避免除零或 NaN 传播。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	const bool use_relative_pose = _relative_pose_active;
	Vector3f ep{};
	bool position_axis_locked[3] {};
	bool initialize_velocity_error = false;

	if (use_relative_pose) {
		// 在目标机 FRD 系计算对接位置误差。
		const Vector3f relative_position(_target_relative_pose.position);
		const Vector3f relative_position_sp(_param_fv_rel_pos_x.get(),
						    _param_fv_rel_pos_y.get(),
						    _param_fv_rel_pos_z.get());
		const Vector3f relative_error_target = relative_position_sp - relative_position;

		// 将位置误差从目标机 FRD 系旋转到 NED。
		ep = _rotation_ned_target * relative_error_target;

		for (bool &axis_locked : position_axis_locked) {
			axis_locked = true;
		}

	} else {
		// 位置目标为 NaN 时，该轴只跟踪速度与加速度。
		for (int i = 0; i < 3; i++) {
			position_axis_locked[i] = PX4_ISFINITE(command.position(i));

			if (position_axis_locked[i]) {
				ep(i) = command.position(i) - state.position(i);
			}
		}
	}

	if (use_relative_pose != _position_outer_uses_relative_pose) {
		// 误差源切换时只重置位置外环历史。
		_pos_error_int.zero();
		_pos_error_prev = ep;
		initialize_velocity_error = true;
		_position_outer_uses_relative_pose = use_relative_pose;

		for (int i = 0; i < 3; i++) {
			_position_axis_locked[i] = position_axis_locked[i];
		}
	}

	if (!_pid_state_initialized) {
		// 首拍同步误差历史，避免微分突变。
		_pos_error_prev = ep;
		initialize_velocity_error = true;
		_pid_state_initialized = true;

		for (int i = 0; i < 3; i++) {
			_position_axis_locked[i] = position_axis_locked[i];
		}
	}

	// 各轴独立管理位置锁定及 PID 历史。
	for (int i = 0; i < 3; i++) {
		if (!position_axis_locked[i]) {
			_pos_error_int(i) = 0.0f;
			_pos_error_prev(i) = 0.0f;

		} else if (!_position_axis_locked[i]) {
			_pos_error_int(i) = 0.0f;
			_pos_error_prev(i) = ep(i);
		}

		// 有限位置目标与 NaN 速度目标互相切换时同步速度误差历史，
		// 避免模式管理器首个 trajectory_setpoint 造成速度 D 项冲击。
		if (position_axis_locked[i] != _position_axis_locked[i]) {
			initialize_velocity_error = true;
		}
	}

	// 位置外环生成期望速度。
	const Vector3f dep = (ep - _pos_error_prev) / dt;
	if (!_actuator_saturated) {
		_pos_error_int += ep * dt;
	}

	// 限制位置积分，避免积分饱和。
	for (int i = 0; i < 3; i++) {
		_pos_error_int(i) = math::constrain(_pos_error_int(i), -5.0f, 5.0f);
	}

	// 读取三轴位置环增益。
	const Vector3f pos_kp{gain_pos_pid(0, 0), gain_pos_pid(1, 0), gain_pos_pid(2, 0)};
	const Vector3f pos_ki{gain_pos_pid(0, 1), gain_pos_pid(1, 1), gain_pos_pid(2, 1)};
	const Vector3f pos_kd{gain_pos_pid(0, 2), gain_pos_pid(1, 2), gain_pos_pid(2, 2)};

	// 对接模式不叠加全局轨迹速度前馈。
	const Vector3f velocity_ff = use_relative_pose ? Vector3f{} : command.velocity;
	Vector3f v_sp = velocity_ff + pos_kp.emult(ep) + pos_ki.emult(_pos_error_int) + pos_kd.emult(dep);

	// 叠加速度前馈并统一限幅。
	for (int i = 0; i < 3; i++) {
		v_sp(i) = math::constrain(v_sp(i), -5.0f, 5.0f);
	}

	// 速度内环生成期望加速度。
	const Vector3f ev = v_sp - state.velocity;

	// 首拍或误差源切换时同步微分历史。
	if (initialize_velocity_error) {
		_vel_error_prev = ev;
	}

	const Vector3f dev = (ev - _vel_error_prev) / dt;
	if (!_actuator_saturated) {
		_vel_error_int += ev * dt;
	}

	// 限制速度积分，避免积分饱和。
	for (int i = 0; i < 3; i++) {
		_vel_error_int(i) = math::constrain(_vel_error_int(i), -3.0f, 3.0f);
	}

	// 读取三轴速度环增益。
	const Vector3f vel_kp{gain_vel_pid(0, 0), gain_vel_pid(1, 0), gain_vel_pid(2, 0)};
	const Vector3f vel_ki{gain_vel_pid(0, 1), gain_vel_pid(1, 1), gain_vel_pid(2, 1)};
	const Vector3f vel_kd{gain_vel_pid(0, 2), gain_vel_pid(1, 2), gain_vel_pid(2, 2)};

	// 对接模式不叠加全局轨迹加速度前馈。
	const Vector3f acceleration_ff = use_relative_pose ? Vector3f{} : command.acceleration;
	Vector3f acc_cmd = acceleration_ff + vel_kp.emult(ev) + vel_ki.emult(_vel_error_int) + vel_kd.emult(dev);
	_diagnostic_acceleration_sp_raw = acc_cmd;

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
		// POSCTL 只输出飞行器能够安全实现的加速度，防止误差扩大后执行器持续顶死。
		const float horizontal_acceleration_max = math::max(_param_fv_acc_hor_max.get(), 0.0f); // m/s^2
		const float upward_acceleration_max = math::max(_param_fv_acc_up_max.get(), 0.0f); // m/s^2
		const float downward_acceleration_max = math::constrain(_param_fv_acc_down_max.get(), 0.0f,
						gravity); // m/s^2
		const float horizontal_acceleration = Vector2f(acc_cmd(0), acc_cmd(1)).norm();

		if (horizontal_acceleration > horizontal_acceleration_max) {
			const float scale = horizontal_acceleration_max / horizontal_acceleration;
			acc_cmd(0) *= scale;
			acc_cmd(1) *= scale;
		}

		// NED Z 轴向下为正，因此向上限幅为负值、向下限幅为正值。
		acc_cmd(2) = math::constrain(acc_cmd(2), -upward_acceleration_max, downward_acceleration_max);
	}

	_diagnostic_acceleration_sp_limited = acc_cmd;
	_diagnostic_acceleration_sp_final = acc_cmd;
	_diagnostic_acceleration_limited = (acc_cmd - _diagnostic_acceleration_sp_raw).norm_squared() > FLT_EPSILON;
	// 保存执行器计算所需的加速度指令。
	_pos_acc_cmd = acc_cmd;

	// 保存下一拍所需误差历史。
	_pos_error_prev = ep;
	_vel_error_prev = ev;

	for (int i = 0; i < 3; i++) {
		_position_axis_locked[i] = position_axis_locked[i];
	}

	// 发布控制器目标，供记录及开环切换反馈使用。
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

// 由姿态与角速度误差生成机体角加速度指令。
void FullvectorControl::AttitudeControl(const UAVStates &state, UAVCommand &command, const float dt,
					bool stabilized_mode)
{
	// 无效 dt 不参与 PID 更新。
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return;
	}

	const bool use_relative_pose = _relative_pose_active;
	const bool full_relative_attitude = _param_fv_rel_att_mode.get() == 1;
	const AttitudeErrorSource attitude_error_source = use_relative_pose
			? (full_relative_attitude ? AttitudeErrorSource::FullRelative : AttitudeErrorSource::RelativeYaw)
			: AttitudeErrorSource::Absolute;
	Vector3f euler_cur;
	Vector3f euler_sp;
	bool initialize_angular_velocity_error = false;

	if (use_relative_pose) {
		if (full_relative_attitude) {
			// 兼容模式：三轴均跟踪视觉相对姿态。
			euler_cur = _relative_euler;
			euler_sp = Vector3f(_param_fv_rel_roll.get(),
					    _param_fv_rel_pitch.get(),
					    _param_fv_rel_yaw.get());

		} else {
			// 默认模式：IMU/EKF 保持滚转、俯仰，视觉对齐偏航。
			euler_cur = Vector3f(Eulerf(state.attitude));
			euler_sp = command.Euler_angles;
			euler_cur(2) = _relative_euler(2);
			euler_sp(2) = _param_fv_rel_yaw.get();
		}

	} else {
		// 非对接模式使用绝对姿态目标。
		euler_cur = Vector3f(Eulerf(state.attitude));
		euler_sp = command.Euler_angles;
	}

	const Vector3f angular_velocity_ff = command.angular_velocity;
	const bool posctl_mode = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL;
	const bool posctl_yaw_rate_active = posctl_mode && (fabsf(angular_velocity_ff(2)) > FLT_EPSILON);
	const bool manual_yaw_control = stabilized_mode || posctl_mode;

	// 姿态误差约束到 [-pi, pi]，避免跨界跳变。
	Vector3f e_att = euler_sp - euler_cur;
	e_att(0) = matrix::wrap_pi(e_att(0));
	e_att(1) = matrix::wrap_pi(e_att(1));
	e_att(2) = matrix::wrap_pi(e_att(2));
	_diagnostic_attitude_sp = euler_sp;
	_diagnostic_attitude_error = e_att;
	_diagnostic_attitude_error_source = static_cast<uint8_t>(attitude_error_source);

	if (attitude_error_source != _attitude_error_source) {
		// 误差源切换时只重置姿态外环历史。
		_att_error_int.zero();
		_att_error_prev = e_att;
		initialize_angular_velocity_error = true;
		_attitude_error_source = attitude_error_source;
	}

	if (!_att_pid_state_initialized) {
		// 首拍同步微分历史。
		_att_error_prev = e_att;
		initialize_angular_velocity_error = true;
		_att_pid_state_initialized = true;
	}

	// 姿态外环生成期望角速度。
	Vector3f de_att = (e_att - _att_error_prev) / dt;

	if (posctl_yaw_rate_active) {
		// 切入 yaw 角速度控制时抑制姿态微分脉冲。
		de_att(2) = 0.0f;
	}

	if (stabilized_mode || posctl_yaw_rate_active) {
		// yaw 角速度控制时停用姿态环 yaw 积分。
		_att_error_int(2) = 0.0f;
	}

	if (!_actuator_saturated) {
		_att_error_int += e_att * dt;
	}

	if (stabilized_mode || posctl_yaw_rate_active) {
		_att_error_int(2) = 0.0f;
	}

	// 限制姿态积分，避免积分饱和。
	for (int i = 0; i < 3; i++) {
		_att_error_int(i) = math::constrain(_att_error_int(i), -1.0f, 1.0f);
	}

	// 读取三轴姿态环增益。
	const Vector3f att_kp{gain_att_pid(0, 0), gain_att_pid(1, 0), gain_att_pid(2, 0)};
	const Vector3f att_ki{gain_att_pid(0, 1), gain_att_pid(1, 1), gain_att_pid(2, 1)};
	const Vector3f att_kd{gain_att_pid(0, 2), gain_att_pid(1, 2), gain_att_pid(2, 2)};

	Vector3f omega_sp = att_kp.emult(e_att) + att_ki.emult(_att_error_int) + att_kd.emult(de_att);

	if (manual_yaw_control) {
		constexpr float max_manual_yaw_rate = 2.0f; // rad/s，与手动 yaw 上限一致。
		omega_sp(2) += angular_velocity_ff(2);
		omega_sp(2) = math::constrain(omega_sp(2), -max_manual_yaw_rate, max_manual_yaw_rate);
	}

	// 保存期望角速度，供内环和诊断使用。
	command.angular_velocity = omega_sp;
	_diagnostic_angular_velocity_sp = omega_sp;

	// 角速度内环生成期望角加速度。
	const Vector3f e_w = omega_sp - state.angular_velocity;
	_diagnostic_angular_velocity_error = e_w;

	// 首拍或误差源切换时同步微分历史。
	if (initialize_angular_velocity_error) {
		_ang_vel_error_prev = e_w;
	}

	const Vector3f de_w = (e_w - _ang_vel_error_prev) / dt;
	if (!_actuator_saturated) {
		_ang_vel_error_int += e_w * dt;
	}

	// 限制角速度积分，避免积分饱和。
	for (int i = 0; i < 3; i++) {
		_ang_vel_error_int(i) = math::constrain(_ang_vel_error_int(i), -3.0f, 3.0f);
	}

	constexpr float yaw_heading_hold_deadband = 0.03f; // rad，航向保持死区。
	constexpr float yaw_rate_hold_deadband = 0.03f; // rad/s，角速度保持死区。
	constexpr float yaw_rate_integral_limit = 0.3f;
	constexpr float max_manual_yaw_accel = 4.0f; // rad/s²，限制差动突变。

	if (manual_yaw_control) {
		// 限制 yaw 角速度积分，避免差动突变。
		_ang_vel_error_int(2) = math::constrain(_ang_vel_error_int(2),
							-yaw_rate_integral_limit,
							yaw_rate_integral_limit);
	}

	// 读取三轴角速度环增益。
	const Vector3f w_kp{gain_ang_vel_pid(0, 0), gain_ang_vel_pid(1, 0), gain_ang_vel_pid(2, 0)};
	const Vector3f w_ki{gain_ang_vel_pid(0, 1), gain_ang_vel_pid(1, 1), gain_ang_vel_pid(2, 1)};
	const Vector3f w_kd{gain_ang_vel_pid(0, 2), gain_ang_vel_pid(1, 2), gain_ang_vel_pid(2, 2)};

	Vector3f ang_acc_cmd = w_kp.emult(e_w) + w_ki.emult(_ang_vel_error_int) + w_kd.emult(de_w);

	if (manual_yaw_control) {
		if ((fabsf(e_att(2)) < yaw_heading_hold_deadband) && (fabsf(e_w(2)) < yaw_rate_hold_deadband)) {
			// 航向与角速度均进入死区后清除残余积分。
			_ang_vel_error_int(2) = 0.0f;
			ang_acc_cmd(2) = w_kp(2) * e_w(2) + w_kd(2) * de_w(2);
		}

		ang_acc_cmd(2) = math::constrain(ang_acc_cmd(2), -max_manual_yaw_accel, max_manual_yaw_accel);
	}

	// 保存执行器分配所需的角加速度。
	_att_ang_acc_cmd = ang_acc_cmd;

	// 保存下一拍所需误差历史。
	_att_error_prev = e_att;
	_ang_vel_error_prev = e_w;

	// 发布姿态控制器输出，便于诊断。
	vehicle_angular_acceleration_setpoint_s attitude_controller_output{};
	attitude_controller_output.timestamp_sample = hrt_absolute_time();
	attitude_controller_output.timestamp = hrt_absolute_time();
	attitude_controller_output.xyz[0] = ang_acc_cmd(0);
	attitude_controller_output.xyz[1] = ang_acc_cmd(1);
	attitude_controller_output.xyz[2] = ang_acc_cmd(2);
	_attitude_controller_output_pub.publish(attitude_controller_output);
	_diagnostic_control_setpoint_valid = true;
}

// 将控制器输出分配为电机转速与倾转舵机指令。
void FullvectorControl::calculateMotorCommand(const UAVStates &state, const UAVCommand &command)
{
	// 读取姿态目标；yaw 由电机差速与公共倾转共同实现。
	const float phi_sp = command.Euler_angles(0);
	const float theta_sp = command.Euler_angles(1);

	// 读取平移与转动控制量。
	const Vector3f &acc_sp_ned = _pos_acc_cmd;
	const Vector3f &ang_acc_sp = _att_ang_acc_cmd;
	const float tilt_angle_max_rad = math::max(_param_fv_tilt_max.get(), 0.01f);
	const float yaw_tilt_max_rad = math::constrain(_param_fv_yaw_tilt_max.get(), 0.0f, tilt_angle_max_rad);

	// 按互补力矩权重拆分 yaw 角加速度。
	const float yaw_motor_mix_weight = math::constrain(_param_fv_yaw_mix_wt.get(), 0.0f, 1.0f);
	const float yaw_motor_ang_acc_sp = yaw_motor_mix_weight * ang_acc_sp(2);
	const float yaw_tilt_ang_acc_sp = (1.0f - yaw_motor_mix_weight) * ang_acc_sp(2);

	// 将 NED 水平加速度旋转到机体 FRD 系。
	const float current_yaw = Vector3f(Eulerf(state.attitude))(2);
	const float cos_yaw = cosf(current_yaw);
	const float sin_yaw = sinf(current_yaw);
	const float acc_sp_body_x = cos_yaw * acc_sp_ned(0) + sin_yaw * acc_sp_ned(1);
	const float acc_sp_body_y = -sin_yaw * acc_sp_ned(0) + cos_yaw * acc_sp_ned(1);

	// 由悬停推力估算基础电机角速度。
	const float kf_safe = math::max(K_F, 1e-6f);
	const float mass_safe = math::max(mass, 1e-3f);
	const float gravity_safe = math::max(gravity, 1e-3f);
	const float distance_safe = math::max(distance, 1e-3f);
	const float arm_d = distance_safe / sqrtf(2.0f);
	const float I_xx = math::max(inertia(0, 0), 1e-6f);
	const float I_yy = math::max(inertia(1, 1), 1e-6f);
	const float I_zz = math::max(inertia(2, 2), 1e-6f);
	const float base_thrust = sqrtf((mass_safe * gravity_safe) / (4.0f * kf_safe));

	// 水平力到倾转角的换算系数。
	const float acc_to_tilt = sqrtf(2.0f) / (mass_safe * gravity_safe);

	// 由姿态与平移指令生成四路基础倾转角。
	// 推力方向与期望水平加速度方向相反。
	const float alpha_base1 =  sqrtf(2.0f) * theta_sp + sqrtf(2.0f) * phi_sp
				   - acc_to_tilt * mass_safe * (acc_sp_body_x - acc_sp_body_y) / 4.0f;
	const float alpha_base2 = -sqrtf(2.0f) * theta_sp - sqrtf(2.0f) * phi_sp
				  + acc_to_tilt * mass_safe * (acc_sp_body_x - acc_sp_body_y) / 4.0f;
	const float alpha_base3 = -sqrtf(2.0f) * theta_sp + sqrtf(2.0f) * phi_sp
				  + acc_to_tilt * mass_safe * (acc_sp_body_x + acc_sp_body_y) / 4.0f;
	const float alpha_base4 =  sqrtf(2.0f) * theta_sp - sqrtf(2.0f) * phi_sp
				   - acc_to_tilt * mass_safe * (acc_sp_body_x + acc_sp_body_y) / 4.0f;


	// 开方时保留控制量符号。
	const auto signed_sqrt = [](float value) {
		return (value >= 0.0f) ? sqrtf(value) : -sqrtf(fabsf(value));
	};

	// 将角加速度与垂向加速度换算为转速增量。
	const float w_roll = signed_sqrt((I_xx * ang_acc_sp(0)) / (4.0f * kf_safe * arm_d));
	const float w_pitch = signed_sqrt((I_yy * ang_acc_sp(1)) / (4.0f * kf_safe * arm_d));
	const float w_yaw = signed_sqrt((I_zz * yaw_motor_ang_acc_sp) / (4.0f * kf_safe * distance_safe));
	const float w_fz = signed_sqrt((mass_safe * (gravity_safe - acc_sp_ned(2))) / (4.0f * kf_safe));

	// 叠加四路电机转速；偏航由两组电机反向差动。
	motor_1 = -w_roll + w_pitch + w_yaw + w_fz;
	motor_2 =  w_roll - w_pitch + w_yaw + w_fz;
	motor_3 =  w_roll + w_pitch - w_yaw + w_fz;
	motor_4 = -w_roll - w_pitch - w_yaw + w_fz;

	// 将电机角速度限制在物理范围内。
	constexpr float motor_speed_max = 20000.0f;
	motor_1 = math::constrain(motor_1, 0.0f, motor_speed_max);
	motor_2 = math::constrain(motor_2, 0.0f, motor_speed_max);
	motor_3 = math::constrain(motor_3, 0.0f, motor_speed_max);
	motor_4 = math::constrain(motor_4, 0.0f, motor_speed_max);

	// 同向公共倾转产生 yaw 力矩，符号按实机机构定义。
	const float motor_sq_sum = math::max(motor_1 * motor_1 + motor_2 * motor_2 + motor_3 * motor_3 + motor_4 * motor_4,
					     1.0f);
	const float tau_z_sp = I_zz * yaw_tilt_ang_acc_sp;
	const float alpha_yaw = math::constrain(tau_z_sp / (kf_safe * distance_safe * motor_sq_sum),
						-yaw_tilt_max_rad, yaw_tilt_max_rad);

	// 叠加 yaw 公共倾转后统一限制舵机行程。
	alpha_offset1 = math::constrain(alpha_base1 + alpha_yaw, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset2 = math::constrain(alpha_base2 + alpha_yaw, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset3 = math::constrain(alpha_base3 + alpha_yaw, -tilt_angle_max_rad, tilt_angle_max_rad);
	alpha_offset4 = math::constrain(alpha_base4 + alpha_yaw, -tilt_angle_max_rad, tilt_angle_max_rad);

	// 按推力与转速平方关系映射到 [0, 1]。
	actuator_motors_s motor_speed{};
	motor_speed.timestamp_sample = hrt_absolute_time();
	motor_speed.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motor_speed.control[i] = NAN;
	}

	const float hover_omega = math::max(base_thrust, 1.0f);
	const float hover_throttle = math::constrain(_param_fv_hover_thr.get(), 0.05f, 0.95f);
	// 以悬停转速为基准归一化推力。
	const auto omega_to_normalized_thrust = [hover_omega, hover_throttle](float omega) {
		const float ratio = omega / hover_omega;
		return math::constrain(hover_throttle * ratio * ratio, 0.0f, 1.0f);
	};

	motor_speed.control[0] = omega_to_normalized_thrust(motor_1);
	motor_speed.control[1] = omega_to_normalized_thrust(motor_2);
	motor_speed.control[2] = omega_to_normalized_thrust(motor_3);
	motor_speed.control[3] = omega_to_normalized_thrust(motor_4);

	// 按最大倾转角归一化舵机位置。
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

	// 记录实际发布端的饱和状态，供下一控制周期执行积分保护。
	_actuator_saturated = false;
	_diagnostic_motor_upper_saturation_mask = 0;
	_diagnostic_motor_lower_saturation_mask = 0;
	_diagnostic_servo_saturation_mask = 0;

	for (int i = 0; i < 4; i++) {
		_actuator_saturated |= (motor_speed.control[i] <= 0.01f) || (motor_speed.control[i] >= 0.99f);
		_actuator_saturated |= fabsf(motor_tilt.control[i]) >= 0.99f;
		const uint8_t actuator_bit = static_cast<uint8_t>(1u << i);

		if (motor_speed.control[i] >= 0.99f) {
			_diagnostic_motor_upper_saturation_mask |= actuator_bit;
		}

		if (motor_speed.control[i] <= 0.01f) {
			_diagnostic_motor_lower_saturation_mask |= actuator_bit;
		}

		if (fabsf(motor_tilt.control[i]) >= 0.99f) {
			_diagnostic_servo_saturation_mask |= actuator_bit;
		}
	}

	// 缓存有效输出，供短时状态掉帧时保持。
	_last_motor_output = motor_speed;
	_last_tilt_output = motor_tilt;
	_last_actuator_output_valid = true;

	_motor_speed_pub_raw.publish(motor_speed);
	_motor_tilt_pub_raw.publish(motor_tilt);
}

// 生成执行器指令，并计算内部刚体响应预测。
void FullvectorControl::controlAllocation(const UAVStates &state, const UAVCommand &command)
{
	// 当前姿态用于机体系到 NED 的力变换。
	const Vector3f euler_cur = Vector3f(Eulerf(state.attitude));
	const float roll_rad = euler_cur(0);
	const float pitch_rad = euler_cur(1);
	const float yaw_rad = euler_cur(2);
	(void)roll_rad;
	(void)pitch_rad;
	(void)yaw_rad;

	// 先生成执行器指令。
	calculateMotorCommand(state, command);

	// 转速平方用于计算推力与反扭矩。
	const float m1_sq = motor_1 * motor_1;
	const float m2_sq = motor_2 * motor_2;
	const float m3_sq = motor_3 * motor_3;
	const float m4_sq = motor_4 * motor_4;
	const float c = sqrtf(2.0f) * 0.5f;

	// 合成四个倾转电机的机体系合力。
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

	// 将机体系力转换为含重力的 NED 加速度。
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

	// 积分预测位置与速度，不写回估计器。
	const Vector3f acc_world(dv_x, dv_y, dv_z);
	const float dt = math::max(_dt, 0.0f);
	const Vector3f vel_integrated = state.velocity + acc_world * dt;
	const Vector3f pos_integrated = state.position + state.velocity * dt + 0.5f * acc_world * dt * dt;

	// 合成反扭矩与倾转推力产生的机体系力矩。
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

	// 由刚体动力学估算机体系角加速度。
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

	// 积分得到预测角速度。
	const Vector3f ang_acc_body(dp, dq, dr);
	const Vector3f omega_integrated = state.angular_velocity + ang_acc_body * dt;
	const float p_new = omega_integrated(0);
	const float q_new = omega_integrated(1);
	const float r_new = omega_integrated(2);

	// 转换欧拉角速率，并保护 pitch 奇异点。
	const float cos_pitch = math::max(fabsf(cosf(pitch_rad)), 1e-4f);
	const float tan_pitch = sinf(pitch_rad) / cos_pitch;
	const float dphi = p_new + q_new * tan_pitch * sinf(roll_rad) + r_new * tan_pitch * cosf(roll_rad);
	const float dtheta = q_new * cosf(roll_rad) - r_new * sinf(roll_rad);
	const float dpsi = (q_new * sinf(roll_rad) + r_new * cosf(roll_rad)) / cos_pitch;

	// 积分预测姿态，不写回估计器。
	const Vector3f euler_integrated = euler_cur + Vector3f(dphi, dtheta, dpsi) * dt;
	const Quatf q_integrated(Eulerf(euler_integrated(0), euler_integrated(1), euler_integrated(2)));
	(void)pos_integrated;
	(void)vel_integrated;
	(void)q_integrated;
	(void)omega_integrated;
	(void)ang_acc_body;
}

// 创建模块实例并启动工作队列。
int FullvectorControl::task_spawn(int argc, char *argv[])
{
	// 创建模块并挂载到工作队列。
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

// 处理模块自定义命令。
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

// 声明 PX4 模块入口。
extern "C" __EXPORT int fullvector_control_main(int argc, char *argv[])
{
	return FullvectorControl::main(argc, argv);
}
