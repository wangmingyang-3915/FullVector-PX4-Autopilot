/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file fullvector_control.hpp
 *
 * 全矢量四旋翼控制器声明、uORB 接口与可调参数。
 *
 * @author Mingyang Wang <3112311639@qq.com>
 */

#pragma once

#include "RelativePoseStateMachine.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>

// uORB 话题。
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/fullvector_control_diagnostics.h>
#include <uORB/topics/fullvector_control_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/target_relative_pose.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

using namespace time_literals;
using namespace matrix;

// 估计器状态：平移量使用 NED，转动量使用机体 FRD。
struct UAVStates {
	Vector3f position;                      // NED 位置，m。
	Vector3f velocity;                      // NED 速度，m/s。
	Vector3f Euler_angles;                  // 欧拉角，rad。
	Quatf attitude;                         // 姿态四元数。
	Vector3f angular_velocity;              // 机体系角速度，rad/s。
};

// 控制目标：位置环使用 NED，姿态环使用欧拉角和机体系角速度。
struct UAVCommand {
	Vector3f position;                      // NED 位置，m。
	Vector3f velocity;                      // NED 速度，m/s。
	Vector3f acceleration;                  // NED 加速度，m/s²。
	Vector3f Euler_angles;                  // 欧拉角，rad。
	Vector3f angular_velocity;              // 机体系角速度，rad/s。
};

class FullvectorControl : public ModuleBase<FullvectorControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FullvectorControl();
	~FullvectorControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/**
	 * 运行位置和速度串级 PID。
	 * @param state 当前飞行器状态
	 * @param command 控制目标
	 * @param dt 控制周期，s
	 */
	void PositionControl(const UAVStates &state, const UAVCommand &command, const float dt);

	/**
	 * 运行姿态和角速度串级 PID。
	 * @param state 当前飞行器状态
	 * @param command 控制目标
	 * @param dt 控制周期，s
	 * @param stabilized_mode 是否为 STAB 模式
	 */
	void AttitudeControl(const UAVStates &state, UAVCommand &command, const float dt, bool stabilized_mode);

	void calculateMotorCommand(const UAVStates &state, const UAVCommand &command); // 生成执行器指令。
	void controlAllocation(const UAVStates &state, const UAVCommand &command); // 执行分配与响应预测。

private:
	enum class AttitudeErrorSource : uint8_t {
		Absolute = 0,
		RelativeYaw,
		FullRelative,
	};

	void Run() override;

	/**
	 * 同步 PX4 参数。
	 * @param force 是否强制更新
	 */
	void parameters_update(bool force);
	void resetPositionPidState();
	void resetAttitudePidState();
	void resetPidState();
	void resetRelativePoseSession();
	void publishSafeActuatorFallback();
	bool publishLastActuatorCommand();
	void publishNeutralTiltServos();
	void printControlDebug(hrt_abstime now);
	void publishFullvectorControlDiagnostics(uint8_t output_state, uint8_t fallback_reason);
	void publishFullvectorControlStatus(bool fullvector_active, bool native_requested, bool rc_switch_valid,
					    float rc_switch_value);
	// 对接失联回退和输入门控。
	void requestPositionControlFallback(hrt_abstime now);
	bool evaluateNativeControllerRequest(float &rc_switch_value, bool &rc_switch_valid);
	bool validateRelativePoseSample(const target_relative_pose_s &candidate, hrt_abstime now);

	bool updateUAVState();
	bool updateAttitudeStateOnly();
	void updateAttitudeAndAngularVelocity();
	bool shouldLogFaultWarning(hrt_abstime now);

	// 控制循环性能计数器。
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FV_ENABLE>) _param_fv_enable,
		(ParamBool<px4::params::FV_DBG_EN>) _param_fv_debug_enable,
		// RC 控制权切换。
		(ParamBool<px4::params::FV_RC_SW_EN>) _param_fv_rc_sw_en,
		(ParamInt<px4::params::FV_RC_SW_CH>) _param_fv_rc_sw_ch,
		(ParamFloat<px4::params::FV_RC_SW_THR>) _param_fv_rc_sw_thr,
		(ParamBool<px4::params::FV_RC_SW_REV>) _param_fv_rc_sw_rev,
		// Offboard 相对位姿控制。
		(ParamFloat<px4::params::FV_REL_POS_X>) _param_fv_rel_pos_x,
		(ParamFloat<px4::params::FV_REL_POS_Y>) _param_fv_rel_pos_y,
		(ParamFloat<px4::params::FV_REL_POS_Z>) _param_fv_rel_pos_z,
		(ParamFloat<px4::params::FV_REL_LOSS_T>) _param_fv_rel_loss_t,
		(ParamFloat<px4::params::FV_REL_DBNC_T>) _param_fv_rel_debounce_t,
		// 相对位姿最长保持和异常值门控。
		(ParamFloat<px4::params::FV_REL_HOLD_T>) _param_fv_rel_hold_t,
		(ParamFloat<px4::params::FV_REL_POS_JMP>) _param_fv_rel_pos_jump,
		(ParamFloat<px4::params::FV_REL_VEL_G>) _param_fv_rel_velocity_gate,
		(ParamFloat<px4::params::FV_REL_ANG_JMP>) _param_fv_rel_angle_jump,
		(ParamFloat<px4::params::FV_REL_RATE_G>) _param_fv_rel_rate_gate,
		(ParamInt<px4::params::FV_REL_ATT_MODE>) _param_fv_rel_att_mode,
		(ParamFloat<px4::params::FV_REL_ROLL>) _param_fv_rel_roll,
		(ParamFloat<px4::params::FV_REL_PITCH>) _param_fv_rel_pitch,
		(ParamFloat<px4::params::FV_REL_YAW>) _param_fv_rel_yaw,
		(ParamFloat<px4::params::FV_POS_P_X>)             _param_fv_pos_p_x,
		(ParamFloat<px4::params::FV_POS_P_Y>)             _param_fv_pos_p_y,
		(ParamFloat<px4::params::FV_POS_P_Z>)             _param_fv_pos_p_z,
		(ParamFloat<px4::params::FV_POS_I_X>)             _param_fv_pos_i_x,
		(ParamFloat<px4::params::FV_POS_I_Y>)             _param_fv_pos_i_y,
		(ParamFloat<px4::params::FV_POS_I_Z>)             _param_fv_pos_i_z,
		(ParamFloat<px4::params::FV_POS_D_X>)             _param_fv_pos_d_x,
		(ParamFloat<px4::params::FV_POS_D_Y>)             _param_fv_pos_d_y,
		(ParamFloat<px4::params::FV_POS_D_Z>)             _param_fv_pos_d_z,
		(ParamFloat<px4::params::FV_VEL_P_X>)             _param_fv_vel_p_x,
		(ParamFloat<px4::params::FV_VEL_P_Y>)             _param_fv_vel_p_y,
		(ParamFloat<px4::params::FV_VEL_P_Z>)             _param_fv_vel_p_z,
		(ParamFloat<px4::params::FV_VEL_I_X>)             _param_fv_vel_i_x,
		(ParamFloat<px4::params::FV_VEL_I_Y>)             _param_fv_vel_i_y,
		(ParamFloat<px4::params::FV_VEL_I_Z>)             _param_fv_vel_i_z,
		(ParamFloat<px4::params::FV_VEL_D_X>)             _param_fv_vel_d_x,
		(ParamFloat<px4::params::FV_VEL_D_Y>)             _param_fv_vel_d_y,
		(ParamFloat<px4::params::FV_VEL_D_Z>)             _param_fv_vel_d_z,
		(ParamFloat<px4::params::FV_ACC_HOR_MAX>)         _param_fv_acc_hor_max,
		(ParamFloat<px4::params::FV_ACC_UP_MAX>)          _param_fv_acc_up_max,
		(ParamFloat<px4::params::FV_ACC_DOWN_MAX>)        _param_fv_acc_down_max,
		(ParamFloat<px4::params::FV_JERK_MAX>)            _param_fv_jerk_max,
		(ParamFloat<px4::params::FV_ATT_P_X>)             _param_fv_att_p_x,
		(ParamFloat<px4::params::FV_ATT_P_Y>)             _param_fv_att_p_y,
		(ParamFloat<px4::params::FV_ATT_P_Z>)             _param_fv_att_p_z,
		(ParamFloat<px4::params::FV_ATT_I_X>)             _param_fv_att_i_x,
		(ParamFloat<px4::params::FV_ATT_I_Y>)             _param_fv_att_i_y,
		(ParamFloat<px4::params::FV_ATT_I_Z>)             _param_fv_att_i_z,
		(ParamFloat<px4::params::FV_ATT_D_X>)             _param_fv_att_d_x,
		(ParamFloat<px4::params::FV_ATT_D_Y>)             _param_fv_att_d_y,
		(ParamFloat<px4::params::FV_ATT_D_Z>)             _param_fv_att_d_z,
		(ParamFloat<px4::params::FV_ANG_VEL_P_X>)         _param_fv_ang_vel_p_x,
		(ParamFloat<px4::params::FV_ANG_VEL_P_Y>)         _param_fv_ang_vel_p_y,
		(ParamFloat<px4::params::FV_ANG_VEL_P_Z>)         _param_fv_ang_vel_p_z,
		(ParamFloat<px4::params::FV_ANG_VEL_I_X>)         _param_fv_ang_vel_i_x,
		(ParamFloat<px4::params::FV_ANG_VEL_I_Y>)	  _param_fv_ang_vel_i_y,
		(ParamFloat<px4::params::FV_ANG_VEL_I_Z>)	  _param_fv_ang_vel_i_z,
		(ParamFloat<px4::params::FV_ANG_VEL_D_X>)         _param_fv_ang_vel_d_x,
		(ParamFloat<px4::params::FV_ANG_VEL_D_Y>)         _param_fv_ang_vel_d_y,
		(ParamFloat<px4::params::FV_ANG_VEL_D_Z>)         _param_fv_ang_vel_d_z,
		(ParamFloat<px4::params::FV_MASS>)                _param_fv_mass,
		(ParamFloat<px4::params::FV_GRAVITY>)             _param_fv_gravity,
		(ParamFloat<px4::params::FV_MOTOR_DIST>)          _param_fv_motor_distance,
		(ParamFloat<px4::params::FV_K_F>)		  _param_fv_K_F,
		(ParamFloat<px4::params::FV_K_M>)		  _param_fv_K_M,
		(ParamFloat<px4::params::FV_HOVER_THR>)		  _param_fv_hover_thr,
		(ParamFloat<px4::params::FV_TILT_MAX>)		  _param_fv_tilt_max,
		(ParamFloat<px4::params::FV_YAW_TILT_MAX>)	  _param_fv_yaw_tilt_max,
		(ParamFloat<px4::params::FV_YAW_MIX_WT>)	  _param_fv_yaw_mix_wt,
		(ParamFloat<px4::params::FV_INERTIA_XX>)          _param_fv_inertia_xx,
		(ParamFloat<px4::params::FV_INERTIA_YY>)          _param_fv_inertia_yy,
		(ParamFloat<px4::params::FV_INERTIA_ZZ>)	  _param_fv_inertia_zz,
		(ParamFloat<px4::params::FV_J_RP>)		  _param_fv_J_RP
	)

	// 中间控制目标，供模式切换反馈和 ULog 记录。
	uORB::Publication<vehicle_local_position_setpoint_s> _position_controller_output_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_angular_acceleration_setpoint_s> _attitude_controller_output_pub{ORB_ID(vehicle_angular_acceleration_setpoint)};
	// 前四路分别对应四个倾转舵机和四个电机。
	uORB::Publication<actuator_servos_s> _motor_tilt_pub_raw{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_motors_s> _motor_speed_pub_raw{ORB_ID(actuator_motors)};
	// 旁路发布控制器诊断数据，不参与任何控制或仲裁。
	uORB::Publication<fullvector_control_diagnostics_s> _fullvector_control_diagnostics_pub{ORB_ID(fullvector_control_diagnostics)};
	// control_allocator 根据该状态决定是否让出原生输出。
	uORB::Publication<fullvector_control_status_s> _fullvector_control_status_pub{ORB_ID(fullvector_control_status)};
	// 对接失联超时后请求 Commander 切回 POSCTL。
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// 控制器输入。
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	// 本机相对目标机的 6DoF 位姿，父坐标系为目标机 body FRD。
	uORB::Subscription _target_relative_pose_sub{ORB_ID(target_relative_pose)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// uORB 消息缓存。
	vehicle_local_position_s _position{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_control_mode_s _control_mode{};
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_status_s _vehicle_status{};
	sensor_selection_s _sensor_selection{};
	trajectory_setpoint_s _trajectory_setpoint{};
	target_relative_pose_s _target_relative_pose{};
	bool _target_relative_pose_valid{false};
	// 相对位姿状态机和当前周期唯一的相对控制判据。
	RelativePoseStateMachine _relative_pose_state_machine{};
	bool _relative_pose_active{false};
	bool _relative_pose_hold_initialized{false};
	Quatf _relative_attitude{};
	Vector3f _relative_euler{};
	Dcmf _rotation_ned_target{};
	Vector3f _relative_pose_hold_position{};
	float _relative_pose_hold_yaw{0.0f};
	hrt_abstime _last_target_relative_pose_update{0};
	// 相对位姿接收、失联和异常拒绝诊断。
	hrt_abstime _last_accepted_target_pose_update{0};
	hrt_abstime _last_fallback_request{0};
	uint64_t _target_pose_timestamp_sample{0};
	uint8_t _target_pose_target_id{0};
	int64_t _target_pose_time_offset{0};
	uint64_t _target_pose_receive_age{0};
	uint64_t _relative_pose_loss_duration{0};
	uint32_t _relative_pose_reject_count{0};
	uint8_t _relative_pose_reject_reason{0};

	// PID 增益：行表示轴，列表示 P/I/D。
	Matrix3f gain_pos_pid{};
	Matrix3f gain_vel_pid{};
	Matrix3f gain_att_pid{};
	Matrix3f gain_ang_vel_pid{};

	// 飞行器物理参数。
	float mass{};       // 总质量，kg。
	float gravity{};    // 重力加速度，m/s²。
	float distance{};   // 电机中心力臂，m。
	float K_F{};        // 推力系数。
	float K_M{};        // 反扭矩系数。
	float J_RP{};       // 转子和螺旋桨转动惯量。

	Matrix3f inertia{}; // 仅使用对角项。

	// 时间和状态新鲜度；状态等级由各消息时间戳独立计算。
	hrt_abstime _last_run_time{0};
	hrt_abstime _last_fault_warning_time{0};
	hrt_abstime _last_debug_print_time{0};
	hrt_abstime _last_position_update{0};
	hrt_abstime _last_velocity_update{0};
	hrt_abstime _last_attitude_update{0};
	hrt_abstime _last_angular_velocity_update{0};
	float _dt{0.01f};

	bool _controller_was_active{false};
	bool _command_initialized{false};
	// 飞行模式切换与执行器饱和保护。
	uint8_t _last_control_nav_state{vehicle_status_s::NAVIGATION_STATE_MAX};
	Vector3f _posctl_acceleration_previous{};
	bool _posctl_acceleration_previous_valid{false};
	bool _actuator_saturated{false};
	uint32_t _selected_accel_device_id{0};
	uint32_t _selected_gyro_device_id{0};
	// POSCTL 航向目标。
	float _posctl_yaw_sp{0.0f};
	bool _posctl_yaw_sp_initialized{false};
	// STAB 航向目标。
	float _manual_yaw_sp{0.0f};
	bool _manual_yaw_sp_initialized{false};
	bool _manual_yaw_stick_active{false};
	actuator_motors_s _last_motor_output{};
	actuator_servos_s _last_tilt_output{};
	bool _last_actuator_output_valid{false};
	uint8_t _state_age_level{0}; // 0 正常，1 告警，2 保持，3 失效。
	uint8_t _position_state_age_level{0};
	uint8_t _attitude_state_age_level{0};

	UAVStates _current_state;
	UAVCommand _current_command;

	Vector3f _pos_acc_cmd{};       // NED 期望加速度。
	Vector3f _att_ang_acc_cmd{};   // 机体系期望角加速度。

	// 仅用于 ULog 的旁路诊断快照，不参与控制计算。
	bool _diagnostic_control_setpoint_valid{false};
	bool _diagnostic_position_control_active{false};
	Vector3f _diagnostic_acceleration_sp_raw{};
	Vector3f _diagnostic_acceleration_sp_limited{};
	Vector3f _diagnostic_acceleration_sp_final{};
	bool _diagnostic_acceleration_limited{false};
	bool _diagnostic_jerk_limited{false};
	Vector3f _diagnostic_attitude_sp{};
	Vector3f _diagnostic_attitude_error{};
	Vector3f _diagnostic_angular_velocity_sp{};
	Vector3f _diagnostic_angular_velocity_error{};
	uint8_t _diagnostic_attitude_error_source{0};
	uint8_t _diagnostic_motor_upper_saturation_mask{0};
	uint8_t _diagnostic_motor_lower_saturation_mask{0};
	uint8_t _diagnostic_servo_saturation_mask{0};

	// 执行器分配结果，编号顺序为右前、左后、左前、右后。
	float alpha_offset1{0.0f};
	float alpha_offset2{0.0f};
	float alpha_offset3{0.0f};
	float alpha_offset4{0.0f};
	float motor_1{0.0f};
	float motor_2{0.0f};
	float motor_3{0.0f};
	float motor_4{0.0f};

	// 位置和速度 PID 状态。
	Vector3f _pos_error_int{};
	Vector3f _pos_error_prev{};
	Vector3f _vel_error_int{};
	Vector3f _vel_error_prev{};
	bool _pid_state_initialized{false};
	// 有限位置目标锁定对应 NED 轴；NaN 表示该轴仅跟踪速度。
	bool _position_axis_locked[3] {};
	bool _position_outer_uses_relative_pose{false};

	// 姿态和角速度 PID 状态。
	Vector3f _att_error_int{};
	Vector3f _att_error_prev{};
	Vector3f _ang_vel_error_int{};
	Vector3f _ang_vel_error_prev{};
	bool _att_pid_state_initialized{false};
	AttitudeErrorSource _attitude_error_source{AttitudeErrorSource::Absolute};
};
