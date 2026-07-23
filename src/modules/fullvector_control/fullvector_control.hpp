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
 * @file fullvector_control_main.hpp
 *
 * Full-vector quadcopter controller declarations, uORB interfaces and tunable parameters.
 *
 * @author Mingyang Wang <3112311639@qq.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <matrix/matrix/math.hpp>


#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>

// Subscriptions
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_land_detected.h>
// 发布 fullvector 是否接管执行器输出的状态，供 control_allocator 做输出仲裁。
#include <uORB/topics/fullvector_control_status.h>
// 读取遥控器 AUX 拨杆输入，用于请求切回 PX4 原生控制器。
#include <uORB/topics/manual_control_setpoint.h>
// 订阅 PX4 当前飞行模式和轨迹目标；模块只在 Run() 中允许的模式下发布 fullvector 输出。
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/target_relative_pose.h>
#include <uORB/topics/vehicle_status.h>

// Publications
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

using namespace time_literals;
using namespace matrix;

// 控制器使用的状态缓存，来自 PX4 估计器发布的 uORB 话题。
struct UAVStates {
	Vector3f position;			// NED 位置，单位 m。
	Vector3f velocity;                      // NED 速度，单位 m/s。
	Vector3f Euler_angles;          	// 当前欧拉角（roll, pitch, yaw），由四元数换算。
	Quatf attitude;                         // 当前姿态四元数。
	Vector3f angular_velocity;              // 机体系角速度，单位 rad/s。
};

// 控制器内部命令缓存，由 trajectory_setpoint 和当前状态共同生成。
struct UAVCommand {
	Vector3f position;			// 期望 NED 位置，单位 m。
	Vector3f velocity;                      // 期望 NED 速度，当前主要用于缓存上层轨迹目标。
	Vector3f acceleration;                  // 期望 NED 加速度，当前主要用于缓存上层轨迹目标。
	Vector3f Euler_angles;          	// 期望欧拉角（roll, pitch, yaw）。
	Vector3f angular_velocity;              // 期望角速度。
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
	 * 执行位置/速度串级 PID，输出期望 NED 加速度。
	 * @param state current UAV state
	 * @param command desired command
	 */
	void PositionControl(const UAVStates &state, const UAVCommand &command, const float dt);

	/**
	 * 执行姿态/角速度串级 PID，输出期望机体系角加速度。
	 * @param state current UAV state
	 * @param command desired command
	 */
	void AttitudeControl(const UAVStates &state, UAVCommand &command, const float dt, bool stabilized_mode);

	void calculateMotorCommand(const UAVCommand &command);
	void controlAllocation(const UAVStates &state, const UAVCommand &command);

private:
	void Run() override;

	/**
	 * 同步 PX4 参数系统到本模块缓存。
	 * @param force 为 true 时即使没有 parameter_update 通知也强制刷新。
	 */
	void parameters_update(bool force);
	void resetPositionPidState();
	void resetAttitudePidState();
	void resetPidState();
	void publishSafeActuatorFallback();
	bool publishLastActuatorCommand();
	// 切回原生控制器时，先让前 4 路倾转舵机回到中立位。
	void publishNeutralTiltServos();
	// 每个周期发布 fullvector 输出归属和遥控切换状态。
	void publishFullvectorControlStatus(bool fullvector_active, bool native_requested, bool rc_switch_valid,
					    float rc_switch_value);
	// 读取配置的 AUX 通道，判断遥控器是否请求 PX4 原生控制器接管。
	bool evaluateNativeControllerRequest(float &rc_switch_value, bool &rc_switch_valid);

	bool updateUAVState();
	bool updateAttitudeStateOnly();

	// 统计 Run() 单次控制循环耗时。
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FV_ENABLE>) _param_fv_enable,
		// 遥控切换参数：启用开关、AUX 通道号、阈值和方向反转。
		(ParamBool<px4::params::FV_RC_SW_EN>) _param_fv_rc_sw_en,
		(ParamInt<px4::params::FV_RC_SW_CH>) _param_fv_rc_sw_ch,
		(ParamFloat<px4::params::FV_RC_SW_THR>) _param_fv_rc_sw_thr,
		(ParamBool<px4::params::FV_RC_SW_REV>) _param_fv_rc_sw_rev,
		// Offboard 相对位姿外环目标，位置采用目标机 body FRD，姿态采用 roll/pitch/yaw。
		(ParamFloat<px4::params::FV_REL_POS_X>) _param_fv_rel_pos_x,
		(ParamFloat<px4::params::FV_REL_POS_Y>) _param_fv_rel_pos_y,
		(ParamFloat<px4::params::FV_REL_POS_Z>) _param_fv_rel_pos_z,
		(ParamFloat<px4::params::FV_REL_ROLL>) _param_fv_rel_roll,
		(ParamFloat<px4::params::FV_REL_PITCH>) _param_fv_rel_pitch,
		(ParamFloat<px4::params::FV_REL_YAW>) _param_fv_rel_yaw,
		(ParamInt<px4::params::PRINT_A_EN>) _param_print_msg_a_en,
		(ParamFloat<px4::params::PRINT_NUM_VALUE>) _param_print_num_value,
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

	// 控制器调试输出和执行器输出。
	uORB::Publication<vehicle_local_position_setpoint_s> _position_controller_output_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_angular_acceleration_setpoint_s> _attitude_controller_output_pub{ORB_ID(vehicle_angular_acceleration_setpoint)};
	// actuator_motors/actuator_servos 发布的是 PX4 期望的归一化输出，前四路对应四个电机和四个倾转舵机。
	uORB::Publication<actuator_servos_s> _motor_tilt_pub_raw{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_motors_s> _motor_speed_pub_raw{ORB_ID(actuator_motors)};
	// 输出归属状态：control_allocator 根据它决定是否让出 PX4 原生 actuator 输出。
	uORB::Publication<fullvector_control_status_s> _fullvector_control_status_pub{ORB_ID(fullvector_control_status)};

	// 控制器运行所需的状态、模式、目标和调试输入订阅。
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	// vehicle_status 提供 nav_state，用于识别 POSCTL/OFFBOARD/TERMINATION 等导航状态。
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	// trajectory_setpoint 是 PX4 上层模式管理或 Offboard 输入生成的位置/速度/加速度/航向目标。
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	// 本机相对目标机的 6DoF 位姿：父坐标系为目标机 body FRD，子坐标系为本机 body FRD。
	uORB::Subscription _target_relative_pose_sub{ORB_ID(target_relative_pose)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	// 遥控器 AUX 输入，供 evaluateNativeControllerRequest() 判断切换请求。
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// 最近一次从 uORB 读取到的原始 PX4 消息。
	vehicle_local_position_s _position{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_control_mode_s _control_mode{};
	// 缓存最新手动控制输入，包含 aux1~aux6 切换通道。
	manual_control_setpoint_s _manual_control_setpoint{};
	// 缓存最新飞行状态和轨迹目标，Run() 中统一转换为 fullvector 控制命令。
	vehicle_status_s _vehicle_status{};
	trajectory_setpoint_s _trajectory_setpoint{};
	target_relative_pose_s _target_relative_pose{};
	bool _target_relative_pose_valid{false};
	hrt_abstime _last_target_relative_pose_update{0};

	// PID 参数矩阵：行对应 X/Y/Z 或 roll/pitch/yaw，列对应 P/I/D。
	Matrix3f gain_pos_pid{};
	Matrix3f gain_vel_pid{};
	Matrix3f gain_att_pid{};
	Matrix3f gain_ang_vel_pid{};

	float mass;	// 飞行器总质量。
	float gravity;	// 重力加速度。
	float distance;	// 电机到机体中心的距离。
	float K_F;	// 电机推力系数。
	float K_M;	// 电机反扭矩系数。
	float J_RP; 	// 转子和螺旋桨绕自身转轴的总转动惯量。

	Matrix3f inertia; // 机体惯性矩阵，目前使用对角项 Ixx/Iyy/Izz。

	// 时间戳和状态新鲜度监测。
	hrt_abstime _last_run_time{0};       	// 上一次进入控制计算的时间。
	hrt_abstime _last_debug_print_time{0};
	hrt_abstime _last_position_update{0};
	hrt_abstime _last_velocity_update{0};
	hrt_abstime _last_attitude_update{0};
	hrt_abstime _last_angular_velocity_update{0};
	float _dt{0.01f};                   	// 当前控制周期，单位 s。

	bool _controller_was_active{false};
	bool _command_initialized{false};
	// 定点模式 yaw 杆控制：进入 POSCTL 时捕获当前航向，松杆后保持释放时的航向。
	float _posctl_yaw_sp{0.0f};
	bool _posctl_yaw_sp_initialized{false};
	// 自稳模式下由 yaw 摇杆积分出的航向目标，进入 STAB 时用当前航向初始化。
	float _manual_yaw_sp{0.0f};
	bool _manual_yaw_sp_initialized{false};
	actuator_motors_s _last_motor_output{};
	actuator_servos_s _last_tilt_output{};
	bool _last_actuator_output_valid{false};
	uint8_t _state_age_level{0}; // 0=fresh, 1=aging, 2=stale-fail
	uint8_t _position_state_age_level{0};
	uint8_t _attitude_state_age_level{0};

	UAVStates _current_state;		// 供控制器使用的最新状态。
	UAVCommand _current_command;		// 供控制器使用的当前目标命令。

	Vector3f _pos_acc_cmd{};		// 位置/速度环输出的期望加速度。
	Vector3f _att_ang_acc_cmd{};		// 姿态/角速度环输出的期望角加速度。

	float alpha_offset1{0.0f};
	float alpha_offset2{0.0f};
	float alpha_offset3{0.0f};
	float alpha_offset4{0.0f};
	float motor_1{0.0f};
	float motor_2{0.0f};
	float motor_3{0.0f};
	float motor_4{0.0f};

	// 位置/速度串级 PID 的积分项和上一拍误差。
	Vector3f _pos_error_int{};
	Vector3f _pos_error_prev{};
	Vector3f _vel_error_int{};
	Vector3f _vel_error_prev{};
	bool _pid_state_initialized{false};
	// true 表示对应 NED 轴当前有有限位置目标；用于无冲击地开关位置外环。
	bool _position_axis_locked[3] {};
	bool _position_outer_uses_relative_pose{false};

	// 姿态/角速度串级 PID 的积分项和上一拍误差。
	Vector3f _att_error_int{};
	Vector3f _att_error_prev{};
	Vector3f _ang_vel_error_int{};
	Vector3f _ang_vel_error_prev{};
	bool _att_pid_state_initialized{false};
	bool _attitude_outer_uses_relative_pose{false};
};
