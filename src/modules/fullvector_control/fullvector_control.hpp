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
 * Parameter description and header file definition for the full-vector quadcopter controller module
 *
 * @author Mingyang Wang <3112311639@qq.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>


#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

// Subscriptions
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_euler_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_land_detected.h>

// Publications
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/actuator_motors.h>

using namespace time_literals;
using namespace matrix;

struct ControlOutputs {
	Vector3f thrust;			// 总推力
	Vector3f moment;			// 控制力矩
};

// 无人机状态结构体
struct UAVStates {
	Vector3f position;			// 位置
	Vector3f velocity;                      // 速度
	Vector3f Euler_angle;                   // 欧拉角
	Matrix3f attitude;                      // 姿态旋转矩阵（由欧拉角转换）
	Vector3f angular_velocity;              // 角速度
};

// 无人机指令结构体
struct UAVCommand {
    	Vector3f position;			// 期望位置
	Vector3f velocity;                      // 期望速度
	Vector3f acceleration;                  // 期望加速度
	Vector3f Euler_angle;                   // 期望欧拉角
	Vector3f angular_velocity;              // 期望角速度
	Matrix3f R_D_prev{};                    // 上一时刻期望旋转矩阵
	Matrix3f R_D_dot_prev{};                // 上一时刻期望旋转矩阵导数
	Vector3f Omega_D_prev{};                // 上一时刻期望角速度
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
	 * Execute geometric position control
	 * @param state current UAV state
	 * @param command desired command
	 * @param control_outputs output control signals
	 */
	void PositionControl(const UAVStates & state, const UAVCommand & command, ControlOutputs & control_outputs);

	/**
	 * Execute geometric attitude control
	 * @param state current UAV state
	 * @param command desired command
	 * @param control_outputs output control signals
	 */
	void AttitudeControl(const UAVStates & state, UAVCommand & command, ControlOutputs & control_outputs);

	void calculateMotorAngleOffset(const UAVStates & state, const UAVCommand & command);

private:
	void Run() override;

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	bool updateUAVState();

	// Performance counter
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FV_ENABLE>) _param_fv_enable,
		(ParamInt<px4::params::FV_USE_TRAJ_SP>) _param_fv_use_traj_sp,
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
		(ParamFloat<px4::params::FV_MOTOR_DISTANCE>)      _param_fv_motor_distance,
		(ParamFloat<px4::params::FV_K_F>)		  _param_fv_K_F,
		(ParamFloat<px4::params::FV_K_M>)		  _param_fv_K_M,
		(ParamFloat<px4::params::FV_INERTIA_XX>)          _param_fv_inertia_xx,
		(ParamFloat<px4::params::FV_INERTIA_YY>)          _param_fv_inertia_yy,
		(ParamFloat<px4::params::FV_INERTIA_ZZ>)	  _param_fv_inertia_zz,
		(ParamFloat<px4::params::FV_J_RP>)		  _param_fv_J_RP,
		(ParamFloat<px4::params::FV_TARGET_X>)		  _param_fv_target_x,
		(ParamFloat<px4::params::FV_TARGET_Y>)		  _param_fv_target_y,
		(ParamFloat<px4::params::FV_TARGET_Z>)		  _param_fv_target_z,
		(ParamFloat<px4::params::FV_TARGET_PITCH>)	  _param_fv_target_pitch,
		(ParamFloat<px4::params::FV_TARGET_YAW>)	  _param_fv_target_yaw,
		(ParamFloat<px4::params::FV_TARGET_ROLL>)	  _param_fv_target_roll,
		(ParamFloat<px4::params::MPC_THR_HOVER>)	  _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MAX>)		  _param_mpc_thr_max
	)

	// Publications
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// Subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_euler_attitude_sub{ORB_ID(vehicle_euler_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// State variables
	vehicle_local_position_s _position{};
	vehicle_euler_attitude_s _attitude_euler{};
    	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_control_mode_s _control_mode{};
	vehicle_status_s _vehicle_status{};
	trajectory_setpoint_s _trajectory_setpoint{};
	vehicle_thrust_setpoint_s _setpoint_thrust{};

	bool _armed{false};
	float printNumValue;
	bool printMsgAEnable;

	// PID 参数矩阵
	Matrix3f gain_pos_pid{};
	Matrix3f gain_vel_pid{};
	Matrix3f gain_att_pid{};
	Matrix3f gain_ang_vel_pid{};

	float mass;	//飞行器总质量
	float distance;	//电机距离机体中心距离
	float K_F;	//电机拉力系数
	float K_M;	//电机力矩系数
	float J_RP; 	//整个电机转子和螺旋桨绕转轴的总转动惯量

	Matrix3f inertia; // 惯性矩阵

	float _hover_thrust{0.5f};
	float _max_thrust{1.0f};

	// 时间相关
	hrt_abstime _last_run_time{0};       	// 上次运行时间
	hrt_abstime _last_debug_print_time{0};
	float _dt{0.01f};                   	// 控制周期（秒）

	UAVStates _current_state;		// 当前无人机状态
	UAVStates _prev_state;			// 上一时刻无人机状态
	UAVCommand _current_command;		// 当前无人机指令
	hrt_abstime _last_external_setpoint_time{0};
	ControlOutputs ctrl_outputs;	    	// 其他成员变量
};
