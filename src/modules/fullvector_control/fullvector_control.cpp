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
#include "GeoMath.hpp"
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
	// 初始化命令（默认使用参数后备目标）
	_current_command.position = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.velocity = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.acceleration = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.Euler_angle = Vector3f(0.0f, 0.0f, 0.0f);
	_current_command.R_D_prev.setIdentity();
	_current_command.R_D_dot_prev.setZero();
	_current_command.Omega_D_prev = Vector3f(0.0f, 0.0f, 0.0f);
	_current_state.attitude.setIdentity();
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
		Matrix3f gain_pos_pid{};
		gain_pos_pid(0, 0) = _param_fv_pos_p_x.get();
		gain_pos_pid(0, 1) = _param_fv_pos_i_x.get();
		gain_pos_pid(0, 2) = _param_fv_pos_d_x.get();
		gain_pos_pid(1, 0) = _param_fv_pos_p_y.get();
		gain_pos_pid(1, 1) = _param_fv_pos_i_y.get();
		gain_pos_pid(1, 2) = _param_fv_pos_d_y.get();
		gain_pos_pid(2, 0) = _param_fv_pos_p_z.get();
		gain_pos_pid(2, 1) = _param_fv_pos_i_z.get();
		gain_pos_pid(2, 2) = _param_fv_pos_d_z.get();

		Matrix3f gain_vel_pid{};
		gain_vel_pid(0, 0) = _param_fv_vel_p_x.get();
		gain_vel_pid(0, 1) = _param_fv_vel_i_x.get();
		gain_vel_pid(0, 2) = _param_fv_vel_d_x.get();
		gain_vel_pid(1, 0) = _param_fv_vel_p_y.get();
		gain_vel_pid(1, 1) = _param_fv_vel_i_y.get();
		gain_vel_pid(1, 2) = _param_fv_vel_d_y.get();
		gain_vel_pid(2, 0) = _param_fv_vel_p_z.get();
		gain_vel_pid(2, 1) = _param_fv_vel_i_z.get();
		gain_vel_pid(2, 2) = _param_fv_vel_d_z.get();

		Matrix3f gain_att_pid{};
		gain_att_pid(0, 0) = _param_fv_att_p_x.get();
		gain_att_pid(0, 1) = _param_fv_att_i_x.get();
		gain_att_pid(0, 2) = _param_fv_att_d_x.get();
		gain_att_pid(1, 0) = _param_fv_att_p_y.get();
		gain_att_pid(1, 1) = _param_fv_att_i_y.get();
		gain_att_pid(1, 2) = _param_fv_att_d_y.get();
		gain_att_pid(2, 0) = _param_fv_att_p_z.get();
		gain_att_pid(2, 1) = _param_fv_att_i_z.get();
		gain_att_pid(2, 2) = _param_fv_att_d_z.get();

		Matrix3f gain_ang_vel_pid{};
		gain_ang_vel_pid(0, 0) = _param_fv_ang_vel_p_x.get();
		gain_ang_vel_pid(0, 1) = _param_fv_ang_vel_i_x.get();
		gain_ang_vel_pid(0, 2) = _param_fv_ang_vel_d_x.get();
		gain_ang_vel_pid(1, 0) = _param_fv_ang_vel_p_y.get();
		gain_ang_vel_pid(1, 1) = _param_fv_ang_vel_i_y.get();
		gain_ang_vel_pid(1, 2) = _param_fv_ang_vel_d_y.get();
		gain_ang_vel_pid(2, 0) = _param_fv_ang_vel_p_z.get();
		gain_ang_vel_pid(2, 1) = _param_fv_ang_vel_i_z.get();
		gain_ang_vel_pid(2, 2) = _param_fv_ang_vel_d_z.get();

		//飞行器总质量及电机距离机体中心距离参数
		mass = _param_fv_mass.get();
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

		_hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.05f, 0.95f);
		_max_thrust = math::constrain(_param_mpc_thr_max.get(), 0.05f, 1.0f);
		_current_command.position = Vector3f(_param_fv_target_x.get(), _param_fv_target_y.get(), _param_fv_target_z.get());
		_current_command.Euler_angle = Vector3f(_param_fv_target_pitch.get(), _param_fv_target_yaw.get(), _param_fv_target_roll.get());
		printMsgAEnable = _param_print_msg_a_en.get();
		printNumValue = _param_print_num_value.get();
	}

}

bool FullvectorControl::updateUAVState()
{
    bool updated = false;

    // 1. 更新位置和速度
    if (_vehicle_local_position_sub.updated()) {
        _vehicle_local_position_sub.copy(&_position);

        // 检查数据有效性
        if (_position.xy_valid && _position.z_valid) {
            _current_state.position = Vector3f(_position.x, _position.y, _position.z);
            updated = true;
            printf("Position updated: [%.2f, %.2f, %.2f]\n",
                   (double)_position.x, (double)_position.y, (double)_position.z);
        }

        if (_position.v_xy_valid && _position.v_z_valid) {
            _current_state.velocity = Vector3f(_position.vx, _position.vy, _position.vz);
            updated = true;
            printf("Velocity updated: [%.2f, %.2f, %.2f]\n",
                   (double)_position.vx, (double)_position.vy, (double)_position.vz);
        }
    }

	// 2. 更新姿态与角速度
	if (_vehicle_euler_attitude_sub.updated()) {
		_vehicle_euler_attitude_sub.copy(&_attitude_euler);

		if (PX4_ISFINITE(_attitude_euler.roll) && PX4_ISFINITE(_attitude_euler.pitch)
	    && PX4_ISFINITE(_attitude_euler.yaw)) {
			_current_state.Euler_angle = Vector3f(_attitude_euler.roll,
									   _attitude_euler.pitch,
									   _attitude_euler.yaw);
			_current_state.attitude = Dcmf(Eulerf(_attitude_euler.roll,
								      _attitude_euler.pitch,
								      _attitude_euler.yaw));
			updated = true;
			printf("Attitude updated (euler): [%.2f, %.2f, %.2f]\n",
			       (double)_attitude_euler.roll, (double)_attitude_euler.pitch,
			       (double)_attitude_euler.yaw);
		}
	}
1
	if (_vehicle_angular_velocity_sub.updated()) {
		_vehicle_angular_velocity_sub.copy(&_angular_velocity);
		_current_state.angular_velocity = Vector3f(_angular_velocity.xyz[0],
											   _angular_velocity.xyz[1],
											   _angular_velocity.xyz[2]);
		updated = true;
		printf("Angular velocity updated: [%.2f, %.2f, %.2f]\n",
		       (double)_angular_velocity.xyz[0], (double)_angular_velocity.xyz[1],
		       (double)_angular_velocity.xyz[2]);
	}

    return updated;
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

	const bool geo_enabled = (_param_fv_enable.get() == 1);
	const bool armed = _control_mode.flag_armed;

	if (!geo_enabled || !armed) {
		return;
	}


	if ((_last_external_setpoint_time == 0) || (hrt_elapsed_time(&_last_external_setpoint_time) > 1_s)) {
		_current_command.position = Vector3f(_param_fv_target_x.get(), _param_fv_target_y.get(), _param_fv_target_z.get());
		_current_command.velocity.zero();
		_current_command.acceleration.zero();
	}

	// 计算时间间隔
	const hrt_abstime now = hrt_absolute_time();
	if (_last_run_time == 0) {
		_last_run_time = now;
	}

	_dt = (now - _last_run_time) / 1e6f;  // 转换为秒
	_last_run_time = now;

	 // 关键：调用状态更新函数
   	updateUAVState();

	perf_begin(_loop_perf);


	if(_param_print_msg_a_en.get())
	{
	   printf("MSG a print  value : %f \r\n",(double)_param_print_num_value.get());
	}else{
	   printf("hello jone\r\n");
	}

	vehicle_thrust_setpoint_s thrust_feedback{};
	const bool thrust_updated = _vehicle_thrust_setpoint_sub.update(&thrust_feedback);

	actuator_motors_s actuator_motors{};
	const bool motors_updated = _actuator_motors_sub.update(&actuator_motors);

	const hrt_abstime now_debug = hrt_absolute_time();

	if ((thrust_updated || motors_updated)) {
		if (thrust_updated) {
			printf("[ThrustSub] thrust_sp=[%.3f, %.3f, %.3f] upward=%.3f\n",
			       (double)thrust_feedback.xyz[0],
			       (double)thrust_feedback.xyz[1],
			       (double)thrust_feedback.xyz[2],
			       (double)(-thrust_feedback.xyz[2]));
		}

		if (motors_updated) {
			printf("[MotorOut] m1=%.3f m2=%.3f m3=%.3f m4=%.3f\n",
			       (double)actuator_motors.control[0],
			       (double)actuator_motors.control[1],
			       (double)actuator_motors.control[2],
			       (double)actuator_motors.control[3]);
		}

		_last_debug_print_time = now_debug;
	}

	// Diagnostic: check takeoff and land detection status
	takeoff_status_s takeoff_status{};
	vehicle_land_detected_s land_detected{};
	_takeoff_status_sub.update(&takeoff_status);
	_vehicle_land_detected_sub.update(&land_detected);

	printf("[FlightState] takeoff_state=%d landed=%d ground_contact=%d\n",
	       (int)takeoff_status.takeoff_state,
	       (int)land_detected.landed,
	       (int)land_detected.ground_contact);

	// 1. 执行位置控制
	PositionControl(_current_state, _current_command, ctrl_outputs);

	// 2. 执行姿态控制
	AttitudeControl(_current_state, _current_command, ctrl_outputs);


	   perf_end(_loop_perf);


}

//位置控制器函数（串级PID控制器）
void FullvectorControl::PositionControl(const UAVStates & state, const UAVCommand & command, ControlOutputs & control_outputs)
{
	// Calculate position and velocity errors
	Vector3f ep = state.position - command.position;
	Vector3f ev = state.velocity - command.velocity;

	// Calculate acceleration: A = -Kv*ev - Kp*ep - m*g*z_hat
	// Using element-wise multiplication with vectors
	Vector3f gravity_term(0.0f, 0.0f, (float)mass * CONSTANTS_ONE_G);

	Vector3f vel_term = _gain_vel_P.emult(ev);
	Vector3f pos_term = _gain_pos_P.emult(ep);
	control_outputs.A = -(vel_term + pos_term + gravity_term);

	// Calculate total thrust value
	float hover_force = (float)mass * CONSTANTS_ONE_G;
	float f_total = -control_outputs.A(2);  // z-component is thrust

	// Hover-thrust calibration:
	// 1) convert force to hover-force ratio
	// 2) map ratio to normalized thrust around calibrated hover thrust
	// 3) apply maximum thrust limit and PX4 z-axis convention (upward thrust is negative z)
	float thrust_norm = 0.0f;

	if (PX4_ISFINITE(f_total) && PX4_ISFINITE(hover_force) && hover_force > FLT_EPSILON) {
		const float force_ratio = math::constrain(f_total / hover_force, 0.0f, 2.0f);
		thrust_norm = math::constrain(force_ratio * _hover_thrust, 0.0f, _max_thrust);
	}

	thrust_norm = math::constrain(thrust_norm, 0.0f, _param_mpc_thr_max.get());

	control_outputs.thrust(0) = 0.0f;
	control_outputs.thrust(1) = 0.0f;
	control_outputs.thrust(2) = -thrust_norm;

	// 发布总推力
	vehicle_thrust_setpoint_s thrust_setpoint{};
	thrust_setpoint.timestamp_sample = hrt_absolute_time();
	thrust_setpoint.timestamp = hrt_absolute_time();
	thrust_setpoint.xyz[0] = 0.0f;
	thrust_setpoint.xyz[1] = 0.0f;
	thrust_setpoint.xyz[2] = control_outputs.thrust(2);

	printf("[PosCtrl] ep=[%.2f, %.2f, %.2f] A=[%.2f, %.2f, %.2f] thrust=%.3f thrust_sp=[%.3f, %.3f, %.3f] upward=%.3f\n",
	       (double)ep(0), (double)ep(1), (double)ep(2),
	       (double)control_outputs.A(0), (double)control_outputs.A(1), (double)control_outputs.A(2),
	       (double)thrust_norm,
	       (double)thrust_setpoint.xyz[0], (double)thrust_setpoint.xyz[1], (double)thrust_setpoint.xyz[2],
	       (double)(-thrust_setpoint.xyz[2]));

	_vehicle_thrust_setpoint_pub.publish(thrust_setpoint);
}

//姿态控制器函数（串级PID控制器）
void FullvectorControl::AttitudeControl(const UAVStates & state, UAVCommand & command, ControlOutputs & control_outputs)
{
	// Calculate desired body z-axis direction (b3D)
	Vector3f b3D = -control_outputs.A.normalized();




		printf("[AttCtrl] b3D=[%.3f, %.3f, %.3f]\n", (double)b3D(0), (double)b3D(1), (double)b3D(2));



	// Calculate desired body x-axis direction (b1D)
	// Keep current heading to avoid unnecessary yaw torque while tracking position/thrust.
	Vector3f b1D = state.attitude.col(0);
	b1D(2) = 0.0f;

	if (b1D.norm_squared() < 1e-6f) {
		b1D = Vector3f(1.0f, 0.0f, 0.0f);
	}

	b1D.normalize();

	// Calculate desired body y-axis direction (b2D)
	Vector3f b2D = b3D.cross(b1D);

	if (b2D.norm_squared() < 1e-6f) {
		b2D = Vector3f(0.0f, 1.0f, 0.0f);
	}

	b2D.normalize();
	b1D = b2D.cross(b3D);
	b1D.normalize();

	// Construct desired rotation matrix
	Matrix3f RD;
	RD.col(0) = b1D;  // x-axis
	RD.col(1) = b2D;  // y-axis
	RD.col(2) = b3D;  // z-axis

	// Calculate rotation error eR (SO3 Lie algebra)
	Vector3f eR = GeoMath::vee((RD.transpose() * state.attitude - state.attitude.transpose() * RD) / 2.0f);

	// 计算前后两个时间期望旋转矩阵的增量delta_R
	Matrix3f delta_R= RD * command.R_D_prev.transpose();

	// 计算期望角速度WD
	Vector3f WD = GeoMath::matToSo3Lie(delta_R) / _dt;

	//计算期望角速度的导数WD_dot
	    	//计算期望旋转矩阵的导数R_D_dot
	Matrix3f RD_dot = RD*GeoMath::hat(WD);
		//计算期望旋转矩阵的二阶导数
	Matrix3f RD_ddot = (RD_dot - command.R_D_dot_prev) / _dt;
	Vector3f WD_dot = GeoMath::vee(RD.transpose() * RD_ddot - RD.transpose() * RD);

	// Calculate angular velocity error
	Vector3f eW = state.angular_velocity - state.attitude.transpose() * RD * command.Omega_D_prev;

	Vector3f att_term = _gain_att_P.emult(eR);
	Vector3f ang_vel_term = _gain_ang_vel_P.emult(eW);
	Vector3f gyro_cross = state.angular_velocity.cross(inertia * state.angular_velocity);
	Vector3f inertia_term = inertia * (GeoMath::hat(state.angular_velocity)*state.attitude.transpose() * RD * WD - state.attitude.transpose() * RD * WD_dot);

	control_outputs.moment = -att_term - ang_vel_term + gyro_cross - inertia_term;

	command.Omega_D_prev = WD;
	command.R_D_prev = RD;
	command.R_D_dot_prev = RD_dot;

	const Vector3f moment_limited{
		math::constrain(control_outputs.moment(0), -1.f, 1.f),
		math::constrain(control_outputs.moment(1), -1.f, 1.f),
		math::constrain(control_outputs.moment(2), -1.f, 1.f)
	};

	control_outputs.moment = moment_limited;

	// 发布控制力矩
	vehicle_torque_setpoint_s torque_setpoint{};
	torque_setpoint.timestamp_sample = hrt_absolute_time();
	torque_setpoint.timestamp = hrt_absolute_time();
	torque_setpoint.xyz[0] = moment_limited(0);
	torque_setpoint.xyz[1] = moment_limited(1);
	torque_setpoint.xyz[2] = moment_limited(2);
	_vehicle_torque_setpoint_pub.publish(torque_setpoint);
}

//电机角度偏移计算函数
void FullvectorControl::calculateMotorAngleOffset(const UAVStates & state, const UAVCommand & command)
{

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
