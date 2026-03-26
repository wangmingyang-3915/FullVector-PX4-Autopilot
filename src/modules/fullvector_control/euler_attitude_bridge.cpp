/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "euler_attitude_bridge.h"

#include <errno.h>
#include <float.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

extern "C" __EXPORT int euler_attitude_bridge_main(int argc, char *argv[]);

int EulerAttitudeBridge::print_status()
{
	PX4_INFO("running, published=%llu", (unsigned long long)_publish_count);
	return 0;
}

int EulerAttitudeBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EulerAttitudeBridge::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("euler_attitude_bridge",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return PX4_OK;
}

EulerAttitudeBridge *EulerAttitudeBridge::instantiate(int argc, char *argv[])
{
	return new EulerAttitudeBridge();
}

void EulerAttitudeBridge::run()
{
	const int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	px4_pollfd_struct_t fds[1]{};
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;

	while (!should_exit()) {
		const int pret = px4_poll(fds, 1, 1000);

		if (pret < 0) {
			PX4_ERR("poll error %d", pret);
			px4_usleep(50000);
			continue;
		}

		if (pret == 0 || !(fds[0].revents & POLLIN)) {
			continue;
		}

		vehicle_attitude_s att{};
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

		const float q0 = att.q[0];
		const float q1 = att.q[1];
		const float q2 = att.q[2];
		const float q3 = att.q[3];

		if (!PX4_ISFINITE(q0) || !PX4_ISFINITE(q1) || !PX4_ISFINITE(q2) || !PX4_ISFINITE(q3)) {
			continue;
		}

		const float q_norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

		if (q_norm_sq <= FLT_EPSILON) {
			continue;
		}

		const float inv_norm = 1.0f / sqrtf(q_norm_sq);
		const float qw = q0 * inv_norm;
		const float qx = q1 * inv_norm;
		const float qy = q2 * inv_norm;
		const float qz = q3 * inv_norm;

		const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
		const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
		const float roll = atan2f(sinr_cosp, cosr_cosp);

		const float sinp = 2.0f * (qw * qy - qz * qx);
		const float pitch = asinf(math::constrain(sinp, -1.0f, 1.0f));

		const float siny_cosp = 2.0f * (qw * qz + qx * qy);
		const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
		const float yaw = atan2f(siny_cosp, cosy_cosp);

		vehicle_euler_attitude_s euler{};
		euler.timestamp_sample = att.timestamp_sample;
		euler.timestamp = hrt_absolute_time();
		euler.roll = roll;
		euler.pitch = pitch;
		euler.yaw = yaw;

		_euler_pub.publish(euler);
		_publish_count++;
	}

	orb_unsubscribe(vehicle_attitude_sub);
}

int EulerAttitudeBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes `vehicle_euler_attitude` by converting `vehicle_attitude` quaternion to Euler angles.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("euler_attitude_bridge", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int euler_attitude_bridge_main(int argc, char *argv[])
{
	return EulerAttitudeBridge::main(argc, argv);
}
