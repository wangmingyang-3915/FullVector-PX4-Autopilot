/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_euler_attitude.h>

class EulerAttitudeBridge : public ModuleBase<EulerAttitudeBridge>, public ModuleParams
{
public:
	EulerAttitudeBridge() = default;
	~EulerAttitudeBridge() override = default;

	static int task_spawn(int argc, char *argv[]);
	static EulerAttitudeBridge *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	uORB::Publication<vehicle_euler_attitude_s> _euler_pub{ORB_ID(vehicle_euler_attitude)};
	uint64_t _publish_count{0};
};
