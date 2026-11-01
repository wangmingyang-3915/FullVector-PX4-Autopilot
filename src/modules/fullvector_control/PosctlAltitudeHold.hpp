/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cmath>
#include <stdint.h>

// POSCTL 回中后跟随原生制动轨迹，轨迹停止后保持其终点高度。
class PosctlAltitudeHold
{
public:
	struct Trajectory {
		float z;
		float vz;
		float az;
	};

	bool update(bool enabled, float throttle, float deadzone, float z, uint8_t reset_counter, float delta_z,
		    const Trajectory &trajectory)
	{
		if (!enabled || !std::isfinite(throttle) || !std::isfinite(z)
		    || !std::isfinite(deadzone) || fabsf(throttle) > deadzone
		    || !std::isfinite(trajectory.z) || !std::isfinite(trajectory.vz) || !std::isfinite(trajectory.az)) {
			reset();
			return false;
		}

		if (!_active) {
			// 制动期间不覆盖位置、速度或加速度目标，保留 FlightTask 的完整轨迹。
			constexpr float stopped_velocity = 0.01f; // m/s
			constexpr float stopped_acceleration = 0.01f; // m/s²

			if (fabsf(trajectory.vz) > stopped_velocity || fabsf(trajectory.az) > stopped_acceleration) {
				return false;
			}

			// 捕获轨迹终点而非回中时的实测高度，避免把飞机拉回制动起点。
			_z = trajectory.z;

		} else if (reset_counter != _reset_counter) {
			// 单次 EKF 重置平移目标；漏过多次重置时重新捕获，避免使用不完整的 delta。
			if (static_cast<uint8_t>(reset_counter - _reset_counter) == 1 && std::isfinite(delta_z)) {
				_z += delta_z;

			} else {
				_z = z;
			}
		}

		_reset_counter = reset_counter;
		_active = true;
		return true;
	}

	void reset() { _active = false; }
	float position() const { return _z; }

private:
	float _z{0.0f};
	uint8_t _reset_counter{0};
	bool _active{false};
};
