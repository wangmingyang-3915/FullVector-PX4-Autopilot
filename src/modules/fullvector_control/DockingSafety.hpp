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

#include <matrix/matrix/math.hpp>
#include <cmath>

// 对接目标的姿态在一次会话内固定；只比较旋转，不要求目标位置固定。
class DockingAttitudeGuard
{
public:
	bool accept(const matrix::Quatf &body_to_ned, const matrix::Quatf &body_to_target, float tolerance)
	{
		const matrix::Quatf target_to_ned = body_to_ned * body_to_target.inversed();

		if (!target_to_ned.isAllFinite() || !std::isfinite(tolerance) || tolerance <= 0.0f) {
			return false;
		}

		if (!_initialized) {
			_reference = target_to_ned.normalized();
			_initialized = true;
			return true;
		}

		// abs(dot) 同时接受 q 和 -q；不跟随可疑样本更新基准，避免慢漂移绕过门限。
		const float dot = fminf(fabsf(_reference.dot(target_to_ned.normalized())), 1.0f);
		return 2.0f * acosf(dot) <= tolerance;
	}

	void applyEstimatorReset(const matrix::Quatf &delta)
	{
		if (_initialized) {
			_reference = (delta * _reference).normalized();
		}
	}

	void reset() { _initialized = false; }

private:
	matrix::Quatf _reference{};
	bool _initialized{false};
};

// 只限制对接会话的 yaw rate 目标，不改变姿态/角速度串级 PID。
class DockingYawLimiter
{
public:
	float update(float requested_rate, float measured_rate, float rate_max, float accel_max, float dt)
	{
		rate_max = fmaxf(rate_max, 0.0f);
		accel_max = fmaxf(accel_max, 0.0f);

		if (!_initialized) {
			_rate = constrain(measured_rate, rate_max);
			_initialized = true;
		}

		const float requested = constrain(requested_rate, rate_max);
		const float step = accel_max * fmaxf(dt, 0.0f);
		_rate = constrain(_rate + constrain(requested - _rate, step), rate_max);
		return _rate;
	}

	void reset() { _initialized = false; }

private:
	static float constrain(float value, float limit) { return fminf(fmaxf(value, -limit), limit); }
	float _rate{0.0f};
	bool _initialized{false};
};
