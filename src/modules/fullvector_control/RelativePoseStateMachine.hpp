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

#include <stdint.h>

// 管理单次 OFFBOARD 对接会话的视觉跟踪状态。
class RelativePoseStateMachine
{
public:
	// Idle 等待目标；Tracking 跟踪；LossHold 保持；Aborted 中止。
	enum class State : uint8_t {
		Idle = 0,
		Tracking,
		LossHold,
		Aborted,
	};

	State update(bool offboard_mode, bool controller_active, bool pose_valid, uint64_t now,
		     uint64_t hold_timeout, uint64_t validity_debounce)
	{
		// 离开 OFFBOARD 或交还控制权即结束会话。
		if (!offboard_mode || !controller_active) {
			reset();
			return _state;
		}

		switch (_state) {
		case State::Idle:

			// 视觉连续有效后开始跟踪。
			if (pose_valid) {
				startTimer(_valid_started, now);

				if (elapsed(_valid_started, now) >= validity_debounce) {
					_state = State::Tracking;
					_valid_started = unset_time;
				}

			} else {
				_valid_started = unset_time;
			}

			break;

		case State::Tracking:

			// 视觉连续无效后进入保持态。
			if (!pose_valid) {
				startTimer(_invalid_started, now);

				if (elapsed(_invalid_started, now) >= validity_debounce) {
					_state = State::LossHold;
					_loss_started = _invalid_started;
					_invalid_started = unset_time;
				}

			} else {
				_invalid_started = unset_time;
			}

			break;

		case State::LossHold:

			// 保持超时则中止；视觉稳定恢复则继续跟踪。
			if (lossDuration(now) > hold_timeout) {
				_state = State::Aborted;
				_valid_started = unset_time;

			} else if (pose_valid) {
				startTimer(_valid_started, now);

				if (elapsed(_valid_started, now) >= validity_debounce) {
					_state = State::Tracking;
					_loss_started = unset_time;
					_valid_started = unset_time;
				}

			} else {
				_valid_started = unset_time;
			}

			break;

		case State::Aborted:
			// 本次会话锁存中止，等待会话复位。
			break;
		}

		return _state;
	}

	// 复位状态与全部滞回计时器。
	void reset()
	{
		_state = State::Idle;
		_loss_started = unset_time;
		_invalid_started = unset_time;
		_valid_started = unset_time;
	}

	State state() const { return _state; }

	// 返回从首次连续无效样本开始的丢失时长。
	uint64_t lossDuration(uint64_t now) const
	{
		if (((_state != State::LossHold) && (_state != State::Aborted))
		    || (_loss_started == unset_time) || (now < _loss_started)) {
			return 0;
		}

		return now - _loss_started;
	}

	// 返回日志使用的状态名称。
	static const char *name(State state)
	{
		switch (state) {
		case State::Idle: return "Idle";

		case State::Tracking: return "Tracking";

		case State::LossHold: return "LossHold";

		case State::Aborted: return "Aborted";
		}

		return "Unknown";
	}

private:
	static constexpr uint64_t unset_time = UINT64_MAX;

	// 仅在首个样本到达时启动计时器。
	static void startTimer(uint64_t &timer, uint64_t now)
	{
		if (timer == unset_time) {
			timer = now;
		}
	}

	// 安全计算计时器已运行时长。
	static uint64_t elapsed(uint64_t timer, uint64_t now)
	{
		return ((timer != unset_time) && (now >= timer)) ? now - timer : 0;
	}

	State _state{State::Idle};
	// 分别记录丢失起点、连续无效起点和连续有效起点。
	uint64_t _loss_started{unset_time};
	uint64_t _invalid_started{unset_time};
	uint64_t _valid_started{unset_time};
};
