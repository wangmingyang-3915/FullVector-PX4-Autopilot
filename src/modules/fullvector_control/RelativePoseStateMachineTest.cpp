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

#include "RelativePoseStateMachine.hpp"

#include <gtest/gtest.h>

using State = RelativePoseStateMachine::State;

static constexpr uint64_t start_time = 1'000'000;
static constexpr uint64_t hold_timeout = 1'500'000;
static constexpr uint64_t validity_debounce = 80'000;

static uint64_t enterTracking(RelativePoseStateMachine &machine, uint64_t now)
{
	machine.update(true, true, true, now, hold_timeout, validity_debounce);
	machine.update(true, true, true, now + validity_debounce, hold_timeout, validity_debounce);
	return now + validity_debounce;
}

TEST(RelativePoseStateMachine, NormalCapture)
{
	RelativePoseStateMachine machine;

	EXPECT_EQ(machine.update(true, true, false, start_time, hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, start_time + 10, hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, start_time + 10 + validity_debounce,
				 hold_timeout, validity_debounce), State::Tracking);
}

TEST(RelativePoseStateMachine, SingleInvalidSampleRecovers)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);

	EXPECT_EQ(machine.update(true, true, false, tracking_time + 10, hold_timeout, validity_debounce), State::Tracking);
	EXPECT_EQ(machine.update(true, true, true, tracking_time + 20, hold_timeout, validity_debounce), State::Tracking);
	EXPECT_EQ(machine.lossDuration(tracking_time + 20), 0u);
}

TEST(RelativePoseStateMachine, ShortLossHolds)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);
	const uint64_t loss_started = tracking_time + 10;
	machine.update(true, true, false, loss_started, hold_timeout, validity_debounce);

	EXPECT_EQ(machine.update(true, true, false, loss_started + validity_debounce,
				 hold_timeout, validity_debounce), State::LossHold);
	EXPECT_EQ(machine.update(true, true, false, loss_started + hold_timeout,
				 hold_timeout, validity_debounce), State::LossHold);
	EXPECT_EQ(machine.lossDuration(loss_started + hold_timeout), hold_timeout);
}

TEST(RelativePoseStateMachine, TimeoutRecoveryRemainsAborted)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);
	const uint64_t loss_started = tracking_time + 10;
	machine.update(true, true, false, loss_started, hold_timeout, validity_debounce);
	machine.update(true, true, false, loss_started + validity_debounce, hold_timeout, validity_debounce);

	EXPECT_EQ(machine.update(true, true, false, loss_started + hold_timeout + 1,
				 hold_timeout, validity_debounce), State::Aborted);
	EXPECT_EQ(machine.update(true, true, true, loss_started + hold_timeout + 20,
				 hold_timeout, validity_debounce), State::Aborted);
}

TEST(RelativePoseStateMachine, LeavingOffboardStartsNewSession)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);
	const uint64_t loss_started = tracking_time + 10;
	machine.update(true, true, false, loss_started, hold_timeout, validity_debounce);
	machine.update(true, true, false, loss_started + validity_debounce, hold_timeout, validity_debounce);
	machine.update(true, true, false, loss_started + hold_timeout + 1, hold_timeout, validity_debounce);

	const uint64_t reentry_time = loss_started + hold_timeout + 20;
	EXPECT_EQ(machine.update(false, true, true, reentry_time, hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, reentry_time + 10, hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, reentry_time + 10 + validity_debounce,
				 hold_timeout, validity_debounce), State::Tracking);
}

TEST(RelativePoseStateMachine, RcHandoffResetsSession)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);

	EXPECT_EQ(machine.update(true, false, true, tracking_time + 10,
				 hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, tracking_time + 20,
				 hold_timeout, validity_debounce), State::Idle);
	EXPECT_EQ(machine.update(true, true, true, tracking_time + 20 + validity_debounce,
				 hold_timeout, validity_debounce), State::Tracking);
}

TEST(RelativePoseStateMachine, RecoveryRequiresStableValidity)
{
	RelativePoseStateMachine machine;
	const uint64_t tracking_time = enterTracking(machine, start_time);
	const uint64_t loss_started = tracking_time + 10;
	machine.update(true, true, false, loss_started, hold_timeout, validity_debounce);
	machine.update(true, true, false, loss_started + validity_debounce, hold_timeout, validity_debounce);

	const uint64_t first_recovery = loss_started + validity_debounce + 10;
	EXPECT_EQ(machine.update(true, true, true, first_recovery,
				 hold_timeout, validity_debounce), State::LossHold);
	EXPECT_EQ(machine.update(true, true, false, first_recovery + 10,
				 hold_timeout, validity_debounce), State::LossHold);

	const uint64_t stable_recovery = first_recovery + 20;
	EXPECT_EQ(machine.update(true, true, true, stable_recovery,
				 hold_timeout, validity_debounce), State::LossHold);
	EXPECT_EQ(machine.update(true, true, true, stable_recovery + validity_debounce,
				 hold_timeout, validity_debounce), State::Tracking);
}
