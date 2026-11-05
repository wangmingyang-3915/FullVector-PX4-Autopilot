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

#include <gtest/gtest.h>
#include "DockingSafety.hpp"
#include "RelativePoseStateMachine.hpp"

using matrix::Quatf;
using matrix::Eulerf;

TEST(DockingSafety, CorrectRelativeRotationAcceptsFixedTargetDuringBodyMotion)
{
	DockingAttitudeGuard guard;
	const Quatf target(Eulerf(0.05f, -0.02f, 1.57f));

	for (int i = 0; i < 150; ++i) {
		const Quatf body(Eulerf(0.03f, -0.04f, i * 0.02f));
		EXPECT_TRUE(guard.accept(body, target.inversed() * body, 0.35f));
	}
}

TEST(DockingSafety, AccumulatingWrongSignFeedbackIsRejectedDespiteSmallFrameSteps)
{
	DockingAttitudeGuard guard;
	RelativePoseStateMachine session;
	ASSERT_TRUE(guard.accept(Quatf(), Quatf(), 0.35f));
	session.update(true, true, true, 1, 1500000, 0);
	bool rejected = false;

	for (int i = 1; i <= 40; ++i) {
		const float yaw = i * 0.01f;
		const bool accepted = guard.accept(Quatf(Eulerf(0, 0, yaw)), Quatf(Eulerf(0, 0, -yaw)), 0.35f);

		if (!accepted) {
			rejected = true;
			EXPECT_EQ(session.update(true, true, false, i * 20000, 1500000, 0),
				  RelativePoseStateMachine::State::LossHold);
			break;
		}
	}

	EXPECT_TRUE(rejected);
	// 无效样本不能刷新基准；恢复正确输入后仍可通过。
	EXPECT_TRUE(guard.accept(Quatf(Eulerf(0, 0, 0.4f)), Quatf(Eulerf(0, 0, 0.4f)), 0.35f));
}

TEST(DockingSafety, QuaternionSignAndEstimatorResetDoNotCauseFalseRejection)
{
	DockingAttitudeGuard guard;
	const Quatf body(Eulerf(0.1f, -0.05f, 1.0f));
	ASSERT_TRUE(guard.accept(body, body, 0.35f));
	EXPECT_TRUE(guard.accept(body, Quatf(-body(0), -body(1), -body(2), -body(3)), 0.35f));
	const Quatf delta(Eulerf(0, 0, 0.8f));
	guard.applyEstimatorReset(delta);
	EXPECT_TRUE(guard.accept(delta * body, body, 0.35f));
	guard.reset();
	EXPECT_TRUE(guard.accept(Quatf(Eulerf(0, 0, 2.5f)), Quatf(), 0.35f));
}

TEST(DockingSafety, YawStepAndReversalRespectBothRateAndAccelerationLimits)
{
	DockingYawLimiter limiter;
	float previous = 0.0f;

	for (int i = 0; i < 300; ++i) {
		const float requested = i < 150 ? 5.9f : -5.9f; // 约 +/-338 deg/s 的日志异常目标。
		const float rate = limiter.update(requested, 0.0f, 0.5f, 1.0f, 0.005f);
		EXPECT_LE(fabsf(rate), 0.5f);
		EXPECT_LE(fabsf(rate - previous), 0.005001f);
		previous = rate;
	}

	EXPECT_NEAR(previous, -0.25f, 1e-5f);
}

TEST(DockingSafety, ReentryAndLimitReductionDoNotRetainOldYawCommand)
{
	DockingYawLimiter limiter;
	EXPECT_NEAR(limiter.update(2.0f, 0.2f, 0.5f, 1.0f, 0.005f), 0.205f, 1e-6f);
	limiter.reset();
	EXPECT_NEAR(limiter.update(2.0f, -0.2f, 0.5f, 1.0f, 0.005f), -0.195f, 1e-6f);
	EXPECT_FLOAT_EQ(limiter.update(2.0f, 0.0f, 0.1f, 1.0f, 0.005f), -0.1f);
	EXPECT_FLOAT_EQ(limiter.update(2.0f, 0.0f, 0.0f, 1.0f, 0.005f), 0.0f);
}
