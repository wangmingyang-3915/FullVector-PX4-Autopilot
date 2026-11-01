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
#include "PosctlAltitudeHold.hpp"

TEST(PosctlAltitudeHold, CenterCapturesOnceAndAllowsPositionCorrection)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -3.0f);
	ASSERT_TRUE(hold.update(true, 0.05f, 0.1f, -3.2f, 4, 0.0f, {-3.2f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -3.0f);
	ASSERT_TRUE(hold.update(true, -0.1f, 0.1f, -2.8f, 4, 0.0f, {-2.8f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -3.0f);
}

TEST(PosctlAltitudeHold, MovingStickReleasesAndRecenteringCapturesNewHeight)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.11f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, -0.11f, 0.1f, -4.0f, 4, 0.0f, {-4.0f, 0.0f, 0.0f}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -5.0f, 4, 0.0f, {-5.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -5.0f);
}

TEST(PosctlAltitudeHold, DisabledOrInvalidInputReleasesOldTarget)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(false, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -4.0f, 4, 0.0f, {-4.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -4.0f);
	EXPECT_FALSE(hold.update(true, NAN, 0.1f, -4.0f, 4, 0.0f, {-4.0f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, NAN, 4, 0.0f, {NAN, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.0f, NAN, -4.0f, 4, 0.0f, {-4.0f, 0.0f, 0.0f}));
}

TEST(PosctlAltitudeHold, EstimatorResetPreservesHeightErrorAndAppliesDeltaOnce)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	// Before reset z=-2.8, target=-3, error=-0.2; delta=-4.2 shifts both.
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -7.0f, 5, -4.2f, {-7.0f, 0.0f, 0.0f}));
	EXPECT_NEAR(hold.position() - (-7.0f), -0.2f, 1e-6f);
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -7.1f, 5, -4.2f, {-7.1f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -7.2f);
}

TEST(PosctlAltitudeHold, ResetCounterWrapAndMissedResets)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 255, 0.0f, {-3.0f, 0.0f, 0.0f}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -4.0f, 0, -1.0f, {-4.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -4.0f);
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -6.0f, 2, -1.0f, {-6.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -6.0f);
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -7.0f, 3, NAN, {-7.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -7.0f);
}

TEST(PosctlAltitudeHold, ControllerResetRecapturesHeight)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, 0.0f}));
	hold.reset();
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -5.0f, 4, 0.0f, {-5.0f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -5.0f);
}

TEST(PosctlAltitudeHold, ClimbAndDescentBrakeBeforeHoldingTrajectoryEndpoint)
{
	for (float direction : {-1.0f, 1.0f}) {
		PosctlAltitudeHold hold;
		// 回中时仍在运动；实测高度为 -3，制动期间不能立即锁定此高度。
		EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f,
		{NAN, direction, -direction}));
		// 有限位置目标不表示制动完成，仍需等待速度和加速度接近零。
		EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.2f, 4, 0.0f,
		{-3.4f, 0.05f * direction, -0.1f * direction}));
		EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.3f, 4, 0.0f,
		{-3.4f, 0.0f, -0.05f * direction}));
		ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.35f, 4, 0.0f, {-3.4f, 0.0f, 0.0f}));
		EXPECT_FLOAT_EQ(hold.position(), -3.4f);
	}
}

TEST(PosctlAltitudeHold, ReleasingHoldRequiresNewBrakingTrajectoryToFinish)
{
	PosctlAltitudeHold hold;
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.1f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.3f, 0.1f, -3.0f, 4, 0.0f, {NAN, -1.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -4.0f, 4, 0.0f, {NAN, -0.4f, 0.2f}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -4.4f, 4, 0.0f, {-4.5f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -4.5f);
}

TEST(PosctlAltitudeHold, InvalidTrajectoryCannotTriggerOrRetainHold)
{
	PosctlAltitudeHold hold;
	EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, NAN, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.0f, 0.0f, NAN}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.1f, 0.0f, 0.0f}));
	EXPECT_FALSE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {NAN, 0.0f, 0.0f}));
	ASSERT_TRUE(hold.update(true, 0.0f, 0.1f, -3.0f, 4, 0.0f, {-3.2f, 0.0f, 0.0f}));
	EXPECT_FLOAT_EQ(hold.position(), -3.2f);
}
