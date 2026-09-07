// Copyright 2026 Larry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include <gtest/gtest.h>
#include <gz/math/Vector3.hh>

#include "quadcopter_gz_plugins/aero_stabilizer.hh"

namespace quadcopter_gz_plugins
{

constexpr double kRho = 1.2;
constexpr double kSurface = 2.0 * 39.0e-4;
constexpr double kMomentArm = 0.06;

TEST(AeroStabilizer, AlignsBodyAxisWithOppositeVelocity)
{
  const gz::math::Vector3d v(1.0, 0.0, -3.0);
  const double tilt = 10.0 * M_PI / 180.0;
  const gz::math::Vector3d z(std::sin(tilt), 0.0, std::cos(tilt));
  const auto out = AeroStabilizerWrench(z, v, kRho, kSurface, 1.0,
                                        kMomentArm);
  EXPECT_GT(out.torque.Length(), 0.0);
  // z x a (a = -v_hat) points along -y for this configuration, so the
  // torque rotates the tilted body axis back toward -v_hat.
  EXPECT_LT(out.torque.Y(), 0.0);
}

TEST(AeroStabilizer, ZeroWhenAligned)
{
  const gz::math::Vector3d v(1.0, 0.0, -3.0);
  const gz::math::Vector3d z = -v.Normalized();
  const auto out = AeroStabilizerWrench(z, v, kRho, kSurface, 1.0,
                                        kMomentArm);
  EXPECT_NEAR(out.torque.Length(), 0.0, 1e-12);
}

TEST(AeroStabilizer, ZeroWhenStillOrAntiparallel)
{
  const gz::math::Vector3d v(1.0, 0.0, -3.0);
  const gz::math::Vector3d z = v.Normalized();
  const auto out1 = AeroStabilizerWrench(z, v, kRho, kSurface, 1.0,
                                         kMomentArm);
  EXPECT_NEAR(out1.torque.Length(), 0.0, 1e-12);

  const auto out2 = AeroStabilizerWrench(
      gz::math::Vector3d::UnitZ, gz::math::Vector3d::Zero,
      kRho, kSurface, 1.0, kMomentArm);
  EXPECT_NEAR(out2.torque.Length(), 0.0, 1e-12);
}

TEST(AeroStabilizer, FlatPlateMagnitude)
{
  const gz::math::Vector3d v(0.0, 0.0, -4.4);
  const double tilt = 10.0 * M_PI / 180.0;
  const gz::math::Vector3d z(std::sin(tilt), 0.0, std::cos(tilt));
  const auto out = AeroStabilizerWrench(z, v, kRho, kSurface, 1.0,
                                        kMomentArm);
  const double expectedForce = kRho * kSurface * std::sin(tilt) * 4.4 * 4.4;
  EXPECT_NEAR(out.forceMag, expectedForce, 1e-9);
  EXPECT_NEAR(out.torque.Length(), kMomentArm * expectedForce, 1e-9);
  EXPECT_GT(out.forceMag, 5.0e-3);
  EXPECT_LT(out.forceMag, 0.1);
}

TEST(AeroStabilizer, ScalesWithSpeedSquared)
{
  const double tilt = 0.3;
  const gz::math::Vector3d z(std::sin(tilt), 0.0, std::cos(tilt));
  const auto t1 = AeroStabilizerWrench(
      z, gz::math::Vector3d(0.0, 0.0, -3.0),
      kRho, kSurface, 1.0, kMomentArm);
  const auto t2 = AeroStabilizerWrench(
      z, gz::math::Vector3d(0.0, 0.0, -6.0),
      kRho, kSurface, 1.0, kMomentArm);
  EXPECT_NEAR(t2.torque.Length(), 4.0 * t1.torque.Length(), 1e-9);
}

}  // namespace quadcopter_gz_plugins
