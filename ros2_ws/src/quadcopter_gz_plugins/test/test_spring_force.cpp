// Unit tests for the paper Eq. 7 spring-damper force used by the leg plugin.

#include <gtest/gtest.h>

#include "quadcopter_gz_plugins/spring_force.hh"

namespace qgz = quadcopter_gz_plugins;

constexpr double kK = 174.0;       // N/m  (k/m = 5.00e3 s^-2, m = 34.8 g)
constexpr double kFc = 0.442;      // N    (f_c/m = 12.7 m/s^2, m = 34.8 g)
constexpr double kLp = 0.0197;     // m    (preload)
constexpr double kVelEps = 0.02;   // m/s  (sign smoothing)

TEST(SpringDamperForce, ZeroAtRestStop)
{
  // At q >= 0 the leg rests on the mechanical stop; no generalized force.
  EXPECT_DOUBLE_EQ(
      qgz::SpringDamperForce(0.0, -1.0, kK, kFc, kLp, kVelEps), 0.0);
  EXPECT_DOUBLE_EQ(
      qgz::SpringDamperForce(0.001, 1.0, kK, kFc, kLp, kVelEps), 0.0);
}

TEST(SpringDamperForce, ElasticTermMatchesPaper)
{
  // F = k * (l_p - q) with q < 0 during compression.
  const double expected = kK * (kLp + 0.02);
  EXPECT_NEAR(
      qgz::SpringDamperForce(-0.02, 0.0, kK, kFc, kLp, kVelEps),
      expected, 1e-9);
}

TEST(SpringDamperForce, CoulombOpposesCompression)
{
  // Compression (q_dot < 0) adds +f_c to the generalized force.
  const double elastic = kK * (kLp + 0.02);
  EXPECT_NEAR(
      qgz::SpringDamperForce(-0.02, -2.0, kK, kFc, kLp, kVelEps),
      elastic + kFc, 1e-6);
}

TEST(SpringDamperForce, CoulombOpposesExtension)
{
  // Extension (q_dot > 0) subtracts f_c from the generalized force.
  const double elastic = kK * (kLp + 0.02);
  EXPECT_NEAR(
      qgz::SpringDamperForce(-0.02, 2.0, kK, kFc, kLp, kVelEps),
      elastic - kFc, 1e-6);
}

TEST(SpringDamperForce, SmoothSignAtZeroVelocity)
{
  // At zero velocity the friction term vanishes smoothly (tanh model).
  const double elastic = kK * (kLp + 0.02);
  EXPECT_NEAR(
      qgz::SpringDamperForce(-0.02, 0.0, kK, kFc, kLp, kVelEps),
      elastic, 1e-9);
}
