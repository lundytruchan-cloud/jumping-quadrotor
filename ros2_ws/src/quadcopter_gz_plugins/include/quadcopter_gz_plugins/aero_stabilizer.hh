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

#ifndef QUADCOPTER_GZ_PLUGINS__AERO_STABILIZER_HH_
#define QUADCOPTER_GZ_PLUGINS__AERO_STABILIZER_HH_

#include <gz/math/Vector3.hh>

namespace quadcopter_gz_plugins
{

/// \brief Flat-plate aerodynamic stabilizer wrench (paper SM Eq. S33).
///
/// The stabilizer consists of three horizontally hinged 39 cm^2 surfaces.
/// When the servo cables are tightened (active), the upward airflow
/// produces an aligning torque steering the body axis z_b toward the
/// opposite of the translational velocity: z_b -> -v_hat.  The force
/// magnitude follows the flat-plate theory
///
///   F_A = rho * S * sin(theta_LD) * U^2
///
/// with rho the air density, S the effective area (2 * 39 cm^2), theta_LD
/// the angle between z_b and -v_hat and U the relative airspeed.  The
/// restoring torque is the force times the effective moment arm about the
/// CoM; the 10 mN force itself is negligible against the ~340 mN weight,
/// so only the torque is applied to the body.
struct AeroWrench
{
  /// Force applied at the centre of pressure (kept zero in this model).
  gz::math::Vector3d force = gz::math::Vector3d::Zero;
  /// Aligning torque about the CoM.
  gz::math::Vector3d torque = gz::math::Vector3d::Zero;
  /// Angle between z_b and -v_hat, rad.
  double thetaLd = 0.0;
  /// Relative airspeed, m/s.
  double speed = 0.0;
  /// Flat-plate force magnitude, N.
  double forceMag = 0.0;
};

/// \brief Compute the aerodynamic stabilizer wrench.
/// \param[in] zBody Body z axis in the world frame.
/// \param[in] velocity Body translational velocity in the world frame.
/// \param[in] rho Air density (kg/m^3).
/// \param[in] surface Effective aerodynamic area (m^2).
/// \param[in] kEff Dimensionless effectiveness gain.
/// \param[in] momentArm Effective moment arm CoM -> centre of pressure (m).
/// \param[in] speedEps Minimum airspeed before the torque engages (m/s).
AeroWrench AeroStabilizerWrench(
    const gz::math::Vector3d &zBody,
    const gz::math::Vector3d &velocity,
    double rho,
    double surface,
    double kEff,
    double momentArm,
    double speedEps = 0.05);

}  // namespace quadcopter_gz_plugins

#endif  // QUADCOPTER_GZ_PLUGINS__AERO_STABILIZER_HH_
