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

#ifndef QUADCOPTER_GZ_PLUGINS_SPRING_FORCE_HH_
#define QUADCOPTER_GZ_PLUGINS_SPRING_FORCE_HH_

#include <cmath>

namespace quadcopter_gz_plugins
{
/// \brief Passive telescopic-leg force along the prismatic joint (paper Eq. 7).
///
/// Joint convention: q = 0 at the rest length l0 (mechanical stop),
/// q < 0 during compression.  The elastic element is pre-stretched by l_p,
/// so the generalized force is
///   F = k * (l_p - q) - f_c * sgn(q_dot)
/// while q < 0, and zero at/above the stop (q >= 0).
/// The sign function is smoothed with tanh to avoid Coulomb chatter.
///
/// \param[in] q Joint position along the leg axis (m).
/// \param[in] q_dot Joint velocity (m/s).
/// \param[in] k Spring stiffness (N/m).
/// \param[in] f_c Coulomb friction magnitude (N).
/// \param[in] l_p Preload (m).
/// \param[in] vel_eps Velocity smoothing scale (m/s).
/// \return Generalized force (N), positive extends the leg.
inline double SpringDamperForce(
    double q, double q_dot, double k, double f_c, double l_p, double vel_eps)
{
  if (q >= 0.0)
    return 0.0;
  const double eps = vel_eps > 0.0 ? vel_eps : 1e-3;
  return k * (l_p - q) - f_c * std::tanh(q_dot / eps);
}
}  // namespace quadcopter_gz_plugins

#endif  // QUADCOPTER_GZ_PLUGINS_SPRING_FORCE_HH_
