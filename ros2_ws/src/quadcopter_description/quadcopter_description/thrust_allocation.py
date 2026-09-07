# Copyright 2026 Larry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Attitude-priority thrust allocation (paper Eqs. 34-35).

Given desired body torque tau_d, find motor forces f_i >= 0 satisfying the
torque equations exactly and making the total thrust T as close as possible
to the desired thrust T_d.  For a standard quad the torque null space is the
uniform-thrust direction, so the problem reduces to

    T_alloc = max(T_d, T_min(tau_d)),
    f_i     = T_alloc/4 + (M * tau_d)_i,

where T_min is the smallest non-negative total thrust that keeps f_i >= 0.
"""

import math


class ThrustAllocation:
    """Closed-form attitude-priority allocator for the four rotors."""

    def __init__(self, arm=0.032527, moment_ratio=0.006):
        self.arm = arm
        self.moment_ratio = moment_ratio
        self._build_mixing()

    def _build_mixing(self):
        """Zero-sum torque mixing matrix M (4x3) with A_t * M = I."""
        # A_t rows: [tau_x, tau_y, tau_z] coefficients for f0..f3.
        # Rotor positions (+-a, +-a); yaw moments alternate sign -c*fi
        # (a CCW rotor pushes the body clockwise, verified in Gazebo) so that
        # roll and yaw remain independent (standard X4 layout).
        a = self.arm
        c = self.moment_ratio
        self._mixing = [
            (1.0 / (4.0 * a), -1.0 / (4.0 * a), -1.0 / (4.0 * c)),
            (1.0 / (4.0 * a), 1.0 / (4.0 * a), 1.0 / (4.0 * c)),
            (-1.0 / (4.0 * a), 1.0 / (4.0 * a), -1.0 / (4.0 * c)),
            (-1.0 / (4.0 * a), -1.0 / (4.0 * a), 1.0 / (4.0 * c)),
        ]

    def allocate(self, thrust_desired, torque):
        """Return f0..f3 (N), actual thrust and the minimal feasible thrust."""
        tau_x, tau_y, tau_z = torque
        f_base = [
            m0 * tau_x + m1 * tau_y + m2 * tau_z
            for m0, m1, m2 in self._mixing
        ]
        t_min = max(0.0, -4.0 * min(f_base))
        t_alloc = max(thrust_desired, t_min)
        forces = tuple(t_alloc / 4.0 + fb for fb in f_base)
        return {
            'f': forces,
            'thrust': t_alloc,
            'min_thrust': t_min,
            'torque': tuple(torque),
            'feasible': True,
        }

    @staticmethod
    def speeds_from_thrust(forces, motor_constant=3.0e-8, max_speed=2200.0):
        """Convert per-motor thrust (N) to clamped rotor speeds (rad/s)."""
        speeds = []
        for force in forces:
            if force <= 0.0:
                speeds.append(0.0)
            else:
                speed = math.sqrt(force / motor_constant)
                speeds.append(min(speed, max_speed))
        return speeds
