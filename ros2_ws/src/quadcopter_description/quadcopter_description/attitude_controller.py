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

"""Cascaded attitude PID: attitude P -> desired rate -> rate PID -> torque."""

from .attitude_math import attitude_error


class AttitudePidController:
    """
    Attitude setpoint tracking for a rigid quadcopter body.

    The attitude loop maps the body-frame rotation error to a desired angular
    rate; the rate loop maps the rate error to body-frame torque with optional
    integral action and anti-windup.
    """

    def __init__(self, kp_att=15.0, kp_rate=1.2e-3, ki_rate=1.0e-3,
                 kd_rate=0.0, integral_limit=1.0e-2, max_torque=0.05):
        self.kp_att = kp_att
        self.kp_rate = kp_rate
        self.ki_rate = ki_rate
        self.kd_rate = kd_rate
        self.integral_limit = integral_limit
        self.max_torque = max_torque
        self.reset()

    def reset(self):
        self.integral = [0.0, 0.0, 0.0]
        self.last_attitude_error = (0.0, 0.0, 0.0)
        self.last_rate_error = (0.0, 0.0, 0.0)
        self.last_torque = (0.0, 0.0, 0.0)

    def update(self, q_ref, q_cur, omega_body, dt):
        """
        Compute body-frame torque for one control tick.

        q_ref/q_cur: (w, x, y, z) unit quaternions.
        omega_body: body-frame angular velocity (rad/s).
        dt: control period (s).
        Returns (tau_x, tau_y, tau_z) in N*m.
        """
        e_rot = attitude_error(q_ref, q_cur)
        self.last_attitude_error = e_rot

        omega_des = [self.kp_att * e for e in e_rot]
        rate_error = [
            omega_des[i] - omega_body[i] for i in range(3)]

        if self.ki_rate > 0.0:
            for i in range(3):
                self.integral[i] += rate_error[i] * dt
                limit = self.integral_limit
                self.integral[i] = max(-limit, min(limit, self.integral[i]))

        derivative = [0.0, 0.0, 0.0]
        if self.kd_rate > 0.0 and dt > 0.0:
            derivative = [
                (rate_error[i] - self.last_rate_error[i]) / dt
                for i in range(3)
            ]
        self.last_rate_error = rate_error

        torque = [
            self.kp_rate * rate_error[i]
            + self.ki_rate * self.integral[i]
            + self.kd_rate * derivative[i]
            for i in range(3)
        ]
        torque = [
            max(-self.max_torque, min(self.max_torque, t))
            for t in torque
        ]
        self.last_torque = tuple(torque)
        return self.last_torque
