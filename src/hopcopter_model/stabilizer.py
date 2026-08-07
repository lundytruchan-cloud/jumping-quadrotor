"""Aerodynamic stabilizer model and activation logic (task 8).

The stabilizer is a 4.9 g module with three horizontally hinged 39 cm^2
surfaces (paper Fig. 6A).  Tightening the servo cables makes the surfaces
rigid (active) so that the upward airflow generates an aligning torque;
loosening them (inactive) lets the surfaces droop by up to 15 deg so that
the aerodynamic force/torque becomes negligible during the ascent.

Force model (paper Supplementary Methods, Eq. S33, flat-plate theory):

    F_A = rho * S * sin(theta_LD) * U^2

with rho = 1.2 kg/m^3, S = 2 * 39 cm^2 effective area, U the relative
airspeed and theta_LD the angle between the body axis z_b and the opposite
of the velocity (-v_hat).  The force acts at the centre of pressure whose
arm from the CoM is 1-2 times the propeller-arm length; only the restoring
torque component is applied to the body here (the 10 mN force itself is
negligible against the ~340 mN weight).
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AeroStabilizerParams:
    """Flat-plate aerodynamic stabilizer parameters (SI units)."""

    rho: float = 1.2
    s_eff: float = 2.0 * 39.0e-4
    k_eff: float = 1.0
    moment_arm: float = 0.06
    speed_eps: float = 0.05
    sin_theta_eps: float = 1e-9


#: Default parameters matching the paper (Fig. 6A / SM Eq. S33).
AERO_PARAMS = AeroStabilizerParams()


@dataclass(frozen=True)
class AeroTorqueResult:
    """Aerodynamic stabilizer wrench at one instant."""

    force_mag: float
    torque: np.ndarray
    torque_norm: float
    theta_ld: float
    speed: float


def aero_aligning_torque(z_b, v, params=AERO_PARAMS):
    """Aligning torque steering the body axis toward the opposite velocity.

    Args:
        z_b: body z axis in the world frame.
        v: body translational velocity in the world frame (m/s).
        params: :class:`AeroStabilizerParams`.

    Returns:
        :class:`AeroTorqueResult` with the torque
        ``tau = moment_arm * F_A * (z_b x (-v_hat)) / sin(theta_LD)``.
    """
    z_b = np.asarray(z_b, dtype=float)
    v = np.asarray(v, dtype=float)
    z_norm = float(np.linalg.norm(z_b))
    speed = float(np.linalg.norm(v))
    zero = AeroTorqueResult(
        force_mag=0.0, torque=np.zeros(3), torque_norm=0.0,
        theta_ld=0.0, speed=speed)
    if z_norm < 1e-12 or speed < params.speed_eps:
        return zero

    z = z_b / z_norm
    a = -v / speed
    sin_theta = float(np.linalg.norm(np.cross(z, a)))
    sin_theta = min(1.0, max(0.0, sin_theta))
    theta_ld = float(np.arctan2(sin_theta, float(np.dot(z, a))))
    if sin_theta <= params.sin_theta_eps:
        return AeroTorqueResult(
            force_mag=0.0, torque=np.zeros(3), torque_norm=0.0,
            theta_ld=theta_ld, speed=speed)

    # Torque axis z x a (with a = -v_hat).  The rotational dynamics
    # z_dot = omega x z, omega ~ tau, give z_dot ~ tau x z = (z x a) x z,
    # whose perpendicular component points toward a: the body axis aligns
    # with the opposite of the translational velocity (zb -> -v_hat).
    n = np.cross(z, a) / sin_theta
    force_mag = params.k_eff * params.rho * params.s_eff * sin_theta * speed ** 2
    torque = params.moment_arm * force_mag * n
    return AeroTorqueResult(
        force_mag=force_mag,
        torque=torque,
        torque_norm=float(np.linalg.norm(torque)),
        theta_ld=theta_ld,
        speed=speed,
    )


def stabilizer_active(strategy, high_state):
    """Whether the stabilizer cables should be tightened at this state.

    ``attitude_only`` never activates the stabilizer; ``stabilizer_only``
    keeps it always active (its torque matters only while falling);
    ``combined`` activates it only during the descent so the ascent
    dynamics stay unaffected (paper "Damper-mediated stability").
    """
    if strategy == "attitude_only":
        return False
    if strategy == "stabilizer_only":
        return True
    return high_state in ("DESCENT", "PRE_LAND", "STANCE")


def descent_control_mode(strategy, high_state):
    """Low-level attitude-control mode for each high-level state.

    With ``stabilizer_only`` the attitude controller is active only while
    ascending and the motors/attitude torque are switched off during the
    descent (paper: "the attitude controller was only active when the
    robot is ascending").
    """
    if high_state == "STANCE":
        # The stance phase is fully passive in every strategy.
        return "zero_thrust"
    if high_state in ("DESCENT", "PRE_LAND"):
        if strategy == "stabilizer_only":
            return "zero_thrust"
        return "attitude_only"
    if high_state == "ASCENT_POWERED":
        return "hover"
    if high_state == "ASCENT_BALLISTIC":
        return "attitude_only"
    return "zero_thrust"
