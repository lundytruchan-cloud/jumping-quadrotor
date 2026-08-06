"""High-level jumping controller planning (paper Eqs. 22-33).

The planner runs once per hopping cycle and combines:

* ballistic landing-state prediction (Eqs. 22-26 area): free-fall from the
  apex with the foot-height touchdown condition;
* the task-4 stance mapping (landing -> takeoff, Eqs. 12-21) embedded in
  3-D through the coplanar model of the paper;
* the desired-takeoff-velocity / iterative landing-attitude optimisation
  (Eq. 31 in the manuscript; equivalent to the dead-beat landing-position
  scheme in the authors' follow-up work);
* the height-control timing laws ``dt_PA`` / ``dt_PJ`` (Eqs. 27-29 in the
  notes: powered ascent at hover thrust keeps the vertical speed constant,
  followed by a ballistic segment);
* the no-slip constraint (Eq. 32): both the landing and takeoff body axes
  must stay within ``atan(mu)`` of the vertical.

The exact symbol layout of the paywalled manuscript could not be retrieved
(Science Robotics is behind Cloudflare); the equations are reconstructed
from the project paper notes and from the authors' open follow-up paper
(Li et al., "A High-Payload Robotic Hopper Powered by Bidirectional
Thrusters", Eqs. 21-38), which reuses the same hopping-controller design.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from .params import PAPER_PARAMS, StanceParams
from .stance import stance_transform

E3 = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class LandingPrediction:
    """Predicted ballistic touchdown state from an aerial reference state."""

    t_fall: float
    p_landing: np.ndarray
    v_landing: np.ndarray


@dataclass(frozen=True)
class TakeoffPrediction:
    """Stance-map output resolved in 3-D."""

    z_b_to: np.ndarray
    v_to: np.ndarray
    v_t: float
    t_stance: float
    p_to: np.ndarray


@dataclass(frozen=True)
class JumpPlan:
    """Full one-cycle plan produced by :func:`solve_landing_attitude`."""

    z_b_landing: np.ndarray
    z_b_takeoff: np.ndarray
    v_takeoff_des: np.ndarray
    v_takeoff_pred: np.ndarray
    p_landing_k: np.ndarray
    v_landing_k: np.ndarray
    p_landing_next_pred: np.ndarray
    t_fall_current: float
    t_flight_next: float
    cost: float
    success: bool


def _rot_axis(v, n, theta):
    """Rodrigues rotation of ``v`` by ``theta`` about unit axis ``n``."""
    c, s = np.cos(theta), np.sin(theta)
    return (
        c * v
        + s * np.cross(n, v)
        + (1.0 - c) * np.dot(n, v) * n
    )


def _landing_time(z0, vz, z_land, g):
    """Positive root of ``z0 + vz t - 0.5 g t^2 = z_land``."""
    a = 0.5 * g
    b = -vz
    c = z_land - z0
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        raise ValueError(
            "ballistic trajectory never reaches the landing height "
            "(z0={}, vz={}, z_land={})".format(z0, vz, z_land))
    return (-b + np.sqrt(disc)) / (2.0 * a)


def predict_landing_from_apex(p_apex, v_apex, z_b_landing, l0, g):
    """Predict the touchdown state from an apex state (Eqs. 22-23 area).

    The apex state is a free-fall initial condition.  Touchdown occurs when
    the foot (``p - l0 z_b``) reaches the ground plane, i.e. the CoM height
    equals ``l0 (e3 . z_b_landing)``.
    """
    p_apex = np.asarray(p_apex, dtype=float)
    v_apex = np.asarray(v_apex, dtype=float)
    z_b = np.asarray(z_b_landing, dtype=float)
    z_land = l0 * float(np.dot(E3, z_b))
    t = _landing_time(float(p_apex[2]), float(v_apex[2]), z_land, g)
    p_landing = p_apex + v_apex * t - 0.5 * g * t * t * E3
    v_landing = v_apex - g * t * E3
    return LandingPrediction(t_fall=t, p_landing=p_landing, v_landing=v_landing)


def predict_landing_from_takeoff(p_to, v_to, z_b_landing_next, l0, g):
    """Predict the next touchdown state from a takeoff state."""
    p_to = np.asarray(p_to, dtype=float)
    v_to = np.asarray(v_to, dtype=float)
    z_b = np.asarray(z_b_landing_next, dtype=float)
    z_land = l0 * float(np.dot(E3, z_b))
    t = _landing_time(float(p_to[2]), float(v_to[2]), z_land, g)
    p_landing = p_to + v_to * t - 0.5 * g * t * t * E3
    v_landing = v_to - g * t * E3
    return LandingPrediction(t_fall=t, p_landing=p_landing, v_landing=v_landing)


def stance_landing_to_takeoff(p_landing, v_landing, z_b_landing, params=PAPER_PARAMS):
    """Embed the 2-D stance map (Eqs. 12-21) in 3-D.

    The landing/takeoff vectors are coplanar: the plane is spanned by the
    opposite-of-landing-velocity direction ``a`` and the landing body axis
    ``z_b_landing``.  The 2-D map provides the takeoff attitude angle and
    takeoff velocity direction measured from ``a``.
    """
    p_landing = np.asarray(p_landing, dtype=float)
    v_landing = np.asarray(v_landing, dtype=float)
    z_b = np.asarray(z_b_landing, dtype=float)
    speed = float(np.linalg.norm(v_landing))
    z_b = z_b / np.linalg.norm(z_b)
    if speed <= 1e-9:
        return TakeoffPrediction(
            z_b_to=z_b, v_to=np.zeros(3), v_t=0.0,
            t_stance=0.0, p_to=p_landing,
        )
    a = -v_landing / speed
    v_p = -float(np.dot(v_landing, z_b))
    if v_p <= 0.0:
        # No axial compression: the passive leg cannot push off.
        return TakeoffPrediction(
            z_b_to=z_b, v_to=np.zeros(3), v_t=0.0,
            t_stance=0.0, p_to=p_landing,
        )
    v_perp_vec = v_landing - float(np.dot(v_landing, z_b)) * z_b
    v_perp = float(np.linalg.norm(v_perp_vec))
    theta_l = np.arctan2(v_perp, v_p)
    res = stance_transform(theta_l, speed, params)
    n = np.cross(a, z_b)
    if np.linalg.norm(n) < 1e-12:
        n = np.array([1.0, 0.0, 0.0])
    else:
        n = n / np.linalg.norm(n)
    z_b_to = _rot_axis(a, n, res.theta_t)
    v_to_dir = _rot_axis(a, n, res.theta_v)
    v_to = res.v_t * v_to_dir
    p_to = p_landing - params.l0 * z_b + params.l0 * z_b_to
    return TakeoffPrediction(
        z_b_to=z_b_to, v_to=v_to, v_t=res.v_t,
        t_stance=res.t_TO, p_to=p_to,
    )


def desired_takeoff_velocity(p_landing_k, p_des_next, z_d, g):
    """Dead-beat takeoff velocity for a target apex height (Eqs. 24-26 area).

    Vertical component from the apex-height energy balance
    (``e3.v = sqrt(2 g z_d)``), horizontal components from the flat-ground
    flight time ``2 e3.v / g``.
    """
    p_landing_k = np.asarray(p_landing_k, dtype=float)
    p_des_next = np.asarray(p_des_next, dtype=float)
    v_z = np.sqrt(2.0 * g * z_d)
    v_h = g * (p_des_next - p_landing_k) / (2.0 * v_z)
    v_h[2] = 0.0
    return v_h + v_z * E3


def powered_ascent_duration(vz, z_d, g):
    """Powered-ascent time ``dt_PA`` (Eqs. 27-28 area).

    With hover-level thrust the vertical speed is kept constant, so the
    height gained during the burst is ``vz dt_PA``; the ballistic part
    contributes ``vz^2/(2g)``.  Solving ``z_d = vz dt_PA + vz^2/(2g)``
    gives ``dt_PA = z_d/vz - vz/(2g)``; zero if the nominal height
    ``vz^2/(2g)`` already meets the target.
    """
    if vz <= 0.0 or z_d <= 0.0:
        return 0.0
    if 0.5 * vz * vz / g >= z_d:
        return 0.0
    return z_d / vz - vz / (2.0 * g)


def ballistic_flight_duration(vz, z_d, g):
    """Ballistic segment duration ``dt_PJ`` (Eq. 29 area).

    Time from the end of the powered ascent to touchdown: time to the apex
    ``vz/g`` plus the fall from the apex height ``z_d``,
    ``sqrt(2 z_d/g)``.
    """
    if z_d < 0.0:
        raise ValueError("desired apex height must be non-negative")
    return vz / g + np.sqrt(2.0 * z_d / g)


def no_slip_angle(z_b):
    """Tilt angle of the body axis from the vertical, rad."""
    z_b = np.asarray(z_b, dtype=float)
    return float(np.arccos(np.clip(np.dot(E3, z_b) / np.linalg.norm(z_b), -1.0, 1.0)))


def no_slip_ok(z_b, mu):
    """No-slip condition of Eq. 32: tilt below ``atan(mu)``."""
    return no_slip_angle(z_b) <= np.arctan(mu) + 1e-9


def _forward_cost(angles, p_apex, v_apex, p_des_next, z_d, params, g,
                  mu, w_position, w_velocity, w_slip):
    alpha, phi = angles
    alpha_max = np.arctan(mu)
    alpha = float(np.clip(alpha, 0.0, alpha_max))
    z_b = np.array([
        np.sin(alpha) * np.cos(phi),
        np.sin(alpha) * np.sin(phi),
        np.cos(alpha),
    ])
    try:
        pred_k = predict_landing_from_apex(p_apex, v_apex, z_b, params.l0, g)
    except ValueError:
        return 1e6
    stance = stance_landing_to_takeoff(pred_k.p_landing, pred_k.v_landing, z_b, params)
    if stance.v_t <= 0.0:
        return 1e6
    if stance.v_to[2] <= 0.0:
        # A downward takeoff cannot start a usable jump; steer away from it.
        return 10.0 * (1.0 + stance.v_to[2] * stance.v_to[2])
    try:
        pred_next = predict_landing_from_takeoff(
            stance.p_to, stance.v_to, z_b, params.l0, g)
    except ValueError:
        return 1e6
    v_des = desired_takeoff_velocity(pred_k.p_landing, p_des_next, z_d, g)
    v_des_dir = v_des / np.linalg.norm(v_des)
    v_to_dir = stance.v_to / np.linalg.norm(stance.v_to)
    # Landing references are 2-D (x, y); the CoM touchdown height is set by
    # the attitude through ``l0 cos(alpha)`` and is not a tracking error.
    pos_err = np.linalg.norm((pred_next.p_landing - p_des_next)[:2])
    dir_err = np.linalg.norm(v_to_dir - v_des_dir)
    slip = max(0.0, no_slip_angle(stance.z_b_to) - np.arctan(mu))
    return (
        w_position * pos_err * pos_err
        + w_velocity * dir_err * dir_err
        + w_slip * slip * slip
    )


def solve_landing_attitude(p_apex, v_apex, p_des_next, z_d,
                           params=PAPER_PARAMS, g=9.80665, mu=0.8,
                           w_position=1.0, w_velocity=0.5, w_slip=10.0,
                           polish_rounds=3, tilt_limit=None):
    """Solve for the desired landing attitude ``z_b(t_LD)`` (Eq. 31 area).

    The attitude is parameterised by its tilt from the vertical ``alpha``
    and azimuth ``phi``.  ``alpha`` is bounded by the no-slip limit; the
    takeoff attitude no-slip violation is added as a quadratic penalty.
    A few restart/polish rounds mimic the paper's iterative algebraic-loop
    solution.
    """
    p_apex = np.asarray(p_apex, dtype=float)
    v_apex = np.asarray(v_apex, dtype=float)
    p_des_next = np.asarray(p_des_next, dtype=float)
    alpha_max = min(
        float(np.arctan(mu)),
        float(tilt_limit) if tilt_limit is not None else float('inf'),
    )

    delta = p_des_next - p_apex
    phi_target = float(np.arctan2(delta[1], delta[0]))
    phi_v = float(np.arctan2(v_apex[1], v_apex[0]))
    starts = [
        np.array([0.0, 0.0]),
        np.array([0.6 * alpha_max, phi_target]),
        np.array([0.8 * alpha_max, phi_target]),
        np.array([0.6 * alpha_max, phi_v]),
    ]

    def cost(angles):
        return _forward_cost(
            angles, p_apex, v_apex, p_des_next, z_d, params, g,
            mu, w_position, w_velocity, w_slip)

    best = None
    for x0 in starts:
        res = minimize(
            cost, x0, method="SLSQP",
            bounds=[(0.0, alpha_max), (-np.pi, np.pi)],
            options={"maxiter": 200, "ftol": 1e-12},
        )
        if best is None or res.fun < best.fun:
            best = res

    if best is None or not best.success:
        # Fall back to the best objective value among the starts.
        vals = [cost(x) for x in starts]
        best = minimize(cost, starts[int(np.argmin(vals))], method="Nelder-Mead",
                        options={"maxiter": 200})

    x = np.clip(best.x[0], 0.0, alpha_max)
    z_b = np.array([
        np.sin(x) * np.cos(best.x[1]),
        np.sin(x) * np.sin(best.x[1]),
        np.cos(x),
    ])
    # Polish rounds: restart from the current optimum to converge the
    # algebraic loop (landing time <-> stance <-> next landing).
    x_current = np.array([x, best.x[1]])
    for _ in range(polish_rounds):
        res = minimize(
            cost, x_current, method="SLSQP",
            bounds=[(0.0, alpha_max), (-np.pi, np.pi)],
            options={"maxiter": 200, "ftol": 1e-12},
        )
        if res.fun <= best.fun:
            best = res
            x_current = np.clip(best.x[0], 0.0, alpha_max), best.x[1]
        else:
            break

    x = float(np.clip(best.x[0], 0.0, alpha_max))
    z_b = np.array([
        np.sin(x) * np.cos(best.x[1]),
        np.sin(x) * np.sin(best.x[1]),
        np.cos(x),
    ])
    pred_k = predict_landing_from_apex(p_apex, v_apex, z_b, params.l0, g)
    stance = stance_landing_to_takeoff(pred_k.p_landing, pred_k.v_landing, z_b, params)
    if stance.v_t <= 0.0:
        pred_next = LandingPrediction(
            t_fall=0.0, p_landing=pred_k.p_landing.copy(),
            v_landing=pred_k.v_landing.copy())
    else:
        pred_next = predict_landing_from_takeoff(
            stance.p_to, stance.v_to, z_b, params.l0, g)
    v_des = desired_takeoff_velocity(pred_k.p_landing, p_des_next, z_d, g)
    success = bool(best.success) and stance.v_t > 0.0
    return JumpPlan(
        z_b_landing=z_b,
        z_b_takeoff=stance.z_b_to,
        v_takeoff_des=v_des,
        v_takeoff_pred=stance.v_to,
        p_landing_k=pred_k.p_landing,
        v_landing_k=pred_k.v_landing,
        p_landing_next_pred=pred_next.p_landing,
        t_fall_current=pred_k.t_fall,
        t_flight_next=pred_next.t_fall,
        cost=float(best.fun),
        success=success,
    )


def circle_reference(t, radius, omega, phase0=0.0, center=(0.0, 0.0)):
    """Circle landing reference: ``(R cos(w t + p0), R sin(...), 0)``."""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    angle = omega * t + phase0
    out = np.zeros((t.size, 3))
    out[:, 0] = center[0] + radius * np.cos(angle)
    out[:, 1] = center[1] + radius * np.sin(angle)
    return out


def step_reference(t, targets, hold_period):
    """Step landing reference: hold each target for ``hold_period`` seconds."""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    idx = (np.floor(t / hold_period).astype(int)) % len(targets)
    out = np.zeros((t.size, 3))
    for i, j in enumerate(idx):
        out[i, 0] = targets[j][0]
        out[i, 1] = targets[j][1]
    return out
