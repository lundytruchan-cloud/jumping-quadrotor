"""Stance-phase dynamics and landing-to-takeoff mapping (paper Eqs. 12-21).

Model summary (all vectors resolved in the vertical plane containing the
landing velocity; subscripts ``LD``/``TO`` denote landing and takeoff):

* Eq. 12: axial leg dynamics with spring and Coulomb friction
    ``m * ldd = -k * (l - l0 - l_p) - f_c * sgn(ld)``.
  With the landing condition ``l(0) = l0``, ``ld(0) = -v_p`` the compression
  phase (``ld < 0``) has the closed-form solution
    ``l_d(t) = C_d - (v_p/omega) sin(omega t) - (C_d - l0) cos(omega t)``
  with ``omega = sqrt(k/m)`` and ``C_d = l0 + l_p + f_c/k`` (per-mass form:
  ``f_c/k = (f_c/m)/(k/m)``). It ends at ``t_1`` where ``ld = 0``.
* The extension phase (``ld > 0``) starts from ``(t_1, l_d(t_1))`` with zero
  axial velocity and equilibrium ``C_u = l0 + l_p - f_c/k``:
    ``l_u(t) = C_u + (l_d(t_1) - C_u) cos(omega (t - t_1))``.
* Takeoff occurs when the leg returns to its rest length, ``l_u(t_TO) = l0``.
* Eqs. 13-14: angular momentum about the fixed foot point is conserved,
    ``beta_dot(t) = l0 * v_perp / l(t)^2``,
  where ``v_perp = |p_dot_LD| sin(theta_LD)`` is the tangential landing speed.
* Eqs. 15-21: the mapping
    ``theta_t = theta_LD + int_0^{t_TO} beta_dot dt``
    ``theta_v = theta_t + atan2(v_perp, ld_u(t_TO))``
    ``v_t = sqrt(v_perp^2 + ld_u(t_TO)^2)``.

This mirrors ``stance_transform`` in the authors' ``run_this.m``; the
piecewise analytic solution is equivalent to the symbolic solution stored in
``eqns_l.mat`` (checked by continuity and by the takeoff condition).
"""

from dataclasses import dataclass

import numpy as np
from scipy.integrate import quad

from .params import PAPER_PARAMS, StanceParams

# Fixed-order Gauss-Legendre rule used for the vectorised grid evaluation.
# The integrand 1/l(t)^2 is smooth over the stance interval, so a moderate
# node count already gives machine-precision accuracy (verified in tests).
_GL_NODES, _GL_WEIGHTS = np.polynomial.legendre.leggauss(128)


@dataclass(frozen=True)
class StanceResult:
    """Output of :func:`stance_transform`.

    Attributes:
        theta_t: body-axis (leg) orientation angle at takeoff, rad.
        theta_v: takeoff velocity direction angle, rad (measured from the
            same reference as ``theta_LD``, i.e. the opposite of the landing
            velocity direction).
        v_t: takeoff speed, m/s.
        t_1: end of the compression phase, s.
        t_TO: takeoff time, s.
        l_t1: leg length at maximum compression, m.
        l_dot_TO: axial extension speed at takeoff, m/s.
        beta_TO: accumulated body rotation during stance, rad.
    """

    theta_t: float
    theta_v: float
    v_t: float
    t_1: float
    t_TO: float
    l_t1: float
    l_dot_TO: float
    beta_TO: float


def _compression_end(theta_l, p_dot_l, p):
    """Closed-form solution of Eq. 12 for the compression phase."""
    omega = p.omega
    v_p = p_dot_l * np.cos(theta_l)
    if v_p <= 0.0:
        # No axial impact: no compression phase.
        return 0.0, p.l0, 0.0
    t_1 = np.arctan2(v_p, omega * (p.c_down - p.l0)) / omega
    l_t1 = p.c_down - (v_p / omega) * np.sin(omega * t_1) - (p.c_down - p.l0) * np.cos(omega * t_1)
    return t_1, l_t1, v_p


def _takeoff_time(l_t1, p):
    """Solve l_u(t_TO) = l0 for the extension phase."""
    amplitude = l_t1 - p.c_up  # negative: l_t1 is below the extension equilibrium
    if amplitude >= 0.0:
        raise ValueError("leg length at maximum compression is not below the extension equilibrium")
    ratio = (p.l0 - p.c_up) / amplitude
    ratio = np.clip(ratio, -1.0, 1.0)
    return np.arccos(ratio) / p.omega


def _integrate_stance(t_1, tau_2, l_t1, v_perp, p, theta_l, p_dot_l):
    omega = p.omega
    v_p = p_dot_l * np.cos(theta_l)
    c_d = p.c_down
    amp_cos_d = -(c_d - p.l0)
    amp_sin_d = -v_p / omega

    def integrand_d(t):
        l = c_d + amp_cos_d * np.cos(omega * t) + amp_sin_d * np.sin(omega * t)
        return 1.0 / l**2

    i_d, _ = quad(integrand_d, 0.0, t_1, epsabs=1e-12, epsrel=1e-12, limit=200)

    c_u = p.c_up
    amp_u = l_t1 - c_u

    def integrand_u(tau):
        l = c_u + amp_u * np.cos(omega * tau)
        return 1.0 / l**2

    i_u, _ = quad(integrand_u, 0.0, tau_2, epsabs=1e-12, epsrel=1e-12, limit=200)
    return p.l0 * v_perp * (i_d + i_u)


def stance_transform(theta_l, p_dot_l, params=PAPER_PARAMS):
    """Landing-to-takeoff map of the stance phase (paper Eqs. 12-21).

    Args:
        theta_l: landing orientation angle ``theta_LD`` (rad): angle between
            the body z axis and the opposite of the landing velocity.
        p_dot_l: landing speed ``|p_dot_LD|`` (m/s).
        params: :class:`StanceParams`.

    Returns:
        :class:`StanceResult` with the takeoff body orientation ``theta_t``,
        takeoff velocity direction ``theta_v`` and takeoff speed ``v_t``.
    """
    theta_l = float(theta_l)
    p_dot_l = float(p_dot_l)
    if p_dot_l <= 0.0:
        return StanceResult(
            theta_t=theta_l, theta_v=theta_l, v_t=0.0,
            t_1=0.0, t_TO=0.0, l_t1=params.l0, l_dot_TO=0.0, beta_TO=0.0,
        )

    omega = params.omega
    v_perp = p_dot_l * np.sin(theta_l)
    t_1, l_t1, _v_p = _compression_end(theta_l, p_dot_l, params)
    tau_2 = _takeoff_time(l_t1, params)
    t_TO = t_1 + tau_2

    beta_TO = _integrate_stance(t_1, tau_2, l_t1, v_perp, params, theta_l, p_dot_l)
    l_dot_TO = -(l_t1 - params.c_up) * omega * np.sin(omega * tau_2)
    theta_t = theta_l + beta_TO
    theta_2 = np.arctan2(v_perp, l_dot_TO)
    theta_v = theta_t + theta_2
    v_t = np.hypot(v_perp, l_dot_TO)
    return StanceResult(
        theta_t=theta_t, theta_v=theta_v, v_t=v_t,
        t_1=t_1, t_TO=t_TO, l_t1=l_t1, l_dot_TO=l_dot_TO, beta_TO=beta_TO,
    )


def stance_mapping_grid(theta_ls, speeds, params=PAPER_PARAMS):
    """Vectorised landing-to-takeoff map on a grid of (theta_LD, |p_dot_LD|).

    Returns a dictionary of 2-D arrays (shape ``[len(theta_ls), len(speeds)]``)
    matching the layout of the authors' ``run_this.m`` meshgrid:
    rows = landing angle, columns = landing speed.
    """
    theta_l = np.asarray(theta_ls, dtype=float)[:, None]
    speed = np.asarray(speeds, dtype=float)[None, :]
    v_p = speed * np.cos(theta_l)
    v_perp = speed * np.sin(theta_l)
    omega = params.omega
    c_d = params.c_down
    c_u = params.c_up

    # Compression phase (vectorised closed form).
    t_1 = np.where(
        v_p > 0,
        np.arctan2(v_p, omega * (c_d - params.l0)) / omega,
        0.0,
    )
    l_t1 = c_d - (v_p / omega) * np.sin(omega * t_1) - (c_d - params.l0) * np.cos(omega * t_1)

    # Extension phase: l_u(tau) = C_u + (l_t1 - C_u) cos(omega tau).
    amplitude = l_t1 - c_u
    ratio = np.clip((params.l0 - c_u) / amplitude, -1.0, 1.0)
    tau_2 = np.arccos(ratio) / omega
    t_TO = t_1 + tau_2

    # Angular momentum integral on the grid via fixed Gauss-Legendre nodes.
    nodes = _GL_NODES[None, None, :]  # (1, 1, K)
    weights = _GL_WEIGHTS[None, None, :]

    # Compression phase: t in [0, t_1] -> x = 0.5*(t_1 + t_1*nodes)
    x_d = 0.5 * t_1[..., None] * (nodes + 1.0)
    l_d = c_d + (-(c_d - params.l0)) * np.cos(omega * x_d) + (-(v_p / omega)[..., None]) * np.sin(omega * x_d)
    integ_d = np.where(t_1[..., None] > 0, weights / l_d**2, 0.0)
    i_d = 0.5 * t_1[..., None] * integ_d
    i_d = np.sum(i_d, axis=-1)

    # Extension phase: tau in [0, tau_2].
    x_u = 0.5 * tau_2[..., None] * (nodes + 1.0)
    l_u = c_u + amplitude[..., None] * np.cos(omega * x_u)
    integ_u = np.where(tau_2[..., None] > 0, weights / l_u**2, 0.0)
    i_u = 0.5 * tau_2[..., None] * integ_u
    i_u = np.sum(i_u, axis=-1)

    beta_TO = params.l0 * v_perp * (i_d + i_u)
    l_dot_TO = -amplitude * omega * np.sin(omega * tau_2)
    theta_t = theta_l + beta_TO
    theta_2 = np.arctan2(v_perp, l_dot_TO)
    theta_v = theta_t + theta_2
    v_t = np.hypot(v_perp, l_dot_TO)

    return {
        "theta_t": theta_t,
        "theta_v": theta_v,
        "v_t": v_t,
        "beta_TO": beta_TO,
        "delta_psi": theta_t - theta_l,
        "theta_TO": theta_v - theta_t,
        "t_1": t_1,
        "t_TO": t_TO,
        "l_t1": l_t1,
        "l_dot_TO": l_dot_TO,
    }


def leg_trajectory(theta_l, p_dot_l, params=PAPER_PARAMS, n_pts=400):
    """Sample the stance trajectory l(t), ld(t), beta_dot(t) for one landing.

    Returns a dict with arrays ``t``, ``l``, ``l_dot``, ``beta_dot``, ``beta``
    and scalars ``t_1``, ``t_TO``.
    """
    omega = params.omega
    v_p = p_dot_l * np.cos(theta_l)
    v_perp = p_dot_l * np.sin(theta_l)
    t_1, l_t1, _ = _compression_end(theta_l, p_dot_l, params)
    tau_2 = _takeoff_time(l_t1, params)
    t_TO = t_1 + tau_2

    t_down = np.linspace(0.0, t_1, int(np.ceil(n_pts * t_1 / t_TO)) + 1) if t_1 > 0 else np.array([0.0])
    t_up = np.linspace(t_1, t_TO, n_pts - len(t_down) + 2)[1:]
    t = np.concatenate([t_down, t_up]) if len(t_up) else t_down

    c_d = params.c_down
    c_u = params.c_up
    amplitude = l_t1 - c_u
    l = np.where(
        t <= t_1,
        c_d - (v_p / omega) * np.sin(omega * t) - (c_d - params.l0) * np.cos(omega * t),
        c_u + amplitude * np.cos(omega * (t - t_1)),
    )
    l_dot = np.where(
        t <= t_1,
        -v_p * np.cos(omega * t) + (c_d - params.l0) * omega * np.sin(omega * t),
        -amplitude * omega * np.sin(omega * (t - t_1)),
    )
    beta_dot = params.l0 * v_perp / l**2
    # Cumulative rotation by numerical integration of beta_dot.
    beta = np.concatenate([[0.0], np.cumsum(0.5 * (beta_dot[1:] + beta_dot[:-1]) * np.diff(t))])
    return {
        "t": t,
        "l": l,
        "l_dot": l_dot,
        "beta_dot": beta_dot,
        "beta": beta,
        "t_1": t_1,
        "t_TO": t_TO,
        "l_t1": l_t1,
    }
