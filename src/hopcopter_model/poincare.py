"""Poincare map of the hopping dynamics (authors' ``run_this_0.m`` port).

The landing state is sampled once per hopping cycle at touchdown.  With a
constant hopping height the vertical landing speed is fixed, so the map is
one-dimensional in the landing-velocity direction angle

    theta_z = angle between the vertical z_w and -p_dot(t_LD),

parametrised by the signed landing attitude

    phi = theta_z - alpha,   alpha = signed angle z_b to -p_dot(t_LD).

The stance phase maps the landing state to the takeoff state (task-4
module, paper Eqs. 12-21); after takeoff the powered climb restores the
cycle energy and the horizontal takeoff speed is conserved until the next
landing, which yields ``theta_z|k+1`` (paper Materials and Methods,
"Poincare maps for hopping stability analysis"; Eq. 37 fixes the vertical
landing speed).  This mirrors the authors' ``run_this_*.m`` scripts: the
contours of ``theta_z|k+1 / theta_z|k`` reproduce paper Fig. 6C / fig. S8.
"""

import numpy as np
from scipy.optimize import brentq

from .params import AUTHORS_PARAMS
from .stance import stance_transform


# Fixed-order Gauss-Legendre rule (same as stance.stance_mapping_grid).
_GL_NODES, _GL_WEIGHTS = np.polynomial.legendre.leggauss(128)


def _stance_paired(theta_l, speed, params):
    """Vectorised stance map for equal-length input arrays.

    Same closed-form solution as ``stance.stance_mapping_grid`` but along a
    single paired axis (used by the Poincare grid evaluation).
    """
    theta_l = np.asarray(theta_l, dtype=float)
    speed = np.asarray(speed, dtype=float)
    v_p = speed * np.cos(theta_l)
    v_perp = speed * np.sin(theta_l)
    omega = params.omega
    c_d = params.c_down
    c_u = params.c_up

    t_1 = np.where(
        v_p > 0,
        np.arctan2(v_p, omega * (c_d - params.l0)) / omega,
        0.0,
    )
    l_t1 = c_d - (v_p / omega) * np.sin(omega * t_1) - (
        c_d - params.l0) * np.cos(omega * t_1)
    amplitude = l_t1 - c_u
    ratio = np.clip((params.l0 - c_u) / amplitude, -1.0, 1.0)
    tau_2 = np.arccos(ratio) / omega

    nodes = _GL_NODES[None, :]
    weights = _GL_WEIGHTS[None, :]
    x_d = 0.5 * t_1[..., None] * (nodes + 1.0)
    l_d = c_d + (-(c_d - params.l0)) * np.cos(omega * x_d) + (
        -(v_p / omega)[..., None]) * np.sin(omega * x_d)
    integ_d = np.where(t_1[..., None] > 0, weights / l_d ** 2, 0.0)
    i_d = np.sum(0.5 * t_1[..., None] * integ_d, axis=-1)

    x_u = 0.5 * tau_2[..., None] * (nodes + 1.0)
    l_u = c_u + amplitude[..., None] * np.cos(omega * x_u)
    integ_u = np.where(tau_2[..., None] > 0, weights / l_u ** 2, 0.0)
    i_u = np.sum(0.5 * tau_2[..., None] * integ_u, axis=-1)

    beta = params.l0 * v_perp * (i_d + i_u)
    l_dot_to = -amplitude * omega * np.sin(omega * tau_2)
    theta_t = theta_l + beta
    theta_v = theta_t + np.arctan2(v_perp, l_dot_to)
    v_t = np.hypot(v_perp, l_dot_to)
    return theta_v, v_t


def poincare_next_theta(theta_z, phi, v_land, params=AUTHORS_PARAMS):
    """Next-cycle landing-velocity angle ``theta_z|k+1`` (rad).

    Args:
        theta_z: landing velocity direction angle from the vertical, rad.
        phi: signed landing attitude (body axis vs vertical), rad.
        v_land: vertical landing speed ``sqrt(2 g z_d)``, m/s.
        params: stance parameters (authors' values by default).
    """
    theta_z = float(theta_z)
    phi = float(phi)
    v_land = float(v_land)
    if v_land <= 0.0:
        return 0.0

    alpha = theta_z - phi  # signed theta_LD (body to -v_hat)
    landing_speed = v_land / np.cos(theta_z)
    res = stance_transform(abs(alpha), landing_speed, params)
    takeoff_speed_theta = theta_z - res.theta_v * np.sign(alpha)
    takeoff_speed_x = res.v_t * np.sin(takeoff_speed_theta)
    return np.pi / 2.0 - np.arctan2(v_land, abs(takeoff_speed_x))


def poincare_ratio_map(theta_zs_deg, phis_deg, v_land, params=AUTHORS_PARAMS):
    """Evaluate the map on the paper's grid of ``(theta_z, phi)`` degrees.

    Returns a dict with 2-D arrays shaped ``[len(phis), len(thetas)]``
    (rows = phi, columns = theta_z, matching the authors' meshgrid layout)
    containing ``theta_z|k+1`` (deg), the ratio and the coordinate grids.
    At ``theta_z = 0`` the ratio is 1 for ``phi = 0`` (the fixed point) and
    ``inf`` otherwise (a horizontal component appears).
    """
    theta_deg = np.asarray(theta_zs_deg, dtype=float)
    phi_deg = np.asarray(phis_deg, dtype=float)
    theta_grid, phi_grid = np.meshgrid(theta_deg, phi_deg)
    theta_rad = np.deg2rad(theta_grid)
    phi_rad = np.deg2rad(phi_grid)
    alpha = theta_rad - phi_rad
    with np.errstate(divide="ignore", invalid="ignore"):
        landing_speed = v_land / np.cos(theta_rad)
    theta_v, v_t = _stance_paired(abs(alpha).ravel(),
                                  landing_speed.ravel(), params)
    theta_v = theta_v.reshape(phi_rad.shape)
    v_t = v_t.reshape(phi_rad.shape)
    takeoff_speed_theta = theta_rad - theta_v * np.sign(alpha)
    takeoff_speed_x = v_t * np.sin(takeoff_speed_theta)
    next_rad = np.pi / 2.0 - np.arctan2(v_land, abs(takeoff_speed_x))
    next_deg = np.rad2deg(next_rad)

    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(theta_grid > 0.0, next_deg / theta_grid, np.nan)
    # theta_z = 0: no horizontal speed.  With phi = 0 it is the fixed point
    # (neutral ratio); with phi != 0 the next cycle has a horizontal
    # component, so the ratio is unbounded.
    ratio = np.where((theta_grid == 0.0) & (phi_grid == 0.0), 1.0, ratio)
    ratio = np.where(
        (theta_grid == 0.0) & (phi_grid != 0.0), np.inf, ratio)
    return {
        "theta_z_deg": theta_grid,
        "phi_deg": phi_grid,
        "next_theta_z_deg": next_deg,
        "ratio": ratio,
    }


def stability_region(ratio, theta_grid_deg, phi_grid_deg):
    """Stable region of paper Fig. 6C: ``theta_z|k+1/theta_z|k < 1``.

    The exact fixed point ``(theta_z, phi) = (0, 0)`` is included.
    """
    ratio = np.asarray(ratio, dtype=float)
    theta_grid = np.asarray(theta_grid_deg, dtype=float)
    phi_grid = np.asarray(phi_grid_deg, dtype=float)
    stable = ratio < 1.0
    stable |= (theta_grid == 0.0) & (phi_grid == 0.0)
    return stable


def fixed_point_locus(alphas_deg, v_land, params=AUTHORS_PARAMS,
                      theta_max_deg=40.0):
    """Landing angles that are fixed points of the map for each ``alpha``.

    ``alpha`` is the signed body-to-velocity angle ``theta_LD``; the
    corresponding landing attitude is ``phi = theta_z - alpha``.  This is
    the authors' ``fminbnd`` search over ``|NXT(theta) - theta|`` (the
    white/black solid curve of Fig. 6C and fig. S8).
    """
    alphas = np.deg2rad(np.asarray(alphas_deg, dtype=float))
    theta_max = np.deg2rad(theta_max_deg)
    crosses = []
    for alpha in alphas:
        def err(theta):
            return poincare_next_theta(
                theta, theta - alpha, v_land, params) - theta

        # The map is smooth on (0, theta_max]; scan for a sign change with
        # the angle in degrees to keep the bracket dense enough.
        deg_grid = np.linspace(0.5, theta_max_deg, 200)
        vals = np.array([err(np.deg2rad(d)) for d in deg_grid])
        root = None
        for i in range(len(deg_grid) - 1):
            if vals[i] == 0.0:
                root = deg_grid[i]
                break
            if vals[i] * vals[i + 1] < 0.0:
                root = brentq(err, np.deg2rad(deg_grid[i]),
                              np.deg2rad(deg_grid[i + 1]), xtol=1e-12)
                root = np.rad2deg(root)
                break
        if root is None:
            # No fixed point inside the range (e.g. alpha = 0 only has the
            # trivial theta = 0 fixed point); fall back to the minimiser.
            grid_deg = np.linspace(0.0, theta_max_deg, 4001)
            grid_err = np.abs(vals[::20]) if len(vals) else None
            idx = int(np.argmin(np.abs(
                [err(np.deg2rad(d)) for d in grid_deg])))
            root = float(grid_deg[idx])
        crosses.append(float(root))
    return np.asarray(crosses)
