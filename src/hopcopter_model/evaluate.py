"""Port of the authors' ``evaluate_data.m`` mocap pipeline.

Steps mirrored from the MATLAB script:

1. Load the Qualisys mocap record (body position, orientation quaternion,
   sample times).
2. Compute the foot position by translating the body frame by
   ``foot_offset = -0.22 m`` along the body z axis.
3. Detect landing/takeoff instants as crossings of the foot height with
   ``jumping_threshold_foot = 0.03 m`` (linear interpolation, as
   ``polyxpoly``) and snap each crossing to the nearest sample.
4. Compute body velocities by central differences (``diff_same``) and read
   them at the crossing samples.
5. For every landing-takeoff pair, fit the ground plane normal
   (``fminsearch`` over two rotation angles), project velocity and
   orientation vectors onto that plane, and compute
   ``theta_LD``, ``theta_TO`` (body) and ``veloc_TO``.
"""

from dataclasses import dataclass

import numpy as np
import scipy.io
from scipy.optimize import minimize

FOOT_OFFSET = -0.22
JUMPING_THRESHOLD_FOOT = 0.03


@dataclass
class MocapData:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    qw: np.ndarray
    qx: np.ndarray
    qy: np.ndarray
    qz: np.ndarray


def load_mocap(mat_path):
    """Load the Qualisys record saved by ``evaluate_data.m``."""
    m = scipy.io.loadmat(mat_path)

    def vec(name):
        return np.asarray(m[name]).ravel()

    return MocapData(
        t=vec("Abs_time"),
        x=vec("b1_x"),
        y=vec("b1_y"),
        z=vec("b1_z"),
        qw=vec("b1_qw"),
        qx=vec("b1_qx"),
        qy=vec("b1_qy"),
        qz=vec("b1_qz"),
    )


def quat_to_rotm(qw, qx, qy, qz):
    """MATLAB ``quat2rotm`` convention: (w, x, y, z) -> Nx3x3 matrix."""
    qw = np.asarray(qw, dtype=float)
    qx = np.asarray(qx, dtype=float)
    qy = np.asarray(qy, dtype=float)
    qz = np.asarray(qz, dtype=float)
    n = qw.shape[0]
    r = np.empty((n, 3, 3))
    r[:, 0, 0] = 1 - 2 * (qy**2 + qz**2)
    r[:, 0, 1] = 2 * (qx * qy - qw * qz)
    r[:, 0, 2] = 2 * (qx * qz + qw * qy)
    r[:, 1, 0] = 2 * (qx * qy + qw * qz)
    r[:, 1, 1] = 1 - 2 * (qx**2 + qz**2)
    r[:, 1, 2] = 2 * (qy * qz - qw * qx)
    r[:, 2, 0] = 2 * (qx * qz - qw * qy)
    r[:, 2, 1] = 2 * (qy * qz + qw * qx)
    r[:, 2, 2] = 1 - 2 * (qx**2 + qy**2)
    return r


def body_z_world(mocap):
    """World-frame body z axis at every sample: R @ [0, 0, 1]."""
    rotm = quat_to_rotm(mocap.qw, mocap.qx, mocap.qy, mocap.qz)
    return rotm[:, :, 2]


def foot_z(mocap, foot_offset=FOOT_OFFSET):
    """Foot height by translating the body frame along its z axis."""
    z_b = body_z_world(mocap)
    return mocap.z + foot_offset * z_b[:, 2]


def central_diff(t, x):
    """``diff_same``: central differences with copied endpoints."""
    x = np.asarray(x, dtype=float)
    t = np.asarray(t, dtype=float)
    d = np.zeros_like(x)
    d[1:-1] = (x[2:] - x[:-2]) / (t[2:] - t[:-2])
    if len(d) > 2:
        d[0] = d[1]
        d[-1] = d[-2]
    return d


def threshold_crossings(t, z, threshold):
    """Linear-interpolation crossings of a polyline with a horizontal line.

    Equivalent to ``polyxpoly(t, z, t[[0, -1]], [thr, thr])`` for this use.
    """
    t = np.asarray(t, dtype=float)
    z = np.asarray(z, dtype=float)
    above = z >= threshold
    crossings = []
    for i in range(len(t) - 1):
        if above[i] == above[i + 1]:
            continue
        frac = (threshold - z[i]) / (z[i + 1] - z[i])
        tc = t[i] + frac * (t[i + 1] - t[i])
        if crossings and abs(tc - crossings[-1]) < 1e-12:
            continue
        crossings.append(tc)
    return np.asarray(crossings, dtype=float)


def angle_deg(a, b):
    """Angle between vectors in degrees (``atan2(norm(cross), dot)``)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return np.rad2deg(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b)))


def _surface_error(x, v_ld, v_to, z_ld, z_to):
    rx, rz = x
    rotx = np.array(
        [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]]
    )
    rotz = np.array(
        [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]]
    )
    n = rotz @ rotx @ np.array([0.0, 0.0, 1.0])
    angles = [
        angle_deg(-v_ld, n),
        angle_deg(v_to, n),
        angle_deg(z_ld, n),
        angle_deg(z_to, n),
    ]
    return np.sqrt(np.mean((90.0 - np.asarray(angles)) ** 2))


def fit_surface(v_ld, v_to, z_ld, z_to):
    """Fit the ground plane normal (two angles) as in ``find_best_surface``."""
    res = minimize(
        _surface_error,
        x0=np.zeros(2),
        args=(v_ld, v_to, z_ld, z_to),
        method="Nelder-Mead",
        options={"xatol": 1e-4, "fatol": 1e-4, "maxiter": 400},
    )
    rx, rz = res.x
    rotx = np.array(
        [[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]]
    )
    rotz = np.array(
        [[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]]
    )
    normal = rotz @ rotx @ np.array([0.0, 0.0, 1.0])
    return normal, res.fun


def project_on_plane(v, normal):
    """Project vectors onto the plane perpendicular to ``normal``."""
    v = np.asarray(v, dtype=float)
    return v - np.dot(v, normal) * normal


def detect_jump_crossings(mocap, foot_offset=FOOT_OFFSET, threshold=JUMPING_THRESHOLD_FOOT):
    """Detect landing/takeoff crossing samples and velocities.

    Returns a dict with crossing times, nearest-sample indices, central-diff
    velocities and 5-point linear-fit vertical speeds (as in the MATLAB
    script).
    """
    t = mocap.t
    zf = foot_z(mocap, foot_offset)
    vx = central_diff(t, mocap.x)
    vy = central_diff(t, mocap.y)
    vz = central_diff(t, mocap.z)

    ti = threshold_crossings(t, zf, threshold)
    idx = np.array([np.argmin(np.abs(t - tc)) for tc in ti])

    # 5-point linear fit of b1_z around each crossing (used for filtering).
    zi_dot = np.zeros(len(idx))
    for i, k in enumerate(idx):
        k = int(k)
        window = slice(k - 2, k + 3)
        xm = np.vstack([t[window], np.ones(5)])
        slope = np.linalg.lstsq(xm.T, mocap.z[window], rcond=None)[0]
        zi_dot[i] = slope[0]

    return {
        "t_cross": ti,
        "index": idx,
        "vx": vx[idx],
        "vy": vy[idx],
        "vz": vz[idx],
        "zi_dot": zi_dot,
    }


def select_jumps(detect):
    """Pair crossings into landing-takeoff jumps.

    The MATLAB script removes a leading takeoff (positive vertical speed) and
    a trailing landing (negative vertical speed), then groups crossings as
    (landing, takeoff), (landing, takeoff), ...
    """
    keep = np.ones(len(detect["index"]), dtype=bool)
    if len(keep) > 0 and detect["zi_dot"][0] > 0:
        keep[0] = False
    if len(keep) > 1 and detect["zi_dot"][-1] < 0:
        keep[-1] = False
    positions = np.flatnonzero(keep)
    n_pairs = len(positions) // 2
    return positions[: 2 * n_pairs].reshape(n_pairs, 2)


def analyze_jumps(
    mocap,
    params=None,
    foot_offset=FOOT_OFFSET,
    threshold=JUMPING_THRESHOLD_FOOT,
    window=None,
):
    """Run the full ``evaluate_data.m`` pipeline.

    ``window`` optionally restricts the analysis to jumps whose landing time
    lies in ``(window[0], window[1])``; the authors' interactive selection in
    ``evaluate_data.m`` corresponds to ``(26.0, 61.0)`` s (verified against
    the packaged ``data_output/saved_data.mat``).

    Returns a numpy structured array with one row per jump containing the
    measured landing/takeoff angles and speeds plus, when ``params`` is given,
    the model predictions and errors.
    """
    from .params import PAPER_PARAMS
    from .stance import stance_transform

    if params is None:
        params = PAPER_PARAMS
    detect = detect_jump_crossings(mocap, foot_offset, threshold)
    pairs = select_jumps(detect)
    z_b = body_z_world(mocap)
    t = mocap.t

    rows = []
    for ld_i, to_i in pairs:
        ld_sample = int(detect["index"][ld_i])
        to_sample = int(detect["index"][to_i])
        if window is not None and not (window[0] <= t[ld_sample] <= window[1]):
            continue
        v_ld = np.array([detect["vx"][ld_i], detect["vy"][ld_i], detect["vz"][ld_i]])
        v_to = np.array([detect["vx"][to_i], detect["vy"][to_i], detect["vz"][to_i]])
        z_ld = z_b[ld_sample]
        z_to = z_b[to_sample]

        normal, surface_error = fit_surface(v_ld, v_to, z_ld, z_to)
        v_ld_p = project_on_plane(v_ld, normal)
        v_to_p = project_on_plane(v_to, normal)
        z_ld_p = project_on_plane(z_ld, normal)
        z_to_p = project_on_plane(z_to, normal)

        theta_landing = angle_deg(-v_ld_p, z_ld_p)
        theta_takeoff = angle_deg(-v_ld_p, z_to_p)
        veloc_takeoff = angle_deg(-v_ld_p, v_to_p)
        landing_speed = np.linalg.norm(v_ld)

        r = stance_transform(np.deg2rad(theta_landing), landing_speed, params)
        delta_psi_model = np.rad2deg(r.theta_t) - theta_landing
        theta_TO_model = np.rad2deg(r.theta_v - r.theta_t)

        rows.append((
            t[ld_sample], t[to_sample], t[to_sample] - t[ld_sample],
            landing_speed,
            theta_landing, theta_takeoff, veloc_takeoff,
            theta_takeoff - theta_landing, veloc_takeoff - theta_takeoff,
            surface_error,
            np.rad2deg(r.theta_t), np.rad2deg(r.theta_v), r.v_t,
            delta_psi_model, theta_TO_model,
            (theta_takeoff - theta_landing) - delta_psi_model,
            (veloc_takeoff - theta_takeoff) - theta_TO_model,
        ))

    names = [
        "t_LD", "t_TO", "stance_duration",
        "landing_speed",
        "theta_landing", "theta_takeoff", "veloc_takeoff",
        "delta_psi", "theta_TO",
        "surface_angle_error",
        "model_theta_t", "model_theta_v", "model_v_t",
        "model_delta_psi", "model_theta_TO",
        "err_delta_psi", "err_theta_TO",
    ]
    return np.array(rows, dtype=[(n, "f8") for n in names])


def rmse(jumps, name_a, name_b):
    return float(np.sqrt(np.mean((jumps[name_a] - jumps[name_b]) ** 2)))
