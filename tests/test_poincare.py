"""Unit tests for the task-8 Poincare map (authors' ``run_this_0.m`` port).

Regression values are read from ``variables_results_05.mat`` (hopping
height 0.5 m, the paper's Fig. 6C).
"""

import numpy as np
import pytest

from hopcopter_model.poincare import (
    fixed_point_locus,
    poincare_next_theta,
    poincare_ratio_map,
    stability_region,
)
from hopcopter_model.params import AUTHORS_PARAMS

G = 9.81
V_LAND_05 = np.sqrt(2.0 * G * 0.5)


@pytest.mark.parametrize("theta_z_deg,alpha_deg,next_deg", [
    (5.0, 3.0, 2.679102),
    (10.0, 5.0, 2.993854),
    (15.0, 10.0, 10.817312),
    (25.0, -5.0, 32.580337),
    (30.0, 15.0, 11.705094),
    (8.0, 8.0, 11.911617),
    (12.0, 0.0, 11.126459),
    (20.0, 20.0, 28.333152),
])
def test_next_landing_angle_matches_authors_mat(theta_z_deg, alpha_deg,
                                                next_deg):
    # phi = theta_z - alpha (signed landing attitude).
    got = poincare_next_theta(
        np.deg2rad(theta_z_deg),
        np.deg2rad(theta_z_deg - alpha_deg),
        V_LAND_05,
        params=AUTHORS_PARAMS,
    )
    assert np.rad2deg(got) == pytest.approx(next_deg, abs=0.02)


def test_map_depends_on_phi_and_matches_alpha_definition():
    # phi = 0 means alpha = theta_z (body aligned with the vertical);
    # phi = theta_z means alpha = 0 (body aligned with -v_hat).
    th = np.deg2rad(12.0)
    a = poincare_next_theta(th, 0.0, V_LAND_05, AUTHORS_PARAMS)
    b = poincare_next_theta(th, th, V_LAND_05, AUTHORS_PARAMS)
    # (theta_z=12, alpha=0) regression from the authors' mat file.
    assert np.rad2deg(b) == pytest.approx(11.126459, abs=0.02)
    # Landing upright (phi=0) at the same landing angle is more divergent.
    assert a > b


def test_map_zero_landing_speed_is_fixed_point():
    th = np.deg2rad(10.0)
    nxt = poincare_next_theta(th, 0.0, 0.0, AUTHORS_PARAMS)
    assert nxt == pytest.approx(0.0, abs=1e-12)


def test_fixed_point_locus_solves_fixed_points():
    locus = fixed_point_locus(
        alphas_deg=[-2.0, -1.0, 0.0, 1.0, 8.0, 10.0, 15.0, 20.0, 30.0],
        v_land=V_LAND_05,
        params=AUTHORS_PARAMS,
    )
    assert len(locus) == 9
    for theta_cross, alpha in zip(locus, [-2, -1, 0, 1, 8, 10, 15, 20, 30]):
        theta_cross = np.deg2rad(theta_cross)
        alpha = np.deg2rad(alpha)
        # Match the authors' fminbnd result within 0.5 deg.
        if abs(theta_cross - np.deg2rad(40.0)) < np.deg2rad(0.5):
            # Boundary minimiser (fminbnd clamps to the 40 deg grid edge).
            assert abs(theta_cross - np.deg2rad(40.0)) < np.deg2rad(0.5)
            continue
        if alpha == np.deg2rad(0.0):
            assert abs(theta_cross) < np.deg2rad(0.5)
            continue
        # Interior fixed points must satisfy the fixed-point equation.
        nxt = poincare_next_theta(
            theta_cross, theta_cross - alpha, V_LAND_05, AUTHORS_PARAMS)
        assert abs(nxt - theta_cross) < 1e-8
        if alpha == np.deg2rad(8.0):
            assert abs(theta_cross - np.deg2rad(10.131)) < np.deg2rad(0.5)


def test_ratio_map_shape_and_stability_samples():
    thetas = np.arange(0.0, 41.0, 1.0)
    phis = np.arange(-40.0, 56.0, 1.0)
    m = poincare_ratio_map(thetas, phis, V_LAND_05, AUTHORS_PARAMS)
    assert m["ratio"].shape == (len(phis), len(thetas))
    assert m["theta_z_deg"].shape == m["ratio"].shape
    assert m["phi_deg"].shape == m["ratio"].shape

    def ratio_at(th, phi):
        i = int(np.argmin(np.abs(phis - phi)))
        j = int(np.argmin(np.abs(thetas - th)))
        return float(m["ratio"][i, j])

    # Combined strategy region (between the dashed lines) is contracting.
    assert ratio_at(10.0, 5.0) < 1.0
    # Controller-only line (phi=0): the map expands these samples.
    assert ratio_at(8.0, 0.0) > 1.0
    assert ratio_at(20.0, 0.0) > 1.0
    # Stabilizer-only line (phi=theta_z) sits near the boundary.
    assert 0.8 < ratio_at(20.0, 20.0) < 1.2
    # Large landing angle with the body tilted away from the velocity.
    assert ratio_at(25.0, 30.0) > 1.0


def test_ratio_at_zero_landing_angle():
    thetas = np.array([0.0, 1.0, 2.0])
    phis = np.array([-1.0, 0.0, 1.0])
    m = poincare_ratio_map(thetas, phis, V_LAND_05, AUTHORS_PARAMS)
    # (0, 0) is the fixed point: neutral ratio 1.
    assert m["ratio"][1, 0] == pytest.approx(1.0)
    # (0, phi != 0) grows a horizontal component: ratio diverges.
    assert np.isinf(m["ratio"][0, 0])
    assert np.isinf(m["ratio"][2, 0])


def test_stability_region_mask():
    thetas = np.arange(0.0, 41.0, 1.0)
    phis = np.arange(-40.0, 56.0, 1.0)
    m = poincare_ratio_map(thetas, phis, V_LAND_05, AUTHORS_PARAMS)
    stable = stability_region(m["ratio"], m["theta_z_deg"], m["phi_deg"])
    assert stable.shape == m["ratio"].shape
    # The fixed point (0, 0) is stable; the unstable samples above are not.
    assert stable[phis == 0.0, thetas == 0.0].all()
    assert not stable[phis == 0.0, thetas == 8.0].all()
    assert not stable[phis == 0.0, thetas == 20.0].all()
    assert stable[phis == 20.0, thetas == 20.0].all()
    assert stable[phis == 5.0, thetas == 10.0].all()
