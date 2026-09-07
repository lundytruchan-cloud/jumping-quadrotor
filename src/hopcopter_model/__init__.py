"""Hopcopter stance-phase dynamics, Poincare map and stabilizer model."""

from .params import StanceParams, AUTHORS_PARAMS, PAPER_PARAMS
from .poincare import (
    fixed_point_locus,
    poincare_next_theta,
    poincare_ratio_map,
    stability_region,
)
from .stabilizer import (
    AERO_PARAMS,
    AeroStabilizerParams,
    aero_aligning_torque,
    descent_control_mode,
    stabilizer_active,
)
from .stance import StanceResult, stance_transform, stance_mapping_grid, leg_trajectory

__all__ = [
    "StanceParams",
    "AUTHORS_PARAMS",
    "PAPER_PARAMS",
    "StanceResult",
    "stance_transform",
    "stance_mapping_grid",
    "leg_trajectory",
    "AERO_PARAMS",
    "AeroStabilizerParams",
    "aero_aligning_torque",
    "stabilizer_active",
    "descent_control_mode",
    "poincare_next_theta",
    "poincare_ratio_map",
    "stability_region",
    "fixed_point_locus",
]
