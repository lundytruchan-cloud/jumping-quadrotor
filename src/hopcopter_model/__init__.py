"""Hopcopter stance-phase dynamics (paper Eqs. 12-21)."""

from .params import StanceParams, AUTHORS_PARAMS, PAPER_PARAMS
from .stance import StanceResult, stance_transform, stance_mapping_grid, leg_trajectory

__all__ = [
    "StanceParams",
    "AUTHORS_PARAMS",
    "PAPER_PARAMS",
    "StanceResult",
    "stance_transform",
    "stance_mapping_grid",
    "leg_trajectory",
]
