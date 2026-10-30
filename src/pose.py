"""
Result types shared by the tag loop, the solver and the outputs.
Pure module (numpy only) so it can be imported without the DW1000/spidev stack.
"""
from dataclasses import dataclass, field
import numpy as np


def _nan_covariance() -> np.ndarray:
    return np.full((3, 3), np.nan)


@dataclass
class TDOA3_Pose:
    """
    A position solved by multilaterate(). 

    ATTRIBUTES:
    - x, y, z:    Position in meters, in the anchor coordinate frame
    - covariance: 3x3 position covariance in m², order x,y,z. NaN-filled until the solver estimates it.
                  In 2D mode the z row/col is 0 and is replaced at output.
    - ref_anchor: ID of the reference anchor of the TDOA pairs, None if unknown
    - mode_2d:    Whether z was fixed and only x,y were solved

    Only valid solves produce a TDOA3_Pose; rejected or failed solves return None.
    """
    x: float
    y: float
    z: float
    covariance: np.ndarray = field(default_factory=_nan_covariance)
    ref_anchor: int | None = None
    mode_2d: bool = False

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass
class CycleUpdate:
    """
    Summary of one localization loop iteration. Generated every cycle, even when no pose could be solved.

    ATTRIBUTES:
    - stamp_ns:        Host wall time of the cycle (taken right after the solve), in the time.time_ns() domain.
                       Also the pose's timestamp; not using packet receive times as it adds unneeded complexity
    - n_known_anchors: Number of anchors with a known position
    - n_aggregated:    Number of anchors in the aggregated data
    - n_pairs:         Number of TDOA pairs computed
    - pose:            The solved TDOA3_Pose, or None if there was no solution this cycle
    """
    stamp_ns: int
    n_known_anchors: int
    n_aggregated: int
    n_pairs: int
    pose: TDOA3_Pose | None = None
