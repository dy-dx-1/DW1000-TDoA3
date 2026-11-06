"""
Unit conversions. All internal computations are done in meters.
UNITS in config.py only applies to the ANCHORS dict (input) and to the output coordinates.
"""
from config import UNITS, DECIMALS

UNIT_TO_METERS = {"m": 1.0, "cm": 0.01, "mm": 0.001}

if UNITS not in UNIT_TO_METERS:
    raise ValueError(f"Unknown UNITS '{UNITS}' in config.py. Available units: {', '.join(UNIT_TO_METERS)}")
_SCALE = UNIT_TO_METERS[UNITS]


def anchors_to_meters(anchors:dict[int, tuple])->dict[int, tuple[float, float, float]]:
    """Converts an {anchor_id: (x,y,z)} dict from UNITS to meters."""
    return {anchor_id: tuple(c * _SCALE for c in pos) for anchor_id, pos in anchors.items()}


def position_from_meters(position:tuple)->tuple[float, float, float]:
    """Converts an (x,y,z) position from meters to UNITS, rounded to DECIMALS (whole numbers when DECIMALS is 0)."""
    if DECIMALS == 0:
        return tuple(round(c / _SCALE) for c in position)
    return tuple(round(c / _SCALE, DECIMALS) for c in position)


def covariance_from_meters(value:float)->float:
    """Converts a covariance entry from m² to UNITS² (not rounded, variances are often below DECIMALS)."""
    return value / _SCALE**2
