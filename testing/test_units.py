import sys
import unittest
from pathlib import Path
from unittest.mock import patch


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import units
from units import UNIT_TO_METERS, anchors_to_meters, position_from_meters


def with_units(unit, decimals=3):
    """Patches the UNITS/DECIMALS that units.py read from config.py."""
    return patch.multiple(units, _SCALE=UNIT_TO_METERS[unit], DECIMALS=decimals)


class AnchorsToMetersTests(unittest.TestCase):
    def test_converts_every_unit_to_meters(self):
        cases = {"m": (4.736, 2.194, 1.213), "cm": (473.6, 219.4, 121.3), "mm": (4736, 2194, 1213)}
        for unit, pos in cases.items():
            with self.subTest(unit=unit), with_units(unit):
                converted = anchors_to_meters({1: pos})

                for got, expected in zip(converted[1], (4.736, 2.194, 1.213)):
                    self.assertAlmostEqual(got, expected, places=9)

    def test_does_not_modify_input(self):
        anchors = {1: (100, 200, 300)}
        with with_units("cm"):
            anchors_to_meters(anchors)

        self.assertEqual(anchors, {1: (100, 200, 300)})


class PositionFromMetersTests(unittest.TestCase):
    def test_converts_meters_to_each_unit(self):
        cases = {"m": (1.235, 2.5, -0.1), "cm": (123.457, 250.0, -10.0), "mm": (1234.568, 2500.0, -100.0)}
        for unit, expected in cases.items():
            with self.subTest(unit=unit), with_units(unit):
                self.assertEqual(position_from_meters((1.2345678, 2.5, -0.1)), expected)

    def test_rounds_to_decimals(self):
        with with_units("mm", decimals=0):
            rounded = position_from_meters((1.2345678, 0.0004, 0.0006))

            self.assertEqual(rounded, (1235, 0, 1))
            self.assertTrue(all(isinstance(c, int) for c in rounded))
        with with_units("m", decimals=1):
            self.assertEqual(position_from_meters((1.26, 2.0, 3.04)), (1.3, 2.0, 3.0))

    def test_round_trip_with_anchors(self):
        with with_units("cm"):
            anchor = anchors_to_meters({1: (473.6, 219.4, 121.3)})[1]

            self.assertEqual(position_from_meters(anchor), (473.6, 219.4, 121.3))


if __name__ == "__main__":
    unittest.main()
