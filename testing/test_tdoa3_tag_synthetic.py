import sys
import unittest
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from tdoa3_tag_synthetic import (
    ANTENNA_DELAY_TICKS,
    DW1000,
    SPEED_OF_LIGHT,
    TDOA3_Tag,
    unwrap_delta,
    wrap_diff,
)


class WrapDiffTests(unittest.TestCase):
    def test_mixed_width_values_across_32_bit_boundary(self):
        tag_tx_40 = (1 << 32) + 5
        anchor_rx_32 = (1 << 32) - 5

        self.assertEqual(wrap_diff(tag_tx_40, anchor_rx_32, bits=40), 10)

    def test_40_bit_counter_wraps_in_both_directions(self):
        modulus = 1 << 40

        self.assertEqual(wrap_diff(2, modulus - 3, bits=40), 5)
        self.assertEqual(wrap_diff(modulus - 3, 2, bits=40), -5)

    def test_signed_32_bit_half_range(self):
        half_range = 1 << 31

        self.assertEqual(wrap_diff(half_range - 1, 0, bits=32), half_range - 1)
        self.assertEqual(wrap_diff(half_range, 0, bits=32), -half_range)
        self.assertEqual(wrap_diff(0, half_range - 1, bits=32), -half_range + 1)

    def test_signed_40_bit_half_range(self):
        half_range = 1 << 39

        self.assertEqual(wrap_diff(half_range - 1, 0, bits=40), half_range - 1)
        self.assertEqual(wrap_diff(half_range, 0, bits=40), -half_range)
        self.assertEqual(wrap_diff(0, half_range - 1, bits=40), -half_range + 1)

    def test_unsigned_delta_preserves_positive_wrapped_difference(self):
        self.assertEqual(wrap_diff(3, (1 << 32) - 4, bits=32, signed=False), 7)

    def test_unwrap_chooses_alias_nearest_clock_ratio_estimate(self):
        modulus = 1 << 32
        raw_delta = modulus - 100

        self.assertEqual(
            unwrap_delta(raw_delta, raw_delta * 1.00007, bits=32),
            raw_delta,
        )
        self.assertEqual(
            unwrap_delta(100, modulus + 100, bits=32),
            modulus + 100,
        )
        self.assertEqual(
            unwrap_delta(modulus - 100, -100, bits=32),
            -100,
        )


class TofValidationTests(unittest.TestCase):
    def test_expected_tof_includes_firmware_antenna_delay(self):
        tag = TDOA3_Tag(1)
        geometric_tof = tag.TOF_ref_anchors[1][2]
        tolerance_ticks = 1.0 / (SPEED_OF_LIGHT * DW1000.TIME_UNIT)

        self.assertTrue(
            tag.validate_measured_TOF(
                1, 2, round(geometric_tof + ANTENNA_DELAY_TICKS)
            )
        )
        self.assertFalse(tag.validate_measured_TOF(1, 2, geometric_tof))
        self.assertFalse(
            tag.validate_measured_TOF(
                1,
                2,
                round(geometric_tof + ANTENNA_DELAY_TICKS + 2 * tolerance_ticks),
            )
        )


if __name__ == "__main__":
    unittest.main()
