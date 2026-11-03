import contextlib
import io
import pickle
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch


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
from pose import TDOA3_Pose


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
        tag = TDOA3_Tag(1, outputs=[])
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


class TdoaBaselineTests(unittest.TestCase):
    def test_uses_cached_baseline_tof_in_ticks(self):
        tag = TDOA3_Tag(1, outputs=[])
        tag.anchors = {1: (0, 0, 0), 2: (100, 0, 0)}
        tag.TOF_ref_anchors = {1: {2: 100}}
        aggregated_data = {
            1: [
                {"seq": 1, "tx": 0, "rx": 0, "remote_data": {}},
                {
                    "seq": 2,
                    "tx": 10_000,
                    "rx": 10_000,
                    "remote_data": {2: (3, 9_000, 0)},
                },
            ],
            2: [{"seq": 3, "rx": 8_899}],
        }

        with patch.object(tag, "validate_measured_TOF", return_value=True):
            results = tag._tdoa_from_ref(aggregated_data, 1)
            self.assertEqual(results, [])

            aggregated_data[2][0]["rx"] = 8_900
            results = tag._tdoa_from_ref(aggregated_data, 1)

        self.assertEqual(len(results), 1)
        self.assertAlmostEqual(results[0][0], 100 * DW1000.TIME_UNIT)


class SyntheticOutputTests(unittest.TestCase):
    def test_publishes_each_cycle_and_closes_controllers(self):
        controller = MagicMock()
        with patch(
            "tdoa3_tag_synthetic.make_output_controllers",
            return_value=[controller],
        ) as make_controllers:
            tag = TDOA3_Tag(7, outputs=["csv"])

        make_controllers.assert_called_once_with(("csv",), 7)
        tag.aggregate_raw_pkts = MagicMock(return_value={1: []})
        tag.process_anchor_data = MagicMock(return_value=[(1, 2, 3)])
        tag.multilaterate = MagicMock(return_value=TDOA3_Pose(1, 2, 3))

        with tempfile.TemporaryDirectory() as temp_dir:
            saved_data = Path(temp_dir) / "runs.pkl"
            saved_data.write_bytes(pickle.dumps([[]]))
            with contextlib.redirect_stdout(io.StringIO()):
                with tag:
                    tag.run_synthetic(str(saved_data), enable_print=True)

        controller.publish.assert_called_once()
        update = controller.publish.call_args.args[0]
        self.assertEqual(update.n_known_anchors, len(tag.anchors))
        self.assertEqual(update.n_aggregated, 1)
        self.assertEqual(update.n_pairs, 1)
        self.assertEqual(update.pose.position, (1, 2, 3))
        controller.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
