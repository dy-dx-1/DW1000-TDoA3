import contextlib
import csv
import io
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from outputs import make_output_controllers
from outputs.console import ConsoleOutputController
from outputs.csv_output import CSV_HEADER, CsvOutputController
from pose import CycleUpdate, TDOA3_Pose


def make_update(pose=True):
    return CycleUpdate(
        stamp_ns=1_700_000_000_123_000_000,
        n_known_anchors=8,
        n_aggregated=6,
        n_pairs=5 if pose else 1,
        pose=TDOA3_Pose(x=1.5, y=2.5, z=0.5) if pose else None,
    )


class MakeOutputControllersTests(unittest.TestCase):
    def test_builds_requested_controllers_in_order(self):
        controllers = make_output_controllers(["csv", "console"], tag_id=3)

        self.assertIsInstance(controllers[0], CsvOutputController)
        self.assertIsInstance(controllers[1], ConsoleOutputController)
        self.assertEqual(controllers[0].filename, "tdoa3_tag_3.csv")

    def test_unknown_output_raises(self):
        with self.assertRaises(ValueError):
            make_output_controllers(["console", "carrier_pigeon"], tag_id=1)

    def test_does_not_import_radio_driver(self):
        make_output_controllers(["console", "csv"], tag_id=1)

        self.assertNotIn("dw1000_spi", sys.modules)


class ConsoleOutputControllerTests(unittest.TestCase):
    def test_prints_pair_count_and_position(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            ConsoleOutputController(1).publish(make_update())
            ConsoleOutputController(1).publish(make_update(pose=False))

        self.assertEqual(out.getvalue().splitlines(), [
            "Sending a processed list of size 5",
            "[INFO] -----> New position estimated: (1.5, 2.5, 0.5) m",
            "Sending a processed list of size 1",
            "[INFO] -----> New position estimated: None",
        ])


class CsvOutputControllerTests(unittest.TestCase):
    def setUp(self):
        self._cwd = os.getcwd()
        self._tmp = tempfile.TemporaryDirectory()
        os.chdir(self._tmp.name)

    def tearDown(self):
        os.chdir(self._cwd)
        self._tmp.cleanup()

    def read_rows(self, controller):
        with open(controller.filename, newline="", encoding="utf-8") as f:
            return list(csv.reader(f))

    def test_buffers_until_close_then_writes_header_and_rows(self):
        controller = CsvOutputController(7)
        controller.publish(make_update())
        controller.publish(make_update(pose=False))
        self.assertFalse(os.path.exists(controller.filename))

        controller.close()
        rows = self.read_rows(controller)

        self.assertEqual(tuple(rows[0]), CSV_HEADER)
        self.assertEqual(rows[1][1:], ["8", "6", "5", "1.5", "2.5", "0.5"])
        self.assertEqual(rows[2][1:], ["8", "6", "1", "", "", ""])
        self.assertTrue(rows[1][0].startswith("20"))  # ISO timestamp

    def test_flushes_when_row_limit_reached(self):
        controller = CsvOutputController(7)
        controller._max_rows = 2
        controller.publish(make_update())
        self.assertFalse(os.path.exists(controller.filename))

        controller.publish(make_update())

        self.assertEqual(len(self.read_rows(controller)), 3)

    def test_appends_to_existing_file_without_repeating_header(self):
        for _ in range(2):
            controller = CsvOutputController(7)
            controller.publish(make_update())
            controller.close()

        rows = self.read_rows(controller)

        self.assertEqual(len(rows), 3)
        self.assertEqual([r for r in rows if tuple(r) == CSV_HEADER], [rows[0]])

    def test_close_without_rows_creates_no_file(self):
        controller = CsvOutputController(7)
        controller.close()

        self.assertFalse(os.path.exists(controller.filename))


class TagOutputLifecycleTests(unittest.TestCase):
    def setUp(self):
        # tdoa3_tag imports the real radio driver (needs spidev), so stub it for these tests only
        self.radio = MagicMock()
        fake_driver = types.ModuleType("dw1000_spi")
        fake_driver.DW1000 = MagicMock(return_value=self.radio)
        fake_driver.DW1000.TIME_UNIT = 1.5650040064102565e-11
        self._modules = patch.dict(sys.modules, {"dw1000_spi": fake_driver})
        self._modules.start()
        sys.modules.pop("tdoa3_tag", None)
        import tdoa3_tag
        self.tdoa3_tag = tdoa3_tag

    def tearDown(self):
        sys.modules.pop("tdoa3_tag", None)
        self._modules.stop()

    def test_exit_closes_every_controller_then_radio_even_if_one_fails(self):
        tag = self.tdoa3_tag.TDOA3_Tag(1, outputs=[])
        failing, ok = MagicMock(), MagicMock()
        failing.close.side_effect = OSError("disk full")
        tag.output_controllers = [failing, ok]

        with contextlib.redirect_stdout(io.StringIO()):
            with tag:
                pass

        failing.close.assert_called_once()
        ok.close.assert_called_once()
        self.radio.close.assert_called_once()

    def test_invalid_output_fails_before_opening_radio(self):
        with self.assertRaises(ValueError):
            self.tdoa3_tag.TDOA3_Tag(1, outputs=["nope"])

        sys.modules["dw1000_spi"].DW1000.assert_not_called()


if __name__ == "__main__":
    unittest.main()
