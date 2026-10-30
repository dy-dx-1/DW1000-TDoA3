import contextlib
import io
import itertools
import pickle
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from outputs.base import OutputController
from solver import multilaterate


class FakeRadio:
    """Replays recorded batches, ending each listening window at the recorded batch boundary."""
    def __init__(self, batches):
        self._items = itertools.chain.from_iterable(list(b) + [None] for b in batches)

    def listen(self, ranging):
        item = next(self._items, StopIteration)
        if item is StopIteration:
            raise KeyboardInterrupt
        if item is None:
            time.sleep(0.12)  # Longer than RECORDING_DURATION at 10 Hz, so the window closes here
            return None, None
        return item

    def close(self):
        pass


class CaptureOutputController(OutputController):
    def __init__(self):
        self.updates = []

    def publish(self, update):
        self.updates.append(update)


class PoseStampTests(unittest.TestCase):
    def setUp(self):
        # tdoa3_tag imports the real radio driver (needs spidev), so stub it for these tests only
        with open(SRC_DIR / "10_1s_raw_datas.pkl", "rb") as f:
            self.batches = pickle.load(f)[:3]
        fake_driver = types.ModuleType("dw1000_spi")
        fake_driver.DW1000 = MagicMock(side_effect=lambda *a, **k: FakeRadio(self.batches))
        fake_driver.DW1000.TIME_UNIT = 1.5650040064102565e-11
        self._modules = patch.dict(sys.modules, {"dw1000_spi": fake_driver})
        self._modules.start()
        sys.modules.pop("tdoa3_tag", None)
        import tdoa3_tag
        self.tdoa3_tag = tdoa3_tag

    def tearDown(self):
        sys.modules.pop("tdoa3_tag", None)
        self._modules.stop()

    def run_tag(self):
        capture = CaptureOutputController()
        with contextlib.redirect_stdout(io.StringIO()) as out:
            with self.tdoa3_tag.TDOA3_Tag(1, outputs=[]) as tag:
                tag.output_controllers = [capture]
                try:
                    tag.run()
                except KeyboardInterrupt:
                    pass
        return tag, capture.updates, out.getvalue()

    def test_cycle_is_stamped_after_solve_with_reference_anchor(self):
        before = time.time_ns()
        tag, updates, _ = self.run_tag()
        poses = [u.pose for u in updates if u.pose is not None]

        self.assertEqual(len(updates), len(self.batches))
        self.assertTrue(poses)
        stamps = [u.stamp_ns for u in updates]
        self.assertGreater(stamps[0], before)
        self.assertEqual(stamps, sorted(stamps))
        for update in updates:
            if update.pose is not None:
                self.assertIn(update.pose.ref_anchor, tag.anchors)

    def test_startup_print_reports_lateness_from_configured_frequency(self):
        _, _, out = self.run_tag()
        expected_ms = f"{self.tdoa3_tag.RECORDING_DURATION * 1000:.0f} ms"

        self.assertIn(f"up to ~{expected_ms}", out.splitlines()[0])

    def test_process_anchor_data_returns_reference_anchor(self):
        tag = self.tdoa3_tag.TDOA3_Tag(1, outputs=[])
        with contextlib.redirect_stdout(io.StringIO()):
            pairs, ref_anchor = tag.process_anchor_data(tag.aggregate_raw_pkts(self.batches[0]))
            no_pairs = self.tdoa3_tag.TDOA3_Tag(1, outputs=[]).process_anchor_data({})

        self.assertTrue(pairs)
        self.assertIn(ref_anchor, tag.fresh_anchors)
        self.assertTrue(all(ref_pos == tag.anchors[ref_anchor] for _, ref_pos, _ in pairs))
        self.assertEqual(no_pairs, ([], None))


class SolverReferenceAnchorTests(unittest.TestCase):
    def setUp(self):
        from test_solver import make_tdoa_data
        self.data = make_tdoa_data((2.5, 3.0, 0.8), ref_id=4)

    def test_reference_is_copied_to_pose(self):
        pose = multilaterate(self.data, (2.5, 3.0, 1.0), ref_anchor=4)

        self.assertEqual(pose.ref_anchor, 4)


if __name__ == "__main__":
    unittest.main()
