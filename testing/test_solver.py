import sys
import unittest
from pathlib import Path

import numpy as np


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config import ANCHORS
from pose import CycleUpdate, TDOA3_Pose
from solver import SPEED_OF_LIGHT, multilaterate


def make_tdoa_data(tag_pos, ref_id, anchors=ANCHORS):
    """Exact TDOA pairs (tag->ref - tag->remote) for a tag at tag_pos."""
    tag_pos = np.array(tag_pos)
    ref_pos = np.array(anchors[ref_id])
    data = []
    for anchor_id, pos in anchors.items():
        if anchor_id == ref_id:
            continue
        delta = np.linalg.norm(tag_pos - ref_pos) - np.linalg.norm(tag_pos - np.array(pos))
        data.append((delta / SPEED_OF_LIGHT, anchors[ref_id], pos))
    return data


class MultilaterateTests(unittest.TestCase):
    def setUp(self):
        self.initial = np.mean(np.array(list(ANCHORS.values())), axis=0)

    def test_recovers_known_position(self):
        true_pos = (2.5, 3.0, 0.8)
        pose = multilaterate(make_tdoa_data(true_pos, ref_id=1), self.initial)

        self.assertIsInstance(pose, TDOA3_Pose)
        np.testing.assert_allclose(pose.position, true_pos, atol=1e-3)
        self.assertFalse(pose.mode_2d)
        self.assertIsNone(pose.ref_anchor)

    def test_covariance_is_nan_until_estimated(self):
        pose = multilaterate(make_tdoa_data((2.5, 3.0, 0.8), ref_id=1), self.initial)

        self.assertEqual(pose.covariance.shape, (3, 3))
        self.assertTrue(np.isnan(pose.covariance).all())

    def test_too_few_pairs_returns_none(self):
        data = make_tdoa_data((2.5, 3.0, 0.8), ref_id=1)[:2]

        self.assertIsNone(multilaterate(data, self.initial))


class CycleUpdateTests(unittest.TestCase):
    def test_pose_defaults_to_none(self):
        update = CycleUpdate(stamp_ns=1, n_known_anchors=8, n_aggregated=5, n_pairs=2)

        self.assertIsNone(update.pose)


if __name__ == "__main__":
    unittest.main()
