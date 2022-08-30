from copy import deepcopy
import unittest

import PyKDL as kdl

from ed.volume import Volume, BoxVolume, CompositeBoxVolume, OffsetVolume


class TestShape(unittest.TestCase):
    def test_volume_equal(self):
        """
        Two empty volumes should be equal
        """
        v1 = Volume()
        v2 = deepcopy(v1)
        self.assertEqual(v1, v2)

    def test_box_volume_equal(self):
        """
        Two box volumes should be equal
        """
        v1 = BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1))
        v2 = deepcopy(v1)
        self.assertEqual(v1, v2)
        v3 = BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(2, 2, 2))
        self.assertNotEqual(v1, v3)

    def test_composite_box_volume_equal(self):
        """
        Two composite box volumes should be equal
        """
        v1 = CompositeBoxVolume(
            [(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)), (kdl.Vector(0, 0, 0), kdl.Vector(-1, -1, -1))]
        )
        v2 = deepcopy(v1)
        self.assertEqual(v1, v2)

        v3 = CompositeBoxVolume(
            [(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)), (kdl.Vector(0, 0, 0), kdl.Vector(2, 2, 2))]
        )
        self.assertNotEqual(v1, v3)

    def test_offset_volume_equal(self):
        """
        Two offset volumes should be equal
        """
        v1 = OffsetVolume(1.0)
        v2 = deepcopy(v1)
        self.assertEqual(v1, v2)
        v3 = OffsetVolume(2.0)
        self.assertNotEqual(v1, v3)

    def test_volume_hash(self):
        """
        Two empty volumes should have the same hash and should be consistent
        """
        v1 = Volume()
        v2 = deepcopy(v1)
        self.assertEqual(hash(v1), hash(v2))
        self.assertEqual(5740354900026072187, hash(v1))

    def test_box_volume_hash(self):
        """
        Two box volumes should have the same hash and should be consistent
        """
        v1 = BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1))
        v2 = deepcopy(v1)
        self.assertEqual(hash(v1), hash(v2))
        self.assertEqual(2165702983533035897, hash(v1))
        v3 = BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(2, 2, 2))
        self.assertNotEqual(hash(v1), hash(v3))

    def test_composite_box_volume_hash(self):
        """
        Two box volumes should have the same hash and should be consistent
        """
        v1 = CompositeBoxVolume(
            [(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)), (kdl.Vector(0, 0, 0), kdl.Vector(-1, -1, -1))]
        )
        v2 = deepcopy(v1)
        self.assertEqual(hash(v1), hash(v2))
        self.assertEqual(-5073307082080589318, hash(v1))

        v3 = CompositeBoxVolume(
            [(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)), (kdl.Vector(0, 0, 0), kdl.Vector(2, 2, 2))]
        )
        self.assertNotEqual(hash(v1), hash(v3))

    def test_offset_volume_hash(self):
        """
        Two offset volumes should have the same hash and should be consistent
        """
        v1 = OffsetVolume(1.0)
        v2 = deepcopy(v1)
        self.assertEqual(hash(v1), hash(v2))
        self.assertEqual(-6644214454873602895, hash(v1))
        v3 = OffsetVolume(2.0)
        self.assertNotEqual(hash(v1), hash(v3))


if __name__ == "__main__":
    unittest.main()
