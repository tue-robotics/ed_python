from copy import deepcopy
import unittest

import PyKDL as kdl

from ed.shape import Shape, RightPrism


class TestShape(unittest.TestCase):
    def test_shape_equal(self):
        """
        Two empty shapes should be equal
        """
        s1 = Shape()
        s2 = deepcopy(s1)
        self.assertEqual(s1, s2)

    def test_right_prism_equal(self):
        """
        Two right prism shapes should be equal
        """
        s1 = RightPrism(
            [kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Vector(1, 1, 0), kdl.Vector(1, 0, 0)], z_min=0, z_max=1
        )
        s2 = deepcopy(s1)
        self.assertEqual(s1, s2)

    def test_shape_hash(self):
        """
        Two empty shapes should have the same hash and should be consistent
        """
        s1 = Shape()
        s2 = deepcopy(s1)
        self.assertEqual(hash(s1), hash(s2))
        self.assertEqual(5740354900026072187, hash(s1))

    def test_right_prism_hash(self):
        """
        Two right prism shapes should have the same hash and should be consistent
        """
        s1 = RightPrism(
            [kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Vector(1, 1, 0), kdl.Vector(1, 0, 0)], z_min=0, z_max=1
        )
        s2 = deepcopy(s1)
        self.assertEqual(hash(s1), hash(s2))
        self.assertEqual(7375619813495047945, hash(s1))


if __name__ == "__main__":
    unittest.main()
