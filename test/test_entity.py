from copy import deepcopy
import unittest

import PyKDL as kdl
import rospy

from ed.entity import Entity


class TestShape(unittest.TestCase):
    def test_entity_equal(self):
        """
        Two empty entities should be equal
        """
        pose = kdl.Frame.Identity()
        e1 = Entity("dummy", None, "map", pose, None, {}, None, rospy.Time())
        e2 = deepcopy(e1)
        self.assertEqual(e1, e2)
        e3 = deepcopy(e1)
        e3._pose = kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3))
        self.assertNotEqual(e1, e3)

    def test_entity_hash(self):
        """
        Two empty entities should have the same hash and should be consistent
        """
        pose = kdl.Frame.Identity()
        e1 = Entity("dummy", None, "map", pose, None, {}, None, rospy.Time())
        e2 = deepcopy(e1)
        self.assertEqual(hash(e1), hash(e2))
        # self.assertEqual(3511217657577979381, hash(e1))  # Strings don't have a persistent hash
        e3 = deepcopy(e1)
        e3._pose = kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3))
        self.assertNotEqual(hash(e1), hash(e3))


if __name__ == "__main__":
    unittest.main()
