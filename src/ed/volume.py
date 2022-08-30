from typing import Dict, Iterable, List, Optional, Tuple

# ROS
import rospy
import PyKDL as kdl
from numpy import abs

from ed_msgs.msg import Volume as volume_msg

from .util.equal_hash_mixin import EqualHashMixin


class Volume(EqualHashMixin):
    """
    Represents a volume of an entity

    Points are defined relative to the object they belong to
    """

    def __init__(self):
        """Constructor"""
        pass

    @property
    def center_point(self) -> kdl.Vector:
        """Get the center of the Volume"""
        return self._calc_center_point()

    def _calc_center_point(self) -> kdl.Vector:
        raise NotImplementedError(
            "_calc_center_point must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    def contains(self, point: kdl.Vector, padding: float = 0) -> bool:
        """
        Checks if the point is inside this volume

        :param point: kdl Vector w.r.t. the same frame as this volume
        :param padding: Padding to take into account. Positive values make the volume bigger, negative values smaller.
        :return: True if inside, False otherwise
        """
        raise NotImplementedError(
            "contains must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def size(self) -> float:
        return self._calc_size()

    def _calc_size(self) -> float:
        raise NotImplementedError(
            "_calc_size must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )


class BoxVolume(Volume):
    """Represents a box shaped volume"""

    def __init__(self, min_corner: kdl.Vector, max_corner: kdl.Vector):
        """
        Constructor

        Points are defined relative to the object they belong to

        :param min_corner: Vector with the minimum bounding box corner
        :param max_corner: Vector with the maximum bounding box corner
        """
        super(BoxVolume, self).__init__()

        assert isinstance(min_corner, kdl.Vector)
        assert isinstance(max_corner, kdl.Vector)

        self._min_corner = min_corner
        self._max_corner = max_corner

    def _calc_center_point(self) -> kdl.Vector:
        """
        Calculate where the center of the box is located

        >>> b = BoxVolume(kdl.Vector(0,0,0), kdl.Vector(1,1,1))
        >>> b.center_point
        [         0.5,         0.5,         0.5]
        """
        return kdl.Vector(
            0.5 * (self._min_corner.x() + self._max_corner.x()),
            0.5 * (self._min_corner.y() + self._max_corner.y()),
            0.5 * (self._min_corner.z() + self._max_corner.z()),
        )

    def _calc_size(self) -> float:
        """
        Calculate the size of a volume

        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)).size
        1.0
        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(10, 10, 0.1)).size
        10.0
        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 10)).size
        10.0
        """
        size_x = abs(self._max_corner.x() - self._min_corner.x())
        size_y = abs(self._max_corner.y() - self._min_corner.y())
        size_z = abs(self._max_corner.z() - self._min_corner.z())
        return size_x * size_y * size_z

    @property
    def min_corner(self) -> kdl.Vector:
        return self._min_corner

    @property
    def max_corner(self) -> kdl.Vector:
        return self._max_corner

    @property
    def bottom_area(self) -> List[kdl.Vector]:
        convex_hull = []
        convex_hull.append(kdl.Vector(self.min_corner.x(), self.min_corner.y(), self.min_corner.z()))  # 1
        convex_hull.append(kdl.Vector(self.max_corner.x(), self.min_corner.y(), self.min_corner.z()))  # 2
        convex_hull.append(kdl.Vector(self.max_corner.x(), self.max_corner.y(), self.min_corner.z()))  # 3
        convex_hull.append(kdl.Vector(self.min_corner.x(), self.max_corner.y(), self.min_corner.z()))  # 4
        return convex_hull

    def contains(self, point: kdl.Vector, padding: float = 0) -> bool:
        """
        Checks if the point is inside this volume

        >>> b = BoxVolume(kdl.Vector(0,0,0), kdl.Vector(1,1,1))
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1))
        True
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1), padding=0.2)
        True
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1), padding=-0.2)
        False
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1))
        False
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1), padding=0.2)
        True
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1), padding=-0.2)
        False

        :param point: Vector w.r.t. the same frame as this volume
        :param padding: Padding to take into account. Positive values make the volume bigger, negative values smaller.
        :return: True if inside, False otherwise
        """
        return (
            self._min_corner.x() - padding <= point.x() <= self._max_corner.x() + padding
            and self._min_corner.y() - padding <= point.y() <= self._max_corner.y() + padding
            and self._min_corner.z() - padding <= point.z() <= self._max_corner.z() + padding
        )

    def __repr__(self):
        return "BoxVolume(min_corner={}, max_corner={})".format(self.min_corner, self.max_corner)


class CompositeBoxVolume(Volume):
    """Represents a composite box shaped volume"""

    def __init__(self, boxes: Iterable[Tuple[kdl.Vector, kdl.Vector]]):
        """
        Constructor

        Points are defined relative to the object they belong to.

        :param boxes: list of tuples of two vectors. First one with the minimum bounding box corners,
                            second one with the maximum bounding box corners
        """
        super(CompositeBoxVolume, self).__init__()

        assert isinstance(boxes, list)
        assert len(boxes) > 0
        assert isinstance(boxes[0], tuple)

        zipped_corners = list(zip(*boxes))
        self._min_corners = zipped_corners[0]
        self._max_corners = zipped_corners[1]

    def _calc_center_point(self) -> kdl.Vector:
        """
        Calculate where the center of the box is located

        >>> b = CompositeBoxVolume([(kdl.Vector(0,0,0), kdl.Vector(1,1,1))])
        >>> b.center_point
        [         0.5,         0.5,         0.5]
        """
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        max_z = max([v.z() for v in self._max_corners])
        return kdl.Vector(0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.5 * (min_z + max_z))

    @property
    def min_corner(self) -> kdl.Vector:
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        return kdl.Vector(min_x, min_y, min_z)

    @property
    def max_corner(self) -> kdl.Vector:
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        max_z = max([v.z() for v in self._max_corners])
        return kdl.Vector(max_x, max_y, max_z)

    @property
    def bottom_area(self) -> List[kdl.Vector]:
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        convex_hull = [
            kdl.Vector(min_x, min_y, min_z),
            kdl.Vector(max_x, min_y, min_z),
            kdl.Vector(max_x, max_y, min_z),
            kdl.Vector(min_x, max_y, min_z),
        ]
        return convex_hull

    def contains(self, point: kdl.Vector, padding: float = 0) -> bool:
        """
        Checks if the point is inside this volume

        >>> b = CompositeBoxVolume([(kdl.Vector(0,0,0), kdl.Vector(1,1,1))])
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1))
        True
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1), padding=0.2)
        True
        >>> b.contains(kdl.Vector(0.1, 0.1, 0.1), padding=-0.2)
        False
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1))
        False
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1), padding=0.2)
        True
        >>> b.contains(kdl.Vector(-0.1, -0.1, -0.1), padding=-0.2)
        False

        :param point: Vector w.r.t. the same frame as this volume
        :param padding: Padding to take into account. Positive values make the volume bigger, negative values smaller.
        :return: True if inside, False otherwise
        """
        for min_corner, max_corner in zip(self._min_corners, self._max_corners):
            if (
                min_corner.x() - padding <= point.x() <= max_corner.x() + padding
                and min_corner.y() - padding <= point.y() <= max_corner.y() + padding
                and min_corner.z() - padding <= point.z() <= max_corner.z() + padding
            ):
                return True

        return False

    def __repr__(self):
        description = "CompositeBoxVolume:\n"
        for min_corner, max_corner in zip(self._min_corners, self._max_corners):
            description += "min_corner={}, max_corner={}\n".format(min_corner, max_corner)

        return description


class OffsetVolume(Volume):
    """Represents a volume with a certain offset from the convex hull of the entity"""

    def __init__(self, offset):
        """Constructor

        :param offset: Offset [m]
        """
        super(OffsetVolume, self).__init__()
        self._offset = offset

    def __repr__(self):
        return "OffsetVolume(offset={})".format(self._offset)


def volume_from_entity_volume_msg(msg: volume_msg) -> Tuple[Optional[str], Optional[Volume]]:
    """
    Creates a dict mapping strings to Volumes from the EntityInfo data dictionary

    :param msg: ed_msgs.msg.Volume
    :return: tuple of name and volume object
    """
    # Check if we have data and if it contains volumes
    if not msg:
        return None, None

    # Check if the volume has a name. Otherwise: skip
    if not msg.name:
        return None, None
    name = msg.name

    # Check if we have a shape
    if len(msg.subvolumes) == 1:
        subvolume = msg.subvolumes[0]
        if not subvolume.geometry.type == subvolume.geometry.BOX:
            return None, None

        p = subvolume.center_point.point
        center_point = kdl.Vector(p.x, p.y, p.z)

        size = subvolume.geometry.dimensions
        size = kdl.Vector(size[0], size[1], size[2])

        min_corner = center_point - size / 2
        max_corner = center_point + size / 2

        return name, BoxVolume(min_corner, max_corner)
    else:
        min_corners = []
        max_corners = []
        for subvolume in msg.subvolumes:
            if not subvolume.geometry.type == subvolume.geometry.BOX:
                continue

            size = subvolume.geometry.dimensions
            size = kdl.Vector(size[0], size[1], size[2])

            p = subvolume.center_point.point
            center_point = kdl.Vector(p.x(), p.y(), p.z())

            sub_min = center_point - size / 2
            sub_max = center_point + size / 2
            min_corners.append(sub_min)
            max_corners.append(sub_max)

        return name, CompositeBoxVolume(zip(min_corners, max_corners))


def volumes_from_entity_volumes_msg(msgs: Iterable[volume_msg]) -> Dict[str, Volume]:
    if not msgs:
        return {}

    volumes = {}
    for v in msgs:
        if not v.name:
            continue

        name, volume = volume_from_entity_volume_msg(v)
        if name and volume:
            volumes[name] = volume

    return volumes


if __name__ == "__main__":
    import doctest

    doctest.testmod()
