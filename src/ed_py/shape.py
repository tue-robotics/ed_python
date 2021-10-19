from typing import List

from ed_msgs.msg import EntityInfo
import PyKDL as kdl


class Shape:
    """Represents shape properties"""

    def __init__(self):
        pass

    @property
    def convex_hull(self) -> List[kdl.Vector]:
        return self._calc_convex_hull()

    def _calc_convex_hull(self):
        raise NotImplementedError(
            "_calc_convex_hull must be implemented by subclasses. "
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

    @property
    def x_min(self) -> float:
        return self._calc_x_min()

    def _calc_x_min(self) -> float:
        raise NotImplementedError(
            "_calc_x_min must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def x_max(self) -> float:
        return self._calc_x_max()

    def _calc_x_max(self) -> float:
        raise NotImplementedError(
            "_calc_x_max must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def y_min(self) -> float:
        return self._calc_y_min()

    def _calc_y_min(self) -> float:
        raise NotImplementedError(
            "_calc_y_min must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def y_max(self) -> float:
        return self._calc_y_max()

    def _calc_y_max(self) -> float:
        raise NotImplementedError(
            "_calc_y_max must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def z_min(self) -> float:
        return self._calc_z_min()

    def _calc_z_min(self) -> float:
        raise NotImplementedError(
            "_calc_z_min must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )

    @property
    def z_max(self) -> float:
        return self._calc_z_max()

    def _calc_z_max(self) -> float:
        raise NotImplementedError(
            "_calc_z_max must be implemented by subclasses. "
            "Class {cls} has no implementation".format(cls=self.__class__.__name__)
        )


class RightPrism(Shape):
    """
    Represents a right prism, i.e., a  prism in which the joining edges and faces are perpendicular to the base
    faces. This is typical for the shapes resulting from the convex hull, z min and z max of EntityInfo
    """

    def __init__(self, convex_hull: List[kdl.Vector], z_min: float, z_max: float):
        """
        Constructor

        :param convex_hull: list with vectors representing the vertices of the convex hull. N.B.: these should only
            contain x and y values, the z-values are not relevant (the height information is contained in the z_min
            and z_max parameters).
        :param z_min: Minimum height [m] w.r.t. the center of the corresponding Entity
        :param z_max: Maximum height [m] w.r.t. the center of the corresponding Entity
        """
        super(RightPrism, self).__init__()
        self._convex_hull = convex_hull
        self._z_min = z_min
        self._z_max = z_max

    def _calc_x_max(self) -> float:
        return max(ch.x() for ch in self._convex_hull)

    def _calc_x_min(self) -> float:
        return min(ch.x() for ch in self._convex_hull)

    def _calc_y_max(self) -> float:
        return max(ch.y() for ch in self._convex_hull)

    def _calc_y_min(self) -> float:
        return min(ch.y() for ch in self._convex_hull)

    def _calc_z_max(self) -> float:
        return self._z_max

    def _calc_z_min(self) -> float:
        return self._z_min

    def _calc_convex_hull(self) -> list[kdl.Vector]:
        return self._convex_hull

    def _calc_size(self) -> float:
        """
        Calculate the rough size of a shape
        >>> RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 1, 0), kdl.Vector(1, 1, 0), kdl.Vector(1, 0, 0)], z_min=0, z_max=1).size
        1.0
        >>> RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 2, 0), kdl.Vector(2, 2, 0), kdl.Vector(2, 0, 0)], z_min=0, z_max=2).size
        8.0
        """

        # TODO: this only uses the bounding volume's size, which might not be accurate
        size_x = abs(self.x_max - self.x_min)
        size_y = abs(self.y_max - self.y_min)
        size_z = abs(self.z_max - self.z_min)
        return size_x * size_y * size_z


def shape_from_entity_info(e: EntityInfo) -> Shape:
    """
    Creates a shape from the convex_hull, z_min and z_max of the EntityInfo object. If no convex hull is present,
    an empty Shape object is returned

    :param e: ed_msgs.msg.EntityInfo
    :return: Shape
    """
    # Check if we have a convex hull
    if not e.convex_hull:
        return Shape()

    return RightPrism(convex_hull=[kdl.Vector(p.x, p.y, p.z) for p in e.convex_hull], z_min=e.z_min, z_max=e.z_max)


if __name__ == "__main__":
    import doctest

    doctest.testmod()
