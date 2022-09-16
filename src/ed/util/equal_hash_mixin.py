from collections import Hashable, Mapping

import rospy


class EqualHashMixin:
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return all(v1 == v2 for v1, v2 in zip(self.__dict__.values(), other.__dict__.values()))
        else:
            return False

    def __hash__(self):
        attrs = []
        for item in self.__dict__.values():
            if isinstance(item, Mapping):
                attrs.append(tuple(item.items()))
            elif isinstance(item, list):
                attrs.append(tuple(item))
            elif isinstance(item, Hashable):
                attrs.append(item)
            else:
                rospy.logerr(f"Not able to hash type: {type(item)}")
        return hash(tuple(attrs))
