from __future__ import annotations

from typing import List, Mapping, Optional

import yaml

from ed_msgs.msg import EntityInfo
from genpy import Time
from geometry_msgs.msg import PoseStamped
import PyKDL as kdl
from pykdl_ros import FrameStamped, VectorStamped
import rospy

from std_msgs.msg import Header

import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from .shape import shape_from_entity_info, Shape
from .volume import Volume, volumes_from_entity_volumes_msg

from .util.equal_hash_mixin import EqualHashMixin


class Entity(EqualHashMixin):
    """Holds all data concerning entities"""

    def __init__(
        self,
        identifier: str,
        object_type: str,
        frame_id: str,
        pose: FrameStamped,
        shape: Shape,
        volumes: Mapping[str, Volume],
        super_types: List[str],
        last_update_time: Time,
        person_properties=None,
    ):
        """
        Constructor

        :param identifier: str with the id of this entity
        :param object_type: str with the type of this entity
        :param frame_id: str frame id w.r.t. which the pose is defined
        :param pose: kdl.Frame with the pose of this entity
        :param shape: Shape of this entity
        :param volumes: dict mapping strings to Volume
        :param super_types: list with strings representing super types in an ontology of object types
        :param last_update_time: Time of last update
        """
        self.uuid = identifier
        self.etype = object_type
        self.frame_id = frame_id
        self._pose = pose
        self.shape = shape
        self._volumes = volumes if volumes else {}
        self.super_types = super_types if super_types else []
        self._last_update_time = last_update_time

        self._person_properties = person_properties

    @property
    def volumes(self):
        return self._volumes

    def in_volume(self, point: VectorStamped, volume_id: str, padding: float = 0) -> bool:
        """
        Checks if the point is in the volume identified by the volume id

        :param point: VectorStamped with the point to check
        :param volume_id: string with the volume
        :param padding: Padding to take into account. Positive values make the volume bigger, negative values smaller.
        :return: boolean indicating whether the point is in the designated volume. If an error occurs, False is returned
        """
        # Check if the volume exists
        if volume_id not in self._volumes:
            rospy.logdebug("{} not a volume of {}".format(volume_id, self.uuid))
            return False

        # Transform the point
        fid1 = (
            point.header.frame_id if point.header.frame_id[0] != "/" else point.header.frame_id[1:]
        )  # Remove slash for comparison
        fid2 = self.frame_id if self.frame_id[0] != "/" else self.frame_id[1:]  # Remove slash for comparison
        if fid1 != fid2:
            rospy.logerr(
                f"Cannot compute with volume and entity defined w.r.t. different frame: {point.header.frame_id} and {self.frame_id}"
            )
            return False
        vector = self.pose.frame.Inverse() * point.vector

        # Check if the point is inside of the volume
        return self._volumes[volume_id].contains(vector, padding)

    def entities_in_volume(self, entities: List[Entity], volume_id: str) -> List[Entity]:
        """
        Filter the collection of entities down to only those in the given volume

        :param entities: collection/sequence of entities
        :param volume_id: volume these entities need to be in
        :return: entities that are both in the given volume and in the list 'entities'
        """
        entities = [e for e in entities if self.in_volume(VectorStamped.from_framestamped(e.pose), volume_id)]

        return entities

    @property
    def last_update_time(self):
        return self._last_update_time

    def distance_to_2d(self, point: kdl.Vector) -> float:
        """
        Calculate the distance between this entity's pose and the given point.

        :param point: Assumed to be in the same frame_id as the entity itself
        :return: the distance between the entity's pose and the point

        >>> pose = kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3))
        >>> e = Entity("dummy", None, "map", pose, None, {}, None, rospy.Time())
        >>> point = kdl.Vector(1, 1, 1)
        >>> e.distance_to_2d(point)
        2.8284271247461903
        """

        # The length of the difference vector between the pose's position and the point
        difference = self._pose.p - point
        difference.z(0)
        return difference.Norm()

    def distance_to_3d(self, point: kdl.Vector) -> kdl.Vector:
        """
        Calculate the distance between this entity's pose and the given point.

        :param point: Assumed to be in the same frame_id as the entity itself
        :return: the distance between the entity's pose and the point

        >>> pose = kdl.Frame(kdl.Rotation.RPY(1, 0, 0), kdl.Vector(3, 3, 3))
        >>> e = Entity("dummy", None, "map", pose, None, {}, None, rospy.Time())
        >>> point = kdl.Vector(1, 1, 1)
        >>> e.distance_to_3d(point)
        3.4641016151377544
        """
        return (self._pose.p - point).Norm()

    def is_a(self, super_type: str) -> bool:
        """
        Check whether the entity is a (subclass of) some supertype

        :param super_type: Representing the name of the super_type
        :return: True if the entity is a (sub)type of the given super_type

        >>> e = Entity("dummy", "coffee_table", None, None, None, {}, ["coffee_table", "table", "furniture", "thing"], 0)
        >>> e.is_a("furniture")
        True
        >>> e.is_a("food")
        False
        """
        return super_type in self.super_types

    @property
    def pose(self) -> FrameStamped:
        """Returns the pose of the Entity as a FrameStamped"""
        return FrameStamped(self._pose, self.last_update_time, self.frame_id)

    @pose.setter
    def pose(self, pose: FrameStamped):
        """Setter"""
        self._pose = tf2_ros.convert(pose, FrameStamped).frame

    @property
    def person_properties(self) -> Optional[PersonProperties]:
        if self._person_properties:
            return self._person_properties
        else:
            rospy.logwarn("{} is not a person".format(self))
            return None

    @person_properties.setter
    def person_properties(self, value: PersonProperties):
        self._person_properties = value

    def __repr__(self):
        return "Entity(uuid='{uuid}', etype='{etype}', frame={frame}, person_properties={pp})".format(
            uuid=self.uuid, etype=self.etype, frame=self.pose, pp=self._person_properties
        )


class PersonProperties(object):
    def __init__(
        self,
        name: str,
        age: int,
        emotion: str,
        gender: str,
        gender_confidence: float,
        pointing_pose,
        posture: str,
        reliability,
        shirt_colors: List[str],
        tags,
        tagnames,
        velocity,
        parent_entity,
    ):
        # ToDo: "In the legacy message definition a field named tagnames was defined but never used. Only tags is used.
        #  Once the message definition in people_recognition is cleaned up then tagnames should be removed completely."
        """
        Container for several properties related to a person

        :param name: the person's name. This is separate from the entity, which is unique while this doesn't have to be
        :param age: Estimated age of the person
        :param emotion: Indicating the emotion
        :param gender: Predicted gender of the person
        :param gender_confidence: Confidence of the classifier in the gender above.
        :param pointing_pose: In which direction is the person pointing
        :param posture: String with a value like 'sitting', 'laying', 'standing' etc.
        :param reliability:  ToDO
        :param shirt_colors: list of 3 shirt colors, sorted from most dominant to less dominant
        :param tags: Other tags
        :param tagnames: Other tagnames
        :param velocity: Velocity with which the person in moving
        :param parent_entity: The Entity that these properties belong to
        """
        self._name = name
        self.age = age
        self.emotion = emotion
        self.gender = gender
        self.gender_confidence = gender_confidence
        self.pointing_pose = pointing_pose
        self.posture = posture
        self.reliability = reliability
        self.shirt_colors = shirt_colors
        self.tags = tags
        self.tagnames = tagnames
        self.velocity = velocity

        self._parent_entity = parent_entity

    @property
    def name(self) -> str:
        return self._name

    @name.setter
    def name(self, value: str):
        rospy.loginfo("Changing {}'s name to {}".format(self._parent_entity.id, value))
        self._name = value

    def __repr__(self):
        return (
            f"PersonProperties("
            f"age='{self.age}', "
            f"emotion='{self.emotion}', "
            f"gender='{self.gender}', "
            f"gender_confidence={self.gender_confidence}, "
            f"pointing_pose={self.pointing_pose}, "
            f"posture={self.posture}, "
            f"reliability={self.reliability}, "
            f"shirt_colors={self.shirt_colors}, "
            f"tags={self.tags}, "
            f"tagnames={self.tagnames}, "
            f"velocity={self.velocity}"
            f")"
        )


def from_entity_info(e: EntityInfo) -> Entity:
    """
    Converts ed_msgs.msg.EntityInfo to an Entity

    :param e: ed_msgs.msg.EntityInfo
    :return: Entity
    """
    assert isinstance(e, EntityInfo)
    identifier = e.id
    object_type = e.type
    frame_id = "map"  # ED has all poses in map
    # ToDo: Change pose in msg definition to PoseStamped
    pose_stamped = PoseStamped(pose=e.pose, header=Header(stamp=rospy.Time(), frame_id=frame_id))
    pose = tf2_ros.convert(pose_stamped, FrameStamped).frame
    shape = shape_from_entity_info(e)

    last_update_time = e.last_update_time

    # The data is a string but can be parsed as yaml, which then represent is a much more usable data structure
    volumes = volumes_from_entity_volumes_msg(e.volumes)
    rospy.logdebug(f"Entity(id={identifier}) has volumes {volumes.keys()} ")

    super_types = e.types

    # ToDo: Hacky logic from robot_skills
    if (
        e.has_shape
        and not any([name in e.id for name in ["amigo", "sergio", "hero"]])
        and e.id != "floor"
        and "wall" not in e.id
    ):
        super_types += ["furniture"]

    # ToDo: Hacky logic from robot_skills
    if all([v in volumes for v in ["handle", "frame_left_point", "frame_right_point"]]):
        super_types += ["door"]
        super_types.remove("furniture")

    if "possible_human" in e.flags:
        super_types += ["possible_human"]

    entity = Entity(
        identifier=identifier,
        object_type=object_type,
        frame_id=frame_id,
        pose=pose,
        shape=shape,
        volumes=volumes,
        super_types=super_types,
        last_update_time=last_update_time,
    )

    if e.type == "person":
        try:
            pp_dict = yaml.load(e.data, yaml.SafeLoader)
            del pp_dict["position"]
            del pp_dict["header"]
            entity.person_properties = PersonProperties(parent_entity=entity, **pp_dict)
        except TypeError as te:
            rospy.logerr(f"Cannot instantiate PersonProperties from {e.data}: {te}")

    return entity


if __name__ == "__main__":
    import doctest

    doctest.testmod()
