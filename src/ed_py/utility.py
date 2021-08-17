from typing import List

import rospy

from pykdl_ros import VectorStamped

from .entity import Entity
from .world_model import WM


def rooms_of_entity(wm: WM, entity: Entity) -> List[Entity]:
    """
    Determine all rooms that the entity is in
    """
    all_rooms = wm.get_entities(VectorStamped.from_xyz(0, 0, 0, rospy.Time(0), "map"),
                                etype="room")
    rooms = []
    entity_point = VectorStamped.from_framestamped(entity.pose)
    for room in all_rooms:
        if room.in_volume(entity_point, "in"):
            rooms.append(room)

    return rooms


def rooms_of_volume(wm: WM, entity: Entity, volume_id: str) -> List[Entity]:
    """
    Determine all the rooms a volume of an entity is in
    """
    all_rooms = wm.get_entities(VectorStamped.from_xyz(0, 0, 0, rospy.Time(0), "map"),
                                etype="room")
    rooms = []
    try:
        volume = entity.volumes[volume_id]
        volume_point = VectorStamped(volume.center_point, rospy.Time(0), entity.uuid)
        volume_point_in_map = wm.tf_buffer.transform(volume_point, "map")
    except KeyError as e:
        rospy.logerr(f"Entity({entity.uuid}) has not a volume with the name: {volume_id}\n{e}")
        return []

    for room in all_rooms:
        if room.in_volume(volume_point_in_map, "in"):
            rooms.append(room)

    return rooms
