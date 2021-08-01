from typing import Union

import PyKDL as kdl
from geometry_msgs.msg import Point
import rospy
from pykdl_ros import VectorStamped
import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from ed_msgs.srv import Configure, SimpleQuery, SimpleQueryRequest, UpdateSrv, Reset

from .entity import from_entity_info


class WM:
    def __init__(self, robot_name, tf_buffer=None):
        if tf_buffer is None:
            self.tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = tf_buffer

        self.__ros_connections = {}

        self._ed_simple_query_srv = self.create_service_client(f"/{robot_name}/ed/simple_query", SimpleQuery)
        self._ed_update_srv = self.create_service_client(f"/{robot_name}/ed/update", UpdateSrv)

        self._ed_configure_srv = self.create_service_client(f"/{robot_name}/ed/configure", Configure)
        self._ed_reset_srv = self.create_service_client(f"/{robot_name}/ed/reset", Reset)

        self.robot_name = robot_name

    def create_service_client(self, name: str, srv_type):
        """
        Creates a service client

        :param name: string with the name of the service in the correct namespace
        :param srv_type: service type
        :return: the service client
        """
        srv = rospy.ServiceProxy(name, srv_type)
        self._add_connection(name, srv)
        return srv

    def _add_connection(self, name: str, connection: Union[rospy.Subscriber, rospy.ServiceProxy]):
        """
        Adds a connection to the internal dict with connections that is used when initializing the robot object.

        :param name: name of the connection
        :param connection: connection to add. This might be a ServiceProxy, ActionClient or Subscriber
        """
        self.__ros_connections[name] = connection

    def get_entities(
        self,
        etype="",
        center_point=VectorStamped(kdl.Vector(), rospy.Time.now(), "map"),
        radius=float("inf"),
        uuid="",
        ignore_z=False,
    ):
        """
        Get entities via Simple Query interface

        :param etype: Type of entity
        :param center_point: Point from which radius is measured
        :param radius: Distance between center_point and entity
        :param uuid: uuid of entity
        :param ignore_z: Consider only the distance in the X,Y plane for the radius from center_point
        """
        center_point_in_map = self.tf_buffer.transform(center_point, "map")
        query = SimpleQueryRequest(
            uuid=uuid,
            type=etype,
            center_point=tf2_ros.convert(center_point_in_map, Point),
            radius=radius,
            ignore_z=ignore_z,
        )

        try:
            entity_infos = self._ed_simple_query_srv(query).entities
            entities = list(map(from_entity_info, entity_infos))
        except Exception as e:
            rospy.logerr(
                f"get_entities(uuid={uuid}, type={etype}, center_point={center_point}, radius={radius}, "
                f"ignore_z={ignore_z})\n{e}"
            )
            return []

        return entities

    def get_closest_entity(self, etype="", center_point=None, radius=float("inf")):
        if not center_point:
            center_point = VectorStamped(kdl.Vector(), stamp=rospy.Time.now(), frame_id=self.robot_name + "/base_link")

        entities = self.get_entities(etype=etype, center_point=center_point, radius=radius)

        # HACK
        entities = [e for e in entities if e.shape is not None and e.etype != ""]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            center_in_map = self.tf_buffer.transform(center_point, "map")
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_in_map.vector))
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_closest_room(self, center_point=None, radius=float("inf")):
        if not center_point:
            center_point = VectorStamped(
                kdl.Vector(x=0, y=0, z=0), stamp=rospy.Time.now(), frame_id=self.robot_name + "/base_link"
            )

        return self.get_closest_entity(etype="room", center_point=center_point, radius=radius)

    def get_entity(self, uuid):
        entities = self.get_entities(uuid=uuid)
        if len(entities) == 0:
            rospy.logerr(f'Could not get_entity(uuid="{uuid}")')
            return None

        return entities[0]

    def update_entity(
        self, uuid, etype=None, frame_stamped=None, flags=None, add_flags=None, remove_flags=None, action=None
    ):
        """
        Updates entity

        :param uuid: entity uuid
        :param etype: entity type
        :param frame_stamped: If specified, the entity is updated to be at this FrameStamped
        :param flags: (OBSOLETE, use add_flags and remove_flags): (list of) dict(s) containing key "add" or "remove" and value of the flag to set,  e.g., "perception"
        :param add_flags: list of flags which will be added to the specified entity
        :param remove_flags: list of flags which will removed from the specified entity
        :param action: update_action, e.g. remove
        """
        if add_flags is None:
            add_flags = []
        if remove_flags is None:
            remove_flags = []
        json_entity = f'"id" : "{uuid}"'
        if etype:
            json_entity += f', "type": "{etype}"'

        if action:
            json_entity += f', "action": "{action}"'

        if frame_stamped:
            if frame_stamped.frame_uuid != "/map" and frame_stamped.frame_uuid != "map":
                rospy.loginfo("update_entity: frame not in map, transforming")
                frame_stamped = self.tf_buffer.transform(frame_stamped, "map")

            Z, Y, X = frame_stamped.frame.M.GetEulerZYX()
            t = frame_stamped.frame.p
            json_entity += f', "pose": {{ "x": {t.x()}, "y": {t.y()}, "z": {t.z()}, "X": {X}, "Y": {Y}, "Z": {Z} }}'

        if flags or add_flags or remove_flags:
            json_entity += ', "flags": ['
            first = True

            if isinstance(flags, dict):
                flags = [flags]

            if isinstance(flags, list):
                for flag in flags:
                    if not isinstance(flag, dict):
                        rospy.logerr("update_entity: flags need to be a list of dicts or a dict")
                        return False
                    for k, v in flag.items():
                        if not first:
                            json_entity += ","
                        json_entity += f'{{"{k}": "{v}"}}'
                        first = False

            for flag in add_flags:
                if not first:
                    json_entity += ","
                json_entity += f'{{"add":"{flag}"}}'
                first = False

            for flag in remove_flags:
                if not first:
                    json_entity += ","
                json_entity += f'{{"remove":"{flag}"}}'
                first = False

            json_entity += "]"

        json = f'{{"entities":[{{ {json_entity} }}]}}'
        rospy.logdebug(json)

        return self._ed_update_srv(request=json)

    def remove_entity(self, uuid):
        """
        Removes entity with the provided uuid to the world model

        :param uuid: string with the uuid of the entity to remove
        """
        return self.update_entity(uuid=uuid, action="remove")

    def lock_entities(self, lock_ids, unlock_ids):
        for uuid in lock_ids:
            self.update_entity(uuid=uuid, add_flags=["locked"])

        for uuid in unlock_ids:
            self.update_entity(uuid=uuid, remove_flags=["locked"])

    def get_closest_possible_person_entity(
        self, center_point=VectorStamped(kdl.Vector(), rospy.Time.now(), "map"), radius=float("inf")
    ):
        """
        Returns the "possible_human" entity closest to a certain center point.

        :param center_point: (VectorStamped) indicating where the human should be close to
        :param radius: (float) radius to look for possible humans
        :return: (Entity) entity (if found), None otherwise
        """
        assert center_point.frame_uuid.endswith("map"), "Other frame uuids not yet implemented"

        # Get all entities
        entities = self.get_entities(etype="", center_point=center_point, radius=radius)

        # Filter on "possible humans"
        entities = [e for e in entities if e.is_a("possible_human")]

        # If nothing found, return None
        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_point.vector))
            rospy.logdebug(f"entities sorted closest to robot = {entities}")
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_full_uuid(self, short_uuid):
        """Get an entity"s full uuid based on the first characters of its uuid like you can do with git hashes"""
        all_entities = self.get_entities()
        matches = filter(lambda fill_uuid: fill_uuid.startswith(short_uuid), [entity.uuid for entity in all_entities])
        return matches