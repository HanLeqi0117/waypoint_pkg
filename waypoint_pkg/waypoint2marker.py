from waypoint_pkg.waypoint_utils.utils import (
    MarkerArray, rclpy, ruamel,
    Node, Waypoint, Marker, Pose,
    os, quaternion_from_euler
)
from whill_navi2.modules.ros2_launch_utils import DataPath

class Waypoint2Marker(Node):

    def __init__(self):
        super().__init__("waypoint_display")

        self._waypoint_read_file_ = self.declare_parameter(
            "waypoint_read_file", DataPath().get_rewapypoint_path()[0]
        ).get_parameter_value().string_value
        self._until_node_ = self.declare_parameter("until_node", "navigation_rviz").get_parameter_value().string_value

        self._marker_array_pub_ = self.create_publisher(MarkerArray, "waypoint_marker_array", 100)

        with open(self._waypoint_read_file_, "r") as f:
            self._waypoints_ = ruamel.yaml.safe_load(f)
        
        self.wait_for_node(self._until_node_, 3.0)

        self.main_process()

    def main_process(self):
        
        array_arrow = MarkerArray()
        array_text = MarkerArray()

        for index in range(len(self._waypoints_["waypoints"])):

            arrow = Marker()
            text = Marker()
            pose_tmp = Pose()

            arrow.header.frame_id = text.header.frame_id = "map"
            arrow.header.stamp = text.header.stamp = self.get_clock().now()
            arrow.ns = "WaypointArrow"
            text.ns = "Text"

            arrow.color.b = text.color.b = 1.0
            text.color.g = 1.0
            text.color.r = 1.0
            arrow.color.a = text.color.a = 1.0     
            arrow.scale.x = 1.0
            arrow.scale.y = 0.2
            arrow.scale.z = text.scale.z = 0.4
            arrow.action = text.action = Marker.ADD   
            arrow.type = Marker.ARROW
            text.type = Marker.TEXT_VIEW_FACING

            pose_tmp.position.x = self._waypoints_["waypoints"][index]["pos_x"]
            pose_tmp.position.y = self._waypoints_["waypoints"][index]["pos_y"]
            pose_tmp.position.z = 0.25

            quaternion = quaternion_from_euler(
                0.0, 0.0, self._waypoints_["waypoints"][index]["yaw"]
            )

            pose_tmp.orientation.x = quaternion[0]
            pose_tmp.orientation.y = quaternion[1]
            pose_tmp.orientation.z = quaternion[2]
            pose_tmp.orientation.w = quaternion[3]

            arrow.pose = text.pose = pose_tmp
            text.pose.position.z = 0.5
            arrow.id = text.id = index
            text.text = "No.{}-mode.{}".format(index, self._waypoints_["waypoints"][index]["mode"])

            if self._waypoints_["waypoints"][index]["mode"] == 0:
                arrow.color.r = 1.0
            else:
                arrow.color.r = 0.0

            array_arrow.markers.append(arrow)
            array_text.markers.append(text)
        
        self._marker_array_pub_.publish(array_arrow)
        self._marker_array_pub_.publish(array_text)
    
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Waypoint2Marker())
    rclpy.shutdown()
