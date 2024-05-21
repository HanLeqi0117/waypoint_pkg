import rclpy.node
from waypoint_pkg.waypoint_utils.utils import (
    Waypoint, PointStamped, rclpy,
    Node, os, ruamel, PolygonStamped,
    WaypointMode, get_yaw_with_geos, sys
)

class GeoTopologicalMap(Node):
    
    def __init__(self):
        super().__init__("geo_topological_map")
        
        self._path_ = self.declare_parameter(
            name="path", 
            value=os.path.join(os.environ["HOME"], "Documents", "python_test.yaml")
        ).get_parameter_value().string_value
        self._data_class_name_ = self.declare_parameter(
            name="data_class_name",
            value="topology"
        ).get_parameter_value().string_value
        self._polygon_sub_ = self.create_subscription(PolygonStamped, "topological_map", self.topo_sub_callback, 1)
        
        self.get_logger().info("Ready to recieve topological information!!!")
    
    def topo_sub_callback(self, msg : PolygonStamped):
        waypoints : Waypoint.waypoints
        key_name = self._data_class_name_
        waypoints = Waypoint.get_initial_waypoints(key_name)
        
        if msg.header.frame_id != "wgs84":
            self.get_logger().warn("The Frame ID is not wgs84. The GeoPoint is needed here!!!!")
            return

        for point in msg.polygon.points:            
            size = len(waypoints[key_name])
            if size == 0:
                waypoint = Waypoint.get_initial_waypoint()
                waypoint["mode"] = WaypointMode.NORMAL
                waypoint["latitude"] = point.y
                waypoint["longitude"] = point.x
                
                waypoints[key_name].append(waypoint)
            else:
                waypoint = Waypoint.get_initial_waypoint()
                waypoint["mode"] = WaypointMode.NORMAL
                waypoint["latitude"] = point.y
                waypoint["longitude"] = point.x
                
                waypoints[key_name][size - 1]["yaw"] = get_yaw_with_geos(
                    waypoints[key_name][size - 1]["latitude"],
                    waypoints[key_name][size - 1]["longitude"],
                    point.y,
                    point.x
                ) - 90
                waypoints[key_name].append(waypoint)
        
        with open(self._path_, "w+") as f_operator:
            ruamel.yaml.safe_dump(data=waypoints, stream=f_operator)
            self.get_logger().info("Topological Map is saved into the path given!")
    
        waypoints.clear()


def main():
    rclpy.init(args=sys.argv)
    geo_topological_map = GeoTopologicalMap()
    rclpy.spin(geo_topological_map)
    rclpy.shutdown()

if __name__ == "__main__":
    main()