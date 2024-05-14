from waypoint_pkg.waypoint_utils.utils import (
    Node, rclpy, get_geolocation, NavSatFix,
    sys
)
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time

class Geocoding(Node):
    def __init__(self):
        super().__init__("geocoding_node")
        
        self._place_name_ = self.declare_parameter(
            name="place_name",
            value="Kansai University"
        ).get_parameter_value().string_value
        self._map_frame_ = self.declare_parameter(
            name="map_frame",
            value="map"
        ).get_parameter_value().string_value
        self._tile_map_frame_ = self.declare_parameter(
            name="tile_map_frame",
            value="origin"
        ).get_parameter_value().string_value
        self._node_name_waiting_for_ = self.declare_parameter(
            name="node_name_waiting_for",
            value="initialize_origin"
        ).get_parameter_value().string_value
        
        self._tf2_buffer_ = Buffer()
        self._tf2_listener_ = TransformListener(self._tf2_buffer_, self)
        
        tf2_futrue = self._tf2_buffer_.wait_for_transform_async(self._map_frame_, self._tile_map_frame_, Time(seconds=0, nanoseconds=5 * 10**8))
        rclpy.spin_until_future_complete(self, tf2_futrue)
        self.get_logger().info("Transform [{} -> {}] is OK!".format(self._tile_map_frame_, self._map_frame_))

        while not self.wait_for_node(self._node_name_waiting_for_, 3.0):
            self.get_logger().info("Waiting for Node [{}] ready.....".format(self._node_name_waiting_for_))
        self.get_logger().info("Node [{}] is OK".format(self._node_name_waiting_for_))
        
        self._initial_geo_pub_ = self.create_publisher(NavSatFix, "/gnss/fix", 5)
        
        for index in range(3):
            result = get_geolocation(self.get_name(), self._place_name_)
            if result != None :
                msg = NavSatFix()
                msg.latitude, msg.longitude = result
                msg.header.frame_id = "wgs84"
                msg.header.stamp = self.get_clock().now().to_msg()
                self._initial_geo_pub_.publish(msg)
                self.get_logger().info(
                    "-- Initialize Geolocation -- \n\t\t\t \
                        - Latitude: {}\n\t\t\t \
                        - Longitude: {}".format(result[0], result[1])
                )
                break
            
            if index == 2:
                self.get_logger().error("Couldn't get geo information. Please check your network and try Again!!!")
                return

def main():
    rclpy.init(args=sys.argv)
    node = Geocoding()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()