from waypoint_pkg.waypoint_utils.utils import *

class Data2Waypoint(Node):
    
    def __init__(self):
        super().__init__("data_to_waypoint")
        self._waypoints_ : Waypoint.waypoints
        
        self._waypoint_file_ = self.declare_parameter("waypoint_file", os.path.join(os.environ['HOME'], 'default_waypoint.yaml')).get_parameter_value().string_value
        self._target_frame_ = self.declare_parameter("target_frame", "map").get_parameter_value().string_value
        self._source_frame_ = self.declare_parameter("source_frame", "base_link").get_parameter_value().string_value
        self._waypoint_distance_ = self.declare_parameter("waypoint_distance", 4.0).get_parameter_value().double_value
        self._yaw_deg_thresh_ = self.declare_parameter("yaw_deg_thresh", 15.0).get_parameter_value().double_value
        self._deg_chord_ = self.declare_parameter("deg_chord", 1.0).get_parameter_value().double_value
        
        self._tf_buffer_ = Buffer()
        self._tf_listener_ = TransformListener(self._tf_buffer_, self)
        self._transform_ = TransformStamped()
        self._compus_imu_data_ = Imu()
        self._navsat_fix_data_ = NavSatFix()
        self._gnss_odometry_data_ = Odometry()
        self._waypoints_ = Waypoint.waypoints()
        self._lat_last_ = None
        self._lon_last_ = None
        
        self._marker_pub_ = self.create_publisher(Marker, "waypoint_marker", 100)
        
        self._navsat_fix_sub_ = self.create_subscription(NavSatFix, "fix", self.navsat_fix_cb, 20)
        self._gnss_odometry_sub_ = self.create_subscription(Odometry, "odometry", self.gnss_odometry_cb, 20)
        self._compus_imu_sub_ = self.create_subscription(Imu, "imu", self.compus_imu_cb, 20)
        
        self._main_process_timer_ = self.create_timer(0.01, self.main_process)
    
    def compus_imu_cb(self, msg : Imu):
        self._compus_imu_data_ = msg
    
    def navsat_fix_cb(self, msg : NavSatFix):
        self._navsat_fix_data_ = msg
    
    def gnss_odometry_cb(self, msg : Odometry):
        self._gnss_odometry_data_ = msg
    
    def main_process(self):
        waypoint : Waypoint.waypoint
        
        if self._tf_buffer_.can_transform(self._target_frame_, self._source_frame_, Time()):
            self._transform_ = self._tf_buffer_.lookup_transform(self._target_frame_, self._source_frame_, Time(seconds=0, nanoseconds=0))
            self.get_logger().debug("Transform between map and base_link is ok.x: {}, y: {}".format(self._transform_.transform.translation.x, self._transform_.transform.translation.y))
        else:
            self.get_logger().warn("Trasform between map and base_link is not ready")
            self.get_clock().sleep_for(Duration(seconds=1))
            return
        
        if self._lat_last_ == None and self._navsat_fix_data_ != None:
            self._lat_last_ = self._navsat_fix_data_.latitude
            self._lon_last_ = self._navsat_fix_data_.longitude
        
        waypoint = pose_to_waypoint(self._gnss_odometry_data_.pose.pose)
        waypoint['quat_x'] = self._compus_imu_data_.orientation.x
        waypoint['quat_y'] = self._compus_imu_data_.orientation.y
        waypoint['quat_z'] = self._compus_imu_data_.orientation.z
        waypoint['quat_w'] = self._compus_imu_data_.orientation.w
        waypoint['yaw'] = euler_from_quaternion([
            waypoint['quat_x'],
            waypoint['quat_y'],
            waypoint['quat_z'],
            waypoint['quat_w']
        ])[2]
        waypoint_diff = get_dist_between_geos(
            self._lat_last_,
            self._lon_last_,
            self._navsat_fix_data_.latitude,
            self._navsat_fix_data_.longitude
        )
        
        waypoint['longitude'] = self._navsat_fix_data_.longitude
        waypoint['latitude'] = self._navsat_fix_data_.latitude

        if len(self._waypoints_["waypoints"]) == 0:
            self._waypoints_['waypoints'].append(waypoint)
            return
        
        yaw_diff = abs(
            waypoint['yaw']
            - self._waypoints_["waypoints"][len(self._waypoints_["waypoints"]) - 1]['yaw']
        )
        
        # Debug
        self.get_logger().debug(
            "No. {}, yaw: {}, yaw_diff: {}, waypoint_diff: {}".format(
                len(self._waypoints_["waypoints"]), waypoint['yaw'] / math.pi * 180.0,
                yaw_diff, waypoint_diff
            )
        )
        
        if (yaw_diff / math.pi * 180.0 > self._yaw_deg_thresh_ \
            and waypoint_diff > self._deg_chord_) \
                or waypoint_diff > self._waypoint_distance_:
            
            self._waypoints_["waypoints"].append(waypoint)
            
            self._lat_last_ = self._navsat_fix_data_.latitude
            self._lon_last_ = self._navsat_fix_data_.longitude
            
            with open(self._waypoint_file_, "w+") as f:
                ruamel.yaml.safe_dump(self._waypoints_, f)
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gnss_waypoint"
        marker.color.b = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0
        
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        
        marker.pose.orientation.w = 1.0
        
        for i in range(len(self._waypoints_['waypoints'])):
            point = Point()

            point.x = self._waypoints_['waypoints'][i]['pos_x']
            point.y = self._waypoints_['waypoints'][i]['pos_y']
            point.z = 0.25
            
            marker.points.append(point)

        self._marker_pub_.publish(marker)
    
        
def main(args=None):
    rclpy.init(args=args)
    node = Data2Waypoint()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
    
