from utils.utils import *

class Data2Waypoint(Node):
    
    def __init__(self):
        super().__init__("data_to_waypoint")
        
        self._waypoint_file_ = self.declare_parameter("waypoint_file", os.environ["HOME"]).get_parameter_value().string_value
        self._waypoint_distance_ = self.declare_parameter("waypoint_distance", 4.0).get_parameter_value().double_value
        self._yaw_deg_thresh_ = self.declare_parameter("yaw_deg_thresh", 15.0).get_parameter_value().double_value
        self._deg_chord_ = self.declare_parameter("deg_chord", 1.0).get_parameter_value().double_value
        
        self._tf_buffer_ = Buffer()
        self._tf_listener_ = TransformListener(self._tf_buffer_, self)
        
        self._marker_pub_ = self.create_publisher(Marker, "waypoint_marker", QoSProfile(100))
        
        self._navsat_fix_sub_ = self.create_subscription(NavSatFix, "fix", self.sub_navsat_fix, QoSProfile(20))
        
        self._main_process_timer_ = self.create_timer(0.01, self.main_process)
    
    
    def main_process(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
    
    rclpy.shutdown()
    
    
