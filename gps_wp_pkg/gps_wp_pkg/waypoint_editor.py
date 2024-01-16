from gps_wp_pkg.waypoint_utils.utils import *

class WaypointEditor(Node):
    def __init__(self):
        super().__init__("waypoint_editor")
        
        self._read_file_name_ = self.declare_parameter("read_file_name", os.path.join(os.environ['HOME'], "default_waypoint.yaml")).get_parameter_value().string_value
        self._write_file_name_ = self.declare_parameter("write_file_name", os.path.join(os.environ['HOME'], "default_rewaypoint.yaml")).get_parameter_value().string_array_value
        parameter_descriptor = ParameterDescriptor()
        parameter_descriptor.name = "waypoint_mode"
        parameter_descriptor.description = "GPS or SLAM"
        self._mode_ = self.declare_parameter("mode", "SLAM", parameter_descriptor).get_parameter_value().string_value
        
        self._marker_pub_ = self.create_publisher(Marker, "waypoint_marker", 1)
        self._interactive_marker_server_ = InteractiveMarkerServer(self, "interacetive_marker_server")
        self._menu_handler_ = MenuHandler()
        self._read_file_stream_ = open(self._read_file_name_, "r")
        self._write_file_stream_ = open(self._write_file_name_, "w")
        
        self.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(1e7)))

        self._waypoints_ = ruamel.yaml.safe_load(self._read_file_stream_)["waypoints"]
        
        self._menu_handler_.insert(title="Erase", callback=self.process_callback)
        self._menu_handler_.insert(title="Add", callback=self.process_callback)
        change_menu_handler = self._menu_handler_.insert(title="Change Mode", callback=self.process_callback)
        self._menu_handler_.insert(title="Info", callback=self.process_callback)
        self._menu_handler_.insert(title="Normal", parent=change_menu_handler, callback=self.process_callback)
        
        for index in range(len(self._waypoints_)):
            self.make_marker(index, self._waypoints_[index])
        
        self._interactive_marker_server_.applyChanges()
        
        self._read_file_stream_.close()
        
    
    def process_callback(self, feedback = InteractiveMarkerFeedback()):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            index = int(feedback.marker_name)
            mode = self._waypoints_[index]['mode']
            self._waypoints_[index] = pose_to_waypoint(feedback.pose)
            self._waypoints_[index]['mode'] = mode
            
            self.update_path()
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.get_logger().info("Menu No.{} is selected".format(feedback.menu_entry_id))
            
            if(feedback.menu_entry_id == 1):
                pass
    
    def make_marker(self, waypoint_number=int(), waypoint=dict()):
        interactive_marker = InteractiveMarker()
        control, arrow_control  = InteractiveMarkerControl(), InteractiveMarkerControl()
        arrow_marker = Marker()
        
        interactive_marker.header.frame_id = "map"
        interactive_marker.name = str(waypoint_number)
        interactive_marker.scale = 1
        interactive_marker.pose.position.x = waypoint["pos_x"]
        interactive_marker.pose.position.y = waypoint["pos_y"]
        interactive_marker.pose.position.z = 0.2
        
        interactive_marker.pose.orientation.x = waypoint["quat_x"]
        interactive_marker.pose.orientation.y = waypoint["quat_y"]
        interactive_marker.pose.orientation.z = waypoint["quat_z"]
        interactive_marker.pose.orientation.w = waypoint["quat_w"]
        
        interactive_marker.name = "No. {} - mode. {}".format(waypoint_number, waypoint['mode'])

        control.always_visible = True
        arrow_control.always_visible = True
        control.independent_marker_orientation = True
        arrow_control.interaction_mode = InteractiveMarkerControl.NONE
        
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.x = 0.1
        arrow_marker.color.r = 0.65
        arrow_marker.color.g = 0.65
        arrow_marker.color.b = 0.65
        arrow_marker.color.a = 1.0
        arrow_control.markers.append(arrow_marker)
        interactive_marker.controls.append(arrow_control)
                
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(control)
        
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        interactive_marker.controls.append(control)
        
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(control)
        
        self._interactive_marker_server_.setCallback(marker=interactive_marker.name, feedback_callback=self.process_callback)
        self._menu_handler_.apply(self._interactive_marker_server_, interactive_marker.name)

    def make_marker_all(self, waypoint_start_number=int()):
        for index in range(waypoint_start_number, len(self._waypoints_)):
            self.make_marker(index, self._waypoints_[index])
        
        self._interactive_marker_server_.applyChanges()
    
    def erase_marker(self, waypoint_start_number=int()):
        for index in range(waypoint_start_number, len(self._waypoints_)):
            self._interactive_marker_server_.erase(str(index))
            
        self._interactive_marker_server_.applyChanges()
    
    def update_path(self):
        line_strip = Marker()
        
        line_strip.header.frame_id = "map"
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.ns = "lines"
        line_strip.action = Marker.ADD
        line_strip.pose.orientation.w = 1.0
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.scale.x = 0.1
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0
        
        for index in range(len(self._waypoints_)):
            point = Point()
            point.x = self._waypoints_[index]['pos_x']
            point.y = self._waypoints_[index]['pos_y']
            point.z = 0.0
            
            line_strip.points.append(point)

            
            
            