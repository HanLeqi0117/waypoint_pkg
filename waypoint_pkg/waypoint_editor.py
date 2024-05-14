from waypoint_pkg.waypoint_utils.utils import (
    Waypoint, Node, ParameterDescriptor, Marker,
    InteractiveMarkerServer, MenuHandler, ruamel,
    Duration, InteractiveMarkerFeedback, WaypointMode,
    InteractiveMarker, InteractiveMarkerControl,
    Point, os, rclpy,
    pose_to_waypoint,latlon_while_waypoint_altered,
    get_midlatlon
)


class WaypointEditor(Node):
    def __init__(self):
        super().__init__("waypoint_editor")
        self._waypoints_ : Waypoint.waypoints
        
        self._read_file_name_ = self.declare_parameter("read_file_name", os.path.join(os.environ['HOME'], "default_waypoint.yaml")).get_parameter_value().string_value
        self._write_file_name_ = self.declare_parameter("write_file_name", os.path.join(os.environ['HOME'], "default_rewaypoint.yaml")).get_parameter_value().string_value
        parameter_descriptor = ParameterDescriptor()
        parameter_descriptor.name = "waypoint_mode"
        parameter_descriptor.description = "GNSS or SLAM"
        self._mode_ = self.declare_parameter("mode", "SLAM", parameter_descriptor).get_parameter_value().string_value
        
        self._marker_pub_ = self.create_publisher(Marker, "waypoint_marker", 1)
        self._interactive_marker_server_ = InteractiveMarkerServer(self, "interacetive_marker_server")
        self._menu_handler_ = MenuHandler()
        with open(self._read_file_name_, "r") as f:
            self._waypoints_ = ruamel.yaml.safe_load(f)
        
        self.get_clock().sleep_for(Duration(seconds=0, nanoseconds=int(1e7)))

        self._menu_id_ = {}
        self._menu_id_['mode'] = {}
        
        self._menu_id_['erase'] = self._menu_handler_.insert(title="Erase", callback=self.process_callback)
        self._menu_id_['add'] = self._menu_handler_.insert(title="Add", callback=self.process_callback)
        self._menu_id_['info'] = self._menu_handler_.insert(title="Info", callback=self.process_callback)
        self._menu_id_['mode']['main'] = self._menu_handler_.insert(title="Mode", callback=self.process_callback)
        self._menu_id_['mode']['normal'] = self._menu_handler_.insert(title="Normal", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        self._menu_id_['mode']['search'] = self._menu_handler_.insert(title="Search", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        self._menu_id_['mode']['cancel'] = self._menu_handler_.insert(title="Cancel", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        self._menu_id_['mode']['direct'] = self._menu_handler_.insert(title="Direct", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        self._menu_id_['mode']['stop'] = self._menu_handler_.insert(title="Stop", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        self._menu_id_['mode']['signal'] = self._menu_handler_.insert(title="Signal", parent=self._menu_id_['mode']['main'], callback=self.process_callback)
        
        for index in range(len(self._waypoints_['waypoints'])):
            self.make_marker(index, self._waypoints_['waypoints'][index])
        
        self._interactive_marker_server_.applyChanges()
            
    def process_callback(self, feedback : InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            index = int(feedback.marker_name)
            mode = self._waypoints_['waypoints'][index]['mode']
            waypoint_after = pose_to_waypoint(feedback.pose)
            waypoint_after["latitude"], waypoint_after["longitude"] = latlon_while_waypoint_altered(self._waypoints_['waypoints'][index], waypoint_after)
            self._waypoints_['waypoints'][index] = waypoint_after
            self._waypoints_['waypoints'][index]['mode'] = mode
            
            self.update_path()

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.get_logger().info("Menu No.{} is selected".format(feedback.menu_entry_id))
            waypoint_index = int(feedback.marker_name)
            
            if feedback.menu_entry_id == self._menu_id_['erase']:
                self.get_logger().info("Erase Waypoint No.{}".format(waypoint_index))
                self.erase_marker(waypoint_index)
                self._waypoints_['waypoints'].pop(waypoint_index)
                self.make_marker_all(waypoint_index)
                self.update_path()
                return
            
            elif feedback.menu_entry_id == self._menu_id_['add']:
                self.get_logger().info("Insert one Waypoint after No.{}".format(waypoint_index))
                self.insert_marker(waypoint_index)
                return
                
            elif feedback.menu_entry_id == self._menu_id_['info']:
                self.get_logger().info("Print the information of Waypoint No.{}".format(waypoint_index))
                self.get_logger().info("The status of Waypoint: \
                    \n ================================================================================\
                    \nNo. {}: \
                    \n\tlatittude: {} \
                    \n\tlongtitude: {} \
                    \n\tx: {} \
                    \n\ty: {} \
                    \n\tyaw: {} \
                    \n\tmode: {} \
                ".format(
                    waypoint_index, 
                    self._waypoints_['waypoints'][waypoint_index]["latitude"], 
                    self._waypoints_['waypoints'][waypoint_index]["longitude"], 
                    self._waypoints_['waypoints'][waypoint_index]['pos_x'],
                    self._waypoints_['waypoints'][waypoint_index]['pos_y'],
                    self._waypoints_['waypoints'][waypoint_index]['yaw'],
                    self._waypoints_['waypoints'][waypoint_index]['mode']
                ))
                return
            
            elif feedback.menu_entry_id == self._menu_id_['mode']['main']:
                self.get_logger().info("Change Mode menu is selected.")
                return
            
            if feedback.menu_entry_id == self._menu_id_['mode']['normal']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.NORMAL
                self.get_logger().info("Change the mode of Waypoint No.{} to Normal".format(waypoint_index))
            elif feedback.menu_entry_id == self._menu_id_['mode']['search']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.SEARCH
                self.get_logger().info("Change the mode of Waypoint No.{} to Search".format(waypoint_index))
            elif feedback.menu_entry_id == self._menu_id_['mode']['cancel']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.CANCEL
                self.get_logger().info("Change the mode of Waypoint No.{} to Cancel".format(waypoint_index))
            elif feedback.menu_entry_id == self._menu_id_['mode']['direct']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.DIRECT
                self.get_logger().info("Change the mode of Waypoint No.{} to Direct".format(waypoint_index))
            elif feedback.menu_entry_id == self._menu_id_['mode']['stop']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.STOP
                self.get_logger().info("Change the mode of Waypoint No.{} to Stop".format(waypoint_index))
            elif feedback.menu_entry_id == self._menu_id_['mode']['signal']:
                self._waypoints_['waypoints'][waypoint_index]['mode'] = WaypointMode.SIGNAL
                self.get_logger().info("Change the mode of Waypoint No.{} to Signal".format(waypoint_index))
            
            self._interactive_marker_server_.erase(str(waypoint_index))
            self._interactive_marker_server_.applyChanges()
            self.make_marker(waypoint_index, self._waypoints_['waypoints'][waypoint_index])
            
        with open(self._write_file_name_, "w+") as f:
            ruamel.yaml.safe_dump(self._waypoints_, f)
    
    def make_marker(self, waypoint_number : int, waypoint : dict):
        interactive_marker = InteractiveMarker()
        arrow_control  = InteractiveMarkerControl()
        arrow_marker = Marker()
        
        interactive_marker.header.frame_id = "map"
        interactive_marker.name = str(waypoint_number)
        interactive_marker.scale = 1.0
        interactive_marker.pose.position.x = waypoint["pos_x"]
        interactive_marker.pose.position.y = waypoint["pos_y"]
        interactive_marker.pose.position.z = 0.2
        
        interactive_marker.pose.orientation.x = waypoint["quat_x"]
        interactive_marker.pose.orientation.y = waypoint["quat_y"]
        interactive_marker.pose.orientation.z = waypoint["quat_z"]
        interactive_marker.pose.orientation.w = waypoint["quat_w"]
        
        interactive_marker.description = "No. {} - mode. {}".format(waypoint_number, waypoint['mode'])
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.independent_marker_orientation = True
        arrow_control.always_visible = True
        arrow_control.interaction_mode = InteractiveMarkerControl.FIXED
        
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 0.65
        arrow_marker.color.g = 0.65
        arrow_marker.color.b = 0.65
        arrow_marker.color.a = 1.0
        arrow_control.markers.append(arrow_marker)
        interactive_marker.controls.append(arrow_control)
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.independent_marker_orientation = True        
        
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.independent_marker_orientation = True  
                
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        interactive_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.independent_marker_orientation = True 
                
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(control)
        
        self._interactive_marker_server_.insert(interactive_marker)
        self._interactive_marker_server_.setCallback(name=interactive_marker.name, feedback_callback=self.process_callback)
        self._menu_handler_.apply(server=self._interactive_marker_server_, marker_name=interactive_marker.name)

    def make_marker_all(self, waypoint_start_number : int):
        for index in range(waypoint_start_number, len(self._waypoints_['waypoints'])):
            self.make_marker(index, self._waypoints_['waypoints'][index])
        
        self._interactive_marker_server_.applyChanges()
    
    def erase_marker(self, waypoint_start_number : int):
        for index in range(waypoint_start_number, len(self._waypoints_['waypoints'])):
            self._interactive_marker_server_.erase(str(index))
            
        self._interactive_marker_server_.applyChanges()
    
    def insert_marker(self, waypoint_number : int):
        self.erase_marker(waypoint_number)
        next_waypoint = self._waypoints_['waypoints'][waypoint_number].copy()
        next_waypoint['pos_x'] = (
            self._waypoints_['waypoints'][waypoint_number]['pos_x']
            + self._waypoints_['waypoints'][waypoint_number + 1]['pos_x']
        ) / 2.0
        next_waypoint['pos_y'] = (
            self._waypoints_['waypoints'][waypoint_number]['pos_y']
            + self._waypoints_['waypoints'][waypoint_number + 1]['pos_y']
        ) / 2.0
        
        next_waypoint['latitude'], next_waypoint['longitude'] = get_midlatlon(
            next_waypoint['latitude'],
            next_waypoint['longitude'],
            self._waypoints_['waypoints'][waypoint_number + 1]['latitude'],
            self._waypoints_['waypoints'][waypoint_number + 1]['longitude']
        )
        
        next_waypoint['mode'] = 0
        self._waypoints_['waypoints'].insert(waypoint_number + 1, next_waypoint)
        self.make_marker_all(waypoint_number)
        self.update_path()
    
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
        
        for index in range(len(self._waypoints_['waypoints'])):
            point = Point()
            point.x = self._waypoints_['waypoints'][index]['pos_x']
            point.y = self._waypoints_['waypoints'][index]['pos_y']
            point.z = 0.0
            
            line_strip.points.append(point)
        
        self._marker_pub_.publish(line_strip)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointEditor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            
            
