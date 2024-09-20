#include <waypoint_pkg/marker_server.hpp>
#include <rclcpp/rclcpp.hpp>


WaypointHandler::WaypointHandler() : Node("marker_server")
{
    read_file_name = this->declare_parameter<std::string>("read_file_name", std::string(std::getenv("HOME")) + "/Documents/test_waypoint_recorder.yaml");
    write_file_name = this->declare_parameter<std::string>("write_file_name", std::string(std::getenv("HOME")) + "/Documents/test_waypoint_recorder_rewrite.yaml");

    this->marker_pub = this->create_publisher<Marker>("marker", 1);
	this->marker_server = std::make_shared<InteractiveMarkerServer>("interactive_marker_topic", this);

    rclcpp::sleep_for(std::chrono::milliseconds(10));

}

WaypointHandler::~WaypointHandler()
{
    save_file();

	RCLCPP_DEBUG(get_logger(), "========== Waypoint No. %d ==========", 0);
	RCLCPP_DEBUG(get_logger(), "Position :");
	RCLCPP_DEBUG(get_logger(), "          x: %f", waypoints_vector[0].pos_x);
	RCLCPP_DEBUG(get_logger(), "          y: %f", waypoints_vector[0].pos_y);
	RCLCPP_DEBUG(get_logger(), "          z: %f", waypoints_vector[0].pos_z);
	RCLCPP_DEBUG(get_logger(), "Rotation :");
	RCLCPP_DEBUG(get_logger(), "          roll  : %f", waypoints_vector[0].roll);
	RCLCPP_DEBUG(get_logger(), "          pitch : %f", waypoints_vector[0].pitch);
	RCLCPP_DEBUG(get_logger(), "          yaw   : %f", waypoints_vector[0].yaw);
	RCLCPP_DEBUG(get_logger(), "GNSS :");
	RCLCPP_DEBUG(get_logger(), "          longitude : %f", waypoints_vector[0].longitude);
	RCLCPP_DEBUG(get_logger(), "          latitude  : %f", waypoints_vector[0].latitude);
	RCLCPP_DEBUG(get_logger(), "Mode : %d", waypoints_vector[0].mode);
	RCLCPP_DEBUG(get_logger(), "========== Waypoint No. %d ==========", 0);

    if(this->ofs.is_open())ofs.close();
}

void WaypointHandler::initialize()
{

	main_menu_entry_id["Delete"] = menu_handler.insert("Delete", std::bind(&WaypointHandler::process_feedback, this, _1));
	main_menu_entry_id["Insert"] = menu_handler.insert("Insert", std::bind(&WaypointHandler::process_feedback, this, _1));
	main_menu_entry_id["Info"] = menu_handler.insert("Info", std::bind(&WaypointHandler::process_feedback, this, _1));
	main_menu_entry_id["Mode"] = menu_handler.insert("Change mode", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["NORMAL"] = menu_handler.insert(main_menu_entry_id["Mode"], "NORMAL", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["SEARCH"] = menu_handler.insert(main_menu_entry_id["Mode"], "SEARCH", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["CANCEL"] = menu_handler.insert(main_menu_entry_id["Mode"], "CANCEL", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["DIRECT"] = menu_handler.insert(main_menu_entry_id["Mode"], "DIRECT", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["STOP"] = menu_handler.insert(main_menu_entry_id["Mode"], "STOP", std::bind(&WaypointHandler::process_feedback, this, _1));
	sub_menu_entry_id["SIGNAL"] = menu_handler.insert(main_menu_entry_id["Mode"], "SIGNAL", std::bind(&WaypointHandler::process_feedback, this, _1));
	
	for (int i = 0; i < static_cast<int>(waypoints_vector.size()); i++){
		make_marker(i, waypoints_vector[i]);
	}
	
	// 変更を有効にする
	marker_server->applyChanges();
}

void WaypointHandler::process_feedback(InteractiveMarkerFeedback::ConstSharedPtr feedback){

	// 選択条件：「feedback」のメンバ「event_type」
	switch(feedback->event_type)
	{
		// このマーカーの位置は、既に定義された制御手法によって変更された場合
		case InteractiveMarkerFeedback::POSE_UPDATE:
		{
			unsigned int index = std::stoi(feedback->marker_name);
			waypoints_vector[index].pos_x = feedback->pose.position.x;
			waypoints_vector[index].pos_y = feedback->pose.position.y;
			waypoints_vector[index].pos_z = feedback->pose.position.z;
			waypoints_vector[index].quat_x = feedback->pose.orientation.x;
			waypoints_vector[index].quat_y = feedback->pose.orientation.y;
			waypoints_vector[index].quat_z = feedback->pose.orientation.z;
			waypoints_vector[index].quat_w = feedback->pose.orientation.w;
			tf2::Quaternion tf2_quat;
			tf2::fromMsg(feedback->pose.orientation, tf2_quat);
			tf2::Matrix3x3(tf2_quat).getRPY(
				waypoints_vector[index].roll,
				waypoints_vector[index].pitch,
				waypoints_vector[index].yaw       
			);
			publish_line();				
			break;
		}

		case InteractiveMarkerFeedback::MENU_SELECT:
		{
			unsigned int index = std::stoi(feedback->marker_name);
			std::stringstream ss;
            ss << "Menu " << feedback->menu_entry_id << " is selected";
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
			
			if(feedback->menu_entry_id == main_menu_entry_id["Delete"]){
				std::stringstream ss;
                ss << "No. " << index << " is deleted.";
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
				erase_marker(index);
				waypoints_vector.erase(waypoints_vector.begin() + index);
				make_all_marker(index);
				publish_line();
			}

			if(feedback->menu_entry_id == main_menu_entry_id["Insert"]){
				std::stringstream ss;
                ss  << "Insert a Waypoint behind Waypoint No. " << index;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                insert_marker(index);
            }

			if(feedback->menu_entry_id == main_menu_entry_id["Info"]){
				std::stringstream ss;
                ss << "Show The information of the Waypoint No. " << index;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                print_waypoint_info(index);
            }

			if(feedback->menu_entry_id >= main_menu_entry_id["Mode"]){
				RCLCPP_INFO(get_logger(), "Change Mode Menu is selected");

				if (feedback->menu_entry_id == sub_menu_entry_id["NORMAL"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::NORMAL;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to NORMAL", index);
				} else if (feedback->menu_entry_id == sub_menu_entry_id["SEARCH"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::SEARCH;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to SEARCH", index);
				} else if (feedback->menu_entry_id == sub_menu_entry_id["CANCEL"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::CANCEL;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to CANCEL", index);
				} else if (feedback->menu_entry_id == sub_menu_entry_id["DIRECT"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::DIRECT;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to DIRECT", index);
				} else if (feedback->menu_entry_id == sub_menu_entry_id["STOP"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::STOP;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to STOP", index);
					publish_line();				
				} else if (feedback->menu_entry_id == sub_menu_entry_id["SIGNAL"]) {
					waypoints_vector[index].mode = Waypoint::WaypointMode::SIGNAL;
					RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to SIGNAL", index);
				}

				marker_server->erase(std::to_string(index));
				marker_server->applyChanges();
				make_marker(index, waypoints_vector[index]);

			}

			break;
		}

	
	}
	// 変更を有効にする
	marker_server->applyChanges();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto waypoint_handler_node = std::make_shared<WaypointHandler>();
    
    // ファイルを開くことが成功した場合
    if(waypoint_handler_node->open_files()){
		waypoint_handler_node->initialize();
        rclcpp::spin(waypoint_handler_node);
    }
    else{
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::shutdown();
    return 0;
}
