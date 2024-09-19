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
	RCLCPP_DEBUG(get_logger(), "          x: %f", waypoints[0].pos_x);
	RCLCPP_DEBUG(get_logger(), "          y: %f", waypoints[0].pos_y);
	RCLCPP_DEBUG(get_logger(), "          z: %f", waypoints[0].pos_z);
	RCLCPP_DEBUG(get_logger(), "Rotation :");
	RCLCPP_DEBUG(get_logger(), "          roll  : %f", waypoints[0].roll);
	RCLCPP_DEBUG(get_logger(), "          pitch : %f", waypoints[0].pitch);
	RCLCPP_DEBUG(get_logger(), "          yaw   : %f", waypoints[0].yaw);
	RCLCPP_DEBUG(get_logger(), "GNSS :");
	RCLCPP_DEBUG(get_logger(), "          longitude : %f", waypoints[0].longitude);
	RCLCPP_DEBUG(get_logger(), "          latitude  : %f", waypoints[0].latitude);
	RCLCPP_DEBUG(get_logger(), "Mode : %d", waypoints[0].mode);
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
	
	for (auto &&waypoint : waypoints){
		make_marker(waypoint.first, waypoint.second);
	}
	
	// 変更を有効にする
	marker_server->applyChanges();
}

void WaypointHandler::process_feedback(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
	unsigned int index = std::stoi(feedback->marker_name);
	// 選択条件：「feedback」のメンバ「event_type」
	switch(feedback->event_type)
	{
		// このマーカーの位置は、既に定義された制御手法によって変更された場合
		case InteractiveMarkerFeedback::POSE_UPDATE:
		{
			waypoints[index].pos_x = feedback->pose.position.x;
			waypoints[index].pos_y = feedback->pose.position.y;
			waypoints[index].pos_z = feedback->pose.position.z;
			waypoints[index].quat_x = feedback->pose.orientation.x;
			waypoints[index].quat_y = feedback->pose.orientation.y;
			waypoints[index].quat_z = feedback->pose.orientation.z;
			waypoints[index].quat_w = feedback->pose.orientation.w;
			tf2::Quaternion tf2_quat;
			tf2::fromMsg(feedback->pose.orientation, tf2_quat);
			tf2::Matrix3x3(tf2_quat).getRPY(
				waypoints[index].roll,
				waypoints[index].pitch,
				waypoints[index].yaw       
			);
			publish_line();
			break;
		}

		case InteractiveMarkerFeedback::MENU_SELECT:
		{
			std::stringstream ss;
            ss << "Menu" << feedback->menu_entry_id << " selected" << std::endl;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
			
			if(feedback->menu_entry_id == main_menu_entry_id["Delete"]){
				std::stringstream ss;
                ss << "No" << index << "is deleted." << std::endl;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
				erase_marker(index);
			}

			if(feedback->menu_entry_id == main_menu_entry_id["Insert"]){
				std::stringstream ss;
                ss << "No" << index << "insert" << std::endl;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                insert_marker(index);
            }

			if(feedback->menu_entry_id == main_menu_entry_id["Info"]){
				std::stringstream ss;
                ss << "No" << index << "insert" << std::endl;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                print_waypoint_info(index);
            }

			if(feedback->menu_entry_id == main_menu_entry_id["Mode"]){
				RCLCPP_INFO(get_logger(), "Change mode menu is selected");
			}

			if (feedback->menu_entry_id == sub_menu_entry_id["NORMAL"]) {
				waypoints[index].mode = Waypoint::WaypointMode::NORMAL;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to NORMAL", index);
			} else if (feedback->menu_entry_id == sub_menu_entry_id["SEARCH"]) {
				waypoints[index].mode = Waypoint::WaypointMode::SEARCH;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to SEARCH", index);
			} else if (feedback->menu_entry_id == sub_menu_entry_id["CANCEL"]) {
				waypoints[index].mode = Waypoint::WaypointMode::CANCEL;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to CANCEL", index);
			} else if (feedback->menu_entry_id == sub_menu_entry_id["DIRECT"]) {
				waypoints[index].mode = Waypoint::WaypointMode::DIRECT;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to DIRECT", index);
			} else if (feedback->menu_entry_id == sub_menu_entry_id["STOP"]) {
				waypoints[index].mode = Waypoint::WaypointMode::STOP;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to STOP", index);
			} else if (feedback->menu_entry_id == sub_menu_entry_id["SIGNAL"]) {
				waypoints[index].mode = Waypoint::WaypointMode::SIGNAL;
				RCLCPP_INFO(get_logger(), "Waypoint No. %d mode is changed to SIGNAL", index);
			}

			erase_marker(index);
			make_all_marker(index);
			publish_line();
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
