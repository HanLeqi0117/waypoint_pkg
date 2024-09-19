#include <waypoint_pkg/utilis.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

using namespace interactive_markers;
using namespace visualization_msgs::msg;
using namespace std::placeholders;

class WaypointHandler : public rclcpp::Node
{
    public: 

        WaypointHandler();
        ~WaypointHandler();

        /**
         * @brief Check the read file path and write file path
         * @param 
         * @return If all paths is ok, return true. Otherwise false.
         */        
        bool open_files() {

            try {
                auto root = YAML::LoadAllFromFile(read_file_name);

                for (auto &&doc : root){         
                    for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
                        
                        Waypoint::Waypoint waypoint;
                        int key = it->first.as<int>();
                        const YAML::Node& data = it->second;

                        waypoint.pos_x = data["position_x"].as<double>();
                        waypoint.pos_y = data["position_y"].as<double>();
                        waypoint.pos_z = data["position_z"].as<double>();
                        waypoint.quat_x = data["quaternion_x"].as<double>();
                        waypoint.quat_y = data["quaternion_y"].as<double>();
                        waypoint.quat_z = data["quaternion_z"].as<double>();
                        waypoint.quat_w = data["quaternion_w"].as<double>();
                        waypoint.roll = data["roll"].as<double>();
                        waypoint.pitch = data["pitch"].as<double>();
                        waypoint.yaw = data["yaw"].as<double>();
                        waypoint.longitude = data["longitude"].as<double>();
                        waypoint.latitude = data["latitude"].as<double>();
                        waypoint.mode = static_cast<Waypoint::WaypointMode>(data["mode"].as<int>());

                        waypoints[key] = waypoint;
                    }
                }
            } catch (const YAML::Exception& e) {
                std::stringstream ss;
                ss << "YAML Exception: " << e.what();
                RCLCPP_ERROR(get_logger(), ss.str().c_str());
                return false;
            }

            return true;

        }

        /**
         * @brief save the waypoint data into a yaml file
         * @param 
         * @return save write file.
         */  
        void save_file(){
            // YAML::Node inner_node;
            YAML::Emitter out;
            if (waypoints.size() == 0 && waypoints.empty()){
                RCLCPP_WARN(get_logger(), "There is no data to write! Please check your input yaml file path and its content.");
                return;
            }

            for (auto &&pair : waypoints)
            {
                out << YAML::BeginMap;
                out << YAML::Key << pair.first << YAML::Value << YAML::BeginMap;
                out << YAML::Key << "position_x" << YAML::Value << pair.second.pos_x;
                out << YAML::Key << "position_y" << YAML::Value << pair.second.pos_y;
                out << YAML::Key << "position_z" << YAML::Value << pair.second.pos_z;
                out << YAML::Key << "quaternion_x" << YAML::Value << pair.second.quat_x;
                out << YAML::Key << "quaternion_y" << YAML::Value << pair.second.quat_y;
                out << YAML::Key << "quaternion_z" << YAML::Value << pair.second.quat_z;
                out << YAML::Key << "quaternion_w" << YAML::Value << pair.second.quat_w;
                out << YAML::Key << "roll" << YAML::Value << pair.second.roll;
                out << YAML::Key << "pitch" << YAML::Value << pair.second.pitch;
                out << YAML::Key << "yaw" << YAML::Value << pair.second.yaw;
                out << YAML::Key << "longitude" << YAML::Value << pair.second.longitude;
                out << YAML::Key << "latitude" << YAML::Value << pair.second.latitude;
                out << YAML::Key << "mode" << YAML::Value << int(pair.second.mode);
                out << YAML::EndMap << YAML::EndMap; 
            }
            
            ofs = std::ofstream(write_file_name);
            ofs << out.c_str();
            ofs.close();
        }
        
        /**
         * @brief Get Waypoint data from a yaml file and initialize a InterfactiveMarkerServer
         * @param 
         * @return save write file.
         */  
        void initialize();


    private:

        std::string read_file_name;
        std::string write_file_name;

        std::map<std::string, uint32_t> main_menu_entry_id;
        std::map<std::string, uint32_t> sub_menu_entry_id;

        std::shared_ptr<InteractiveMarkerServer> marker_server;
        Waypoint::Waypoints waypoints;
        rclcpp::Publisher<Marker>::SharedPtr marker_pub;
        MenuHandler menu_handler;
        std::ofstream ofs;

    private:
        // 引数：ROSメッセージインターフェイス「InteractiveMarkerFeedback」のコンスタントポインタ
        // 「InteractiveMarkerFeedback」の内容について、URL：http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerFeedback.html
        // 機能：異なるフィードバックの状態に基づいて、異なる処理(メニューの変更処理と位置情報の変更処理)を行う   		
        void process_feedback(InteractiveMarkerFeedback::ConstSharedPtr feedback);

        // Waypointを編集できるようにする機能を与える関数.次のサイトを参考にして作られている
        // <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server>
        // <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls>
        // 引数1：Waypointとマーカーの番号
        // 引数2：「Waypoint」構造体のインスタンス	
        // 機能：「Waypoint」のインスタンス -> 動的マーカー(Waypoint)
        void make_marker(int name, Waypoint::Waypoint wayp_tmp){

            InteractiveMarker inter_marker;

            inter_marker.header.frame_id = "map";
            inter_marker.scale = 1;
            inter_marker.pose.position.x = wayp_tmp.pos_x;
            inter_marker.pose.position.y = wayp_tmp.pos_y;
            inter_marker.pose.position.z = 0.2;

            inter_marker.pose.orientation.x = wayp_tmp.quat_x;
            inter_marker.pose.orientation.y = wayp_tmp.quat_y;
            inter_marker.pose.orientation.z = wayp_tmp.quat_z;
            inter_marker.pose.orientation.w = wayp_tmp.quat_w;

            std::stringstream ss;
            inter_marker.name = std::to_string(name);
            ss << "No." << inter_marker.name.c_str() << "-" << "mode." << get_mode_name(static_cast<int>(wayp_tmp.mode));
            inter_marker.description = ss.str().c_str();

            InteractiveMarkerControl control, arrow_control;

            control.always_visible = true;
            arrow_control.always_visible = true;
            control.independent_marker_orientation = true;

            arrow_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

            Marker arrow_marker;
            arrow_marker.type = Marker::ARROW;
            arrow_marker.scale.x = 0.5;
            arrow_marker.scale.y = 0.1;
            arrow_marker.scale.z = 0.1;
            arrow_marker.color.r = 0.65;
            arrow_marker.color.g = 0.65;
            arrow_marker.color.b = 0.65;
            arrow_marker.color.a = 1.0;

            arrow_control.markers.push_back(arrow_marker);
            inter_marker.controls.push_back(arrow_control);

            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_x";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            inter_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
            inter_marker.controls.push_back(control);

            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "move_y";
            control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
            inter_marker.controls.push_back(control);

            marker_server->insert(inter_marker);

            marker_server->setCallback(
                inter_marker.name, 
                std::bind(&WaypointHandler::process_feedback, this, _1)
            );
            
            menu_handler.apply(*marker_server, inter_marker.name);
        }

        // 引数：Waypointの番号
        // 機能：Waypointのコンテナに基づいて「start」番号からマーカーを作成する
        void make_all_marker(int start){

            for(int i = start; i < static_cast<int>(waypoints.size()); i++){
                make_marker(i, waypoints[i]);
            }
            marker_server->applyChanges();
        }

        // 引数：Waypointの番号
        // 機能：「start」番号からすべてのマーカーを削除する
        void erase_marker(int start){
            for(int i = start; i < static_cast<int>(waypoints.size()); i++){
                marker_server->erase(std::to_string(i));
            }
	
            erase_waypoint(start);
            marker_server->applyChanges();
        }

        // 引数：挿入する場所を示す番号
        // 機能：「name」の後ろに一つWaypointを挿入する
        void insert_marker(int name){
            if (name == static_cast<int>(waypoints.size())){
                RCLCPP_WARN(get_logger(), "You selected the end of Waypoint, and couldn't insert a waypoint behind.");
                return;
            }

            Waypoint::Waypoint waypoint_new;
            waypoint_new.pos_x = (
                waypoints[name].pos_x + 
                waypoints[name + 1].pos_x
            ) / 2;
            waypoint_new.pos_y = (
                waypoints[name].pos_y + 
                waypoints[name + 1].pos_y
            ) / 2;
            waypoint_new.mode = Waypoint::WaypointMode::NORMAL;

            for (int index = static_cast<int>(waypoints.size()); index > name; --index) {
                waypoints[index + 1] = waypoints[index];
            }
            waypoints[name + 1] = waypoint_new;

        }

        void erase_waypoint(int waypoint_number){
            waypoints.erase(waypoint_number);
            int newkey = 0;
            for (auto &&waypoint : waypoints){
                waypoints[newkey] = waypoint.second;
                newkey ++;
            }
        }

        void print_waypoint_info(int waypoint_number){

            RCLCPP_INFO(get_logger(), "========== Waypoint No. %d ==========", waypoint_number);
            RCLCPP_INFO(get_logger(), "Position :");
            RCLCPP_INFO(get_logger(), "          x: %f", waypoints[waypoint_number].pos_x);
            RCLCPP_INFO(get_logger(), "          y: %f", waypoints[waypoint_number].pos_y);
            RCLCPP_INFO(get_logger(), "          z: %f", waypoints[waypoint_number].pos_z);
            RCLCPP_INFO(get_logger(), "Rotation :");
            RCLCPP_INFO(get_logger(), "          roll  : %f", waypoints[waypoint_number].roll);
            RCLCPP_INFO(get_logger(), "          pitch : %f", waypoints[waypoint_number].pitch);
            RCLCPP_INFO(get_logger(), "          yaw   : %f", waypoints[waypoint_number].yaw);
            RCLCPP_INFO(get_logger(), "GNSS :");
            RCLCPP_INFO(get_logger(), "          longitude : %f", waypoints[waypoint_number].longitude);
            RCLCPP_INFO(get_logger(), "          latitude  : %f", waypoints[waypoint_number].latitude);
            switch (static_cast<Waypoint::WaypointMode>(waypoints[waypoint_number].mode)){
                case Waypoint::WaypointMode::NORMAL:
                    RCLCPP_INFO(get_logger(), "Mode : NORMAL");
                    break;
                case Waypoint::WaypointMode::SEARCH:
                    RCLCPP_INFO(get_logger(), "Mode : SEARCH");
                    break;
                case Waypoint::WaypointMode::CANCEL:
                    RCLCPP_INFO(get_logger(), "Mode : CANCEL");
                    break;
                case Waypoint::WaypointMode::DIRECT:
                    RCLCPP_INFO(get_logger(), "Mode : DIRECT");
                    break;
                case Waypoint::WaypointMode::STOP:
                    RCLCPP_INFO(get_logger(), "Mode : STOP");
                    break;
                case Waypoint::WaypointMode::SIGNAL:
                    RCLCPP_INFO(get_logger(), "Mode : SIGNAL");
                    break;
                default:
                    break;
            }
            RCLCPP_INFO(get_logger(), "========== Waypoint No. %d ==========", 0);            
            return;
        }

        // 機能：現にあるすべてのWaypointの位置情報にしたがって、近い2つのWaypointの間で線を引く
        void publish_line(){
            Marker line_strip;
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = this->get_clock().get()->now();
            line_strip.ns = "lines";
            line_strip.action = Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 0;
            line_strip.type = Marker::LINE_STRIP;
            line_strip.scale.x = 0.1;
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
            for (auto &&waypoint : waypoints)
            {
                geometry_msgs::msg::Point point_tmp;
                point_tmp.x = waypoint.second.pos_x;
                point_tmp.y = waypoint.second.pos_y;
                point_tmp.z = waypoint.second.pos_z;

                line_strip.points.push_back(point_tmp);
            }
            
            marker_pub->publish(line_strip);
        }

        // 引数：Waypointのモードを示す番号(int型)
        // 戻り値：「mode_number」番号と対応するモードの名前
        // 機能：モード番号を渡してそのモード番号と対応する名前を取得する
        std::string get_mode_name(int mode_number){
            std::string mode_name;
            switch(mode_number){
                case static_cast<int>(Waypoint::WaypointMode::NORMAL) :
                    mode_name += "NORMAL";
                    break;
                case static_cast<int>(Waypoint::WaypointMode::SEARCH) :
                    mode_name += "SEARCH";
                    break;
                case static_cast<int>(Waypoint::WaypointMode::CANCEL) :
                    mode_name += "CANCEL";
                    break;
                case static_cast<int>(Waypoint::WaypointMode::DIRECT) :
                    mode_name += "DIRECT";
                    break;
                case static_cast<int>(Waypoint::WaypointMode::STOP) :
                    mode_name += "STOP";
                    break;
                case static_cast<int>(Waypoint::WaypointMode::SIGNAL) :
                    mode_name += "SIGNAL";
            }
            return mode_name;
        }    
};