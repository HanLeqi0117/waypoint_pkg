#include <waypoint_pkg/utilis.hpp>
#include <rclcpp/rclcpp.hpp>

class WaypointRecorder : public rclcpp::Node
{
    public:

        WaypointRecorder() : Node("waypoint_recorder"), waypoint_num(0)
        {
            // Declare ROS Parameters
            waypoint_file_path = this->declare_parameter<std::string>("waypoint_file_path", std::string(std::getenv("HOME")) + "/Documents/test_waypoint_recorder.yaml");
            source_frame = this->declare_parameter<std::string>("source_frame", "map");
            target_frame = this->declare_parameter<std::string>("target_frame", "base_link");
            topic_name = this->declare_parameter<std::string>("topic_name", "odom");
            delta_distance = this->declare_parameter<double>("delta_distance", 4.0);
            delta_yaw = this->declare_parameter<double>("delta_yaw", 15.0);
            delta_chrod = this->declare_parameter<double>("delta_chrod", 1.0);
            rate = this->declare_parameter<double>("rate", 20.0);
            with_rviz = this->declare_parameter<bool>("with_rviz", false);
            from_gnss = this->declare_parameter<bool>("from_gnss", false);
            from_topic = this->declare_parameter<bool>("from_topic", false);

            waypoints[0] = Waypoint::Waypoint();

            ofs.open(waypoint_file_path);

            if (ofs.fail())
            {
                RCLCPP_ERROR(get_logger(), "The text file to save Waypoints is unavailable.");
                ofs.close();
                return; 
            } 

            if (with_rviz) {
                auto marker_pub_qos = rclcpp::QoS(10);
                this->marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", marker_pub_qos);
            }

            if (from_topic) {
                if (from_gnss) {
                    auto navsat_fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name, rclcpp::QoS(10), std::bind(&WaypointRecorder::get_fix_msg, this, std::placeholders::_1));
                } else {
                    auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(topic_name, rclcpp::QoS(10), std::bind(&WaypointRecorder::get_odom_msg, this, std::placeholders::_1));
                }
            } else {
                tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
                tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
                this->timer = this->create_wall_timer(std::chrono::milliseconds(int(1000 / rate)), std::bind(&WaypointRecorder::main_callback, this));
            }

        }

        ~WaypointRecorder()
        {
            // YAML::Node inner_node;
            YAML::Emitter out;

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

            ofs << out.c_str();
            ofs.close();
        }

    private:

        std::string waypoint_file_path;
        std::string source_frame;
        std::string target_frame;
        std::string topic_name;
        double delta_distance;
        double delta_yaw;
        double delta_chrod;
        double rate;
        bool with_rviz;
        bool from_gnss;
        bool from_topic;
        
        std::ofstream ofs;

        int waypoint_num;
        Waypoint::Waypoints waypoints;
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_sub;

        geometry_msgs::msg::TransformStamped transform;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

        rclcpp::TimerBase::SharedPtr timer;

        void main_callback()
        {
            // tf2 lookup
            try {
                RCLCPP_DEBUG(get_logger(), "Get Tranfromation between %s and %s ... \n", source_frame.c_str(), target_frame.c_str());
                transform = tf_buffer->lookupTransform(source_frame, target_frame, rclcpp::Time(0, 0, get_clock()->get_clock_type()), rclcpp::Duration(0, 0));
                RCLCPP_DEBUG(get_logger(), "Get a transform. stamp: %f, translation: [%f, %f, %f]", transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
            }
            // tf2 exception
            catch(const tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Exception when listening Transformation of Frames: %s", ex.what());
                rclcpp::sleep_for(std::chrono::seconds(1));
                return;
            }
            
            // Get an empty waypoint
            Waypoint::Waypoint waypoint = {};
            waypoint.pos_x = transform.transform.translation.x;
            waypoint.pos_y = transform.transform.translation.y;
            waypoint.pos_z = transform.transform.translation.z;
            waypoint.quat_x = transform.transform.rotation.x;
            waypoint.quat_y = transform.transform.rotation.y;
            waypoint.quat_z = transform.transform.rotation.z;
            waypoint.quat_w = transform.transform.rotation.w;

            if(waypoint_num == 0)
            {   
                tf2::Quaternion tf2_quat;
                tf2::fromMsg(transform.transform.rotation, tf2_quat);
                tf2::Matrix3x3(tf2_quat).getRPY(
                    waypoint.roll,
                    waypoint.pitch,
                    waypoint.yaw            
                );
                waypoints[waypoint_num] = waypoint;

                // Debug
                // for (int i = 0; i < 20; ++i){
                //     waypoint.pos_x = i * 3;
                //     waypoint.pos_y = i * 3;
                //     waypoints[i] = waypoint;
                // }
                
                waypoint_num++;
                return;
            }

            double d_dist = hypot(
                waypoint.pos_x - waypoints[waypoint_num - 1].pos_x,
                waypoint.pos_y - waypoints[waypoint_num - 1].pos_y
            );

            tf2::Quaternion tf2_quat;
            tf2::fromMsg(transform.transform.rotation, tf2_quat);
			tf2::Matrix3x3(tf2_quat).getRPY(
                waypoint.roll,
                waypoint.pitch,
                waypoint.yaw            
			);
                            
            double d_yaw = std::abs(
                waypoint.yaw - waypoints[waypoint_num - 1].yaw
            );

            std::stringstream ss;
            ss << "No. " << waypoint_num << " " 
                << " yaw_base " << waypoints[waypoint_num - 1].yaw * 180.0 / M_PI 
                << " yaw_now " << waypoint.yaw * 180.0 / M_PI 
                << " d_yawation " << d_yaw * 180.0 / M_PI 
                << " delta_distance " << d_dist << std::endl;
            RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
            
            if((d_yaw * 180.0 / M_PI > delta_yaw && d_dist > delta_chrod)
                || d_dist > delta_distance){	
                
                waypoints[waypoint_num] = waypoint;
                waypoint_num++;
            }
            
            if (with_rviz){
                visualization_msgs::msg::Marker marker_points;
                marker_points.header.frame_id = "map";
                marker_points.header.stamp = this->get_clock().get()->now();
                marker_points.ns = "marker_points";
                // Color
                marker_points.color.b = 1.0;
                marker_points.color.a = 1.0;
                marker_points.scale.x = 0.3;
                marker_points.scale.y = 0.3;
                marker_points.scale.z = 0.3;
                marker_points.action = visualization_msgs::msg::Marker::ADD;
                marker_points.type = visualization_msgs::msg::Marker::POINTS;
                marker_points.pose.orientation.w = 1.0;

                for (auto &&pair : waypoints) {
                    geometry_msgs::msg::Point p;
                    p.x = waypoints[pair.first].pos_x;
                    p.y = waypoints[pair.first].pos_y;
                    p.z = 0.25;
                    marker_points.points.push_back(p);
                }

                this->marker_pub->publish(marker_points);        
            }            
        }
    
        void get_odom_msg(nav_msgs::msg::Odometry::ConstSharedPtr msg){
            // Get an empty waypoint
            Waypoint::Waypoint waypoint = {};
            waypoint.pos_x = msg->pose.pose.position.x;
            waypoint.pos_y = msg->pose.pose.position.y;
            waypoint.pos_z = msg->pose.pose.position.z;
            waypoint.quat_x = msg->pose.pose.orientation.x;
            waypoint.quat_y = msg->pose.pose.orientation.y;
            waypoint.quat_z = msg->pose.pose.orientation.z;
            waypoint.quat_w = msg->pose.pose.orientation.w;

            if(waypoint_num == 0)
            {   
                tf2::Quaternion tf2_quat;
                tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
                tf2::Matrix3x3(tf2_quat).getRPY(
                    waypoint.roll,
                    waypoint.pitch,
                    waypoint.yaw            
                );
                waypoints[waypoint_num] = waypoint;

                // Debug
                // for (int i = 0; i < 20; ++i){
                //     waypoint.pos_x = i * 3;
                //     waypoint.pos_y = i * 3;
                //     waypoints[i] = waypoint;
                // }
                
                waypoint_num++;
                return;
            }

            double d_dist = hypot(
                waypoint.pos_x - waypoints[waypoint_num - 1].pos_x,
                waypoint.pos_y - waypoints[waypoint_num - 1].pos_y
            );

            tf2::Quaternion tf2_quat;
            tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
            tf2::Matrix3x3(tf2_quat).getRPY(
                waypoint.roll,
                waypoint.pitch,
                waypoint.yaw            
            );
                            
            double d_yaw = std::abs(
                waypoint.yaw - waypoints[waypoint_num - 1].yaw
            );

            std::stringstream ss;
            ss << "No. " << waypoint_num << " " 
                << " yaw_base " << waypoints[waypoint_num - 1].yaw * 180.0 / M_PI 
                << " yaw_now " << waypoint.yaw * 180.0 / M_PI 
                << " d_yawation " << d_yaw * 180.0 / M_PI 
                << " delta_distance " << d_dist << std::endl;
            RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
            
            if((d_yaw * 180.0 / M_PI > delta_yaw && d_dist > delta_chrod)
                || d_dist > delta_distance){	
                
                waypoints[waypoint_num] = waypoint;
                waypoint_num++;
            }
            
            if (with_rviz){
                visualization_msgs::msg::Marker marker_points;
                marker_points.header.frame_id = "odom";
                marker_points.header.stamp = this->get_clock().get()->now();
                marker_points.ns = "marker_points";
                // Color
                marker_points.color.b = 1.0;
                marker_points.color.a = 1.0;
                marker_points.scale.x = 0.3;
                marker_points.scale.y = 0.3;
                marker_points.scale.z = 0.3;
                marker_points.action = visualization_msgs::msg::Marker::ADD;
                marker_points.type = visualization_msgs::msg::Marker::POINTS;
                marker_points.pose.orientation.w = 1.0;

                for (auto &&pair : waypoints) {
                    geometry_msgs::msg::Point p;
                    p.x = waypoints[pair.first].pos_x;
                    p.y = waypoints[pair.first].pos_y;
                    p.z = 0.25;
                    marker_points.points.push_back(p);
                }

                this->marker_pub->publish(marker_points);        
            }
        }

        void get_fix_msg(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            Waypoint::Waypoint waypoint = {};
            waypoint.longitude = msg->longitude;
            waypoint.latitude = msg->latitude;
            waypoints[waypoint_num] = waypoint;
            waypoint_num ++;
        }
        
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();

}