# // 三次元座標系と四元数座標系の座標値
# // 四元数とは、URL：https://ja.wikipedia.org/wiki/%E5%9B%9B%E5%85%83%E6%95%B0
# // 「geometry_msgs::msg::Pose」の詳細について、URL：https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
geometry_msgs/Pose pose
sensor_msgs/NavSatFix fix

# // RPY
std_msgs/Float64 raw
std_msgs/Float64 pitch
std_msgs/Float64 yaw

# // モード
std_msgs/Int8 mode