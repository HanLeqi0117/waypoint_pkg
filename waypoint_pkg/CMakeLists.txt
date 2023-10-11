cmake_minimum_required(VERSION 3.8)
project(waypoint_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(waypoint_pkg_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

# 実行ファイルを生成する
add_executable(waypoint_maker src/twist2txt.cpp)
add_executable(rewaypoint_loader src/txt2marker.cpp)
add_executable(waypoint_editor src/marker_server.cpp)
# add_executable(test_tf2_broadcaster src/test_tf2_broadcaster.cpp)

# インクルードターゲットを追加する
# このパッケージにおいて、「./include」ディレクトリはターゲットになる
target_include_directories(waypoint_maker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(rewaypoint_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(waypoint_editor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(waypoint_maker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(rewaypoint_loader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(waypoint_editor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  waypoint_maker
  "rclcpp"
  "waypoint_pkg_interfaces"
  "interactive_markers"
  "std_srvs"
  "std_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "tf2_ros"
  "turtlesim"
)
ament_target_dependencies(
  rewaypoint_loader
  "rclcpp"
  "waypoint_pkg_interfaces"
  "interactive_markers"
  "std_srvs"
  "std_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "tf2_ros"
  "turtlesim"
)
ament_target_dependencies(
  waypoint_editor
  "rclcpp"
  "waypoint_pkg_interfaces"
  "interactive_markers"
  "std_srvs"
  "std_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "tf2_ros"
  "turtlesim"
)


install(
  TARGETS waypoint_maker rewaypoint_loader waypoint_editor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
