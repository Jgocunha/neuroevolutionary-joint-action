cmake_minimum_required(VERSION 3.8)
project(kuka_lbr_iiwa14_marlab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(joint_control_demo src/joint_control_demo.cpp)
ament_target_dependencies(joint_control_demo 
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs
)
install(TARGETS joint_control_demo DESTINATION lib/${PROJECT_NAME})

add_executable(moveit2_control_demo src/moveit2_control_demo.cpp)
ament_target_dependencies(moveit2_control_demo 
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS moveit2_control_demo DESTINATION lib/${PROJECT_NAME})

add_executable(mock_pick_place src/mock_pick_place.cpp)
ament_target_dependencies(mock_pick_place 
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS mock_pick_place DESTINATION lib/${PROJECT_NAME})

add_executable(mock_pick_place_joint_control src/mock_pick_place_joint_control.cpp)
ament_target_dependencies(mock_pick_place_joint_control 
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS mock_pick_place_joint_control DESTINATION lib/${PROJECT_NAME})

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

# Launch mixins
ament_python_install_package(bringup_py)

ament_package()
