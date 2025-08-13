cmake_minimum_required(VERSION 3.8)
project(kuka_lbr_iiwa14_marlab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/data")
add_compile_definitions(OUTPUT_DIRECTORY="${OUTPUT_DIRECTORY}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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


# Check whether VCPKG is set up in your system
if(NOT DEFINED ENV{VCPKG_ROOT})
  message(FATAL_ERROR "ERROR: This project requires VCPKG.\n")
endif()

# Set VCPKG root directory
set(VCPKG_ROOT $ENV{VCPKG_ROOT})

# Include VCPKG toolchain
include(${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

# My dependencies
find_package(glfw3 REQUIRED)
cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(unofficial-imgui-node-editor CONFIG REQUIRED)
find_package(imgui-platform-kit REQUIRED)
find_package(dynamic-neural-field-composer REQUIRED)


# Add control_architecture_node executable
add_executable(dnf_control_architecture src/dnf_control_architecture.cpp)
ament_target_dependencies(dnf_control_architecture rclcpp std_msgs)
target_link_libraries(dnf_control_architecture
  imgui::imgui
  implot::implot
  nlohmann_json::nlohmann_json
  OpenGL::GL           # prefer the imported target
  glfw                 # or glfw3, depending on your package
  imgui-platform-kit
  dynamic-neural-field-composer
)
install(TARGETS dnf_control_architecture DESTINATION lib/${PROJECT_NAME})

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
