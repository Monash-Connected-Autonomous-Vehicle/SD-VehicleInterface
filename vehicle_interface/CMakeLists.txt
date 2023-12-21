cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(sd_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sd_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

add_executable(sd_vehicle_interface_node src/sd_vehicle_interface/sd_vehicle_interface.cpp
		src/sd_vehicle_interface/sd_control.cpp
		src/sd_vehicle_interface/sd_gps_imu.cpp)

ament_target_dependencies(sd_vehicle_interface_node rclcpp sensor_msgs std_msgs sd_msgs can_msgs autoware_auto_control_msgs)

install(TARGETS
sd_vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
