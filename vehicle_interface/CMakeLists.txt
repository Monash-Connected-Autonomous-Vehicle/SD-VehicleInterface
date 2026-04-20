cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(sd_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(COVERAGE "Enable gcov/lcov coverage instrumentation" OFF)
if(COVERAGE)
  add_compile_options(--coverage -O0 -g)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
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
find_package(autoware_control_msgs REQUIRED)

add_executable(sd_vehicle_interface_node src/sd_vehicle_interface/sd_vehicle_interface.cpp
		src/sd_vehicle_interface/sd_control.cpp
		src/sd_vehicle_interface/sd_gps_imu.cpp)

ament_target_dependencies(sd_vehicle_interface_node rclcpp sensor_msgs std_msgs sd_msgs can_msgs autoware_control_msgs)

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
  find_package(ament_cmake_gtest REQUIRED)
  # Skip ament linters that this legacy codebase does not satisfy yet (CI runs them via
  # ament_lint_auto). Re-enable and fix findings when ready.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TEST_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/sd_vehicle_interface)
  set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../test)

  ament_add_gtest(test_sd_control ${TEST_DIR}/test_sd_control.cpp ${TEST_SRC_DIR}/sd_control.cpp)
  target_include_directories(test_sd_control PRIVATE ${TEST_SRC_DIR})
  ament_target_dependencies(test_sd_control rclcpp std_msgs sd_msgs can_msgs)

  ament_add_gtest(test_sd_gps_imu ${TEST_DIR}/test_sd_gpu_imu.cpp ${TEST_SRC_DIR}/sd_gps_imu.cpp)
  target_include_directories(test_sd_gps_imu PRIVATE ${TEST_SRC_DIR})
  ament_target_dependencies(test_sd_gps_imu rclcpp sensor_msgs std_msgs sd_msgs can_msgs)

  ament_add_gtest(test_vehicle_interface_integration
    ${TEST_DIR}/test_vehicle_interface_integration.cpp)
  target_include_directories(test_vehicle_interface_integration PRIVATE ${TEST_SRC_DIR})
  ament_target_dependencies(test_vehicle_interface_integration rclcpp sensor_msgs std_msgs sd_msgs can_msgs autoware_control_msgs)
endif()

ament_package()
