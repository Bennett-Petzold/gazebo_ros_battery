cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "srv/SetCharge.srv" 
  "srv/SetTemperature.srv"
  DEPENDENCIES std_msgs
  )


include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(battery_plugin SHARED src/gz_ros_battery.cc)
add_executable(test_clients src/test_clients.cc)

ament_export_dependencies(rosidl_default_runtime)
ament_target_dependencies(battery_plugin rclcpp sensor_msgs std_msgs gazebo_ros)
ament_target_dependencies(test_clients rclcpp sensor_msgs std_msgs gazebo_ros)


rosidl_target_interfaces(battery_plugin
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(test_clients
  ${PROJECT_NAME} "rosidl_typesupport_cpp")



target_link_libraries(battery_plugin ${GAZEBO_LIBRARIES})

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

install(TARGETS
  test_clients
  DESTINATION lib/${PROJECT_NAME})

ament_package()
