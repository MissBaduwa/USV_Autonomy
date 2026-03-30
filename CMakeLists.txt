cmake_minimum_required(VERSION 3.8)
project(usv_autonomy)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Debris.msg"
  "msg/MissionCommand.msg"
  "msg/SystemStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
install(
  DIRECTORY usv_autonomy/
  DESTINATION lib/python3.12/site-packages/
)

# Install RViz config file
install(
  FILES demo.rviz
  DESTINATION share/${PROJECT_NAME}
)
# Install the node scripts as executables
install(
  PROGRAMS 
    usv_autonomy/perception_node.py
    usv_autonomy/navigation_node.py
    usv_autonomy/test_debris_publisher.py
    usv_autonomy/simulation_node.py 
    usv_autonomy/supervisor_node.py
    usv_autonomy/camera_simulator.py 
    usv_autonomy/test_sensors.py
    usv_autonomy/visualization_node.py
    usv_autonomy/launch_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# Install the node scripts as executables


