cmake_minimum_required(VERSION 3.8)
project(tableorder)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Define service files
set(srv_files
  "srv/MenuOrder.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# Install Python scripts
install(
  PROGRAMS
    tableorder/table_server.py
    tableorder/table_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies for runtime
ament_export_dependencies(rosidl_default_runtime)

# Finalize package
ament_package()
