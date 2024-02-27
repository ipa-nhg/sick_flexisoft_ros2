cmake_minimum_required(VERSION 3.5)

project(cob_safety_controller)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cob_msgs REQUIRED)
find_package(cob_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sick_flexisoft_client REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
#find_package(dyn_interfaces REQUIRED) # Dynamic interfaces for ROS 2 dynamic_reconfigure
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    chrono
    system
    thread
)

# Declare the package dependencies
set(DEPENDENCIES
    rclcpp
    cob_msgs
    cob_srvs
    diagnostic_msgs
    diagnostic_updater
    # dynamic_reconfigure
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    sick_flexisoft_client
    tinyxml2_vendor
    TinyXML2
)

# Include directories
include_directories("/home/cyz1999unistuttgart/ros2_ws/src/cob_common/cob_msgs")
include_directories("/home/cyz1999unistuttgart/ros2_ws/src/cob_common/cob_srvs")
include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

# Generate dynamic_reconfigure options (equivalent to generate_dynamic_reconfigure_options)
# ament_auto_generate_dynamic_reconfigure_options(cfg/safety_controller.cfg)
install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}
)

# Create a library
add_library(laser_fields common/src/laser_fields.cpp)
ament_target_dependencies(laser_fields ${DEPENDENCIES})

# Create an executable
add_executable(safety_controller ros/src/safety_controller_ros.cpp)
ament_target_dependencies(safety_controller ${DEPENDENCIES})
target_link_libraries(safety_controller laser_fields)
target_link_libraries(safety_controller ${Boost_LIBRARIES})
target_link_libraries(safety_controller ${tinyxml2_vendor_LIBRARIES})
target_link_libraries(safety_controller ${TinyXML2_LIBRARIES})

# Install targets
install(TARGETS safety_controller laser_fields
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  ros/src/flexisoft_sim.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install configuration files if any
# install(FILES my_configuration_file.yaml
#   DESTINATION share/${PROJECT_NAME}
# )

# Export information to downstream packages
ament_export_include_directories(common/include)
ament_export_libraries(laser_fields safety_controller)
ament_export_dependencies(${DEPENDENCIES})

# Make the package available to other packages
ament_package()
