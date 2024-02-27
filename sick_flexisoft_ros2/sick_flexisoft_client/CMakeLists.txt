cmake_minimum_required(VERSION 3.5)

project(sick_flexisoft_client)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Define the include directories
include_directories(include)

# Create a shared library for the project
add_library(${PROJECT_NAME}
  src/flexiclient.cpp
)

# Link the library with ROS 2 libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install other files if needed
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Export information to downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})


ament_package()
