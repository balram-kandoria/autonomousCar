cmake_minimum_required(VERSION 3.8)
project(alfred_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(alfred_interfaces REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(message_filters REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Specify include directories
include_directories(
  include
  src/cfg
  src/nav_utils/nav_structs
)

# Declare your executable target
add_executable(alfred_navigation 
  src/mainNavigation.cpp
  src/worldTransforms_class.cpp
  src/staticTransform_class.cpp
  src/dynamicTransforms_class.cpp
  src/worldMap_class.cpp
  src/nav_utils/nav_structs/StaticTransform_struct.cpp
  )

# Link the executable to the required ROS2 libraries
ament_target_dependencies(alfred_navigation rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros tf2 yaml-cpp alfred_interfaces tf2_geometry_msgs nav_msgs message_filters) 
target_link_libraries(alfred_navigation yaml-cpp)

# Add Executables to directory
install(TARGETS
  alfred_navigation
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.h"
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
