cmake_minimum_required(VERSION 3.8)
project(alfred_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Specify include directories
include_directories(
  include
  src/tag_utils/tag_structs
)


# Declare your executable target
add_executable(alfred_perception 
  src/mainPerception.cpp
  src/AprilTagDetector_class.cpp
  src/TagPerception_class.cpp
  src/tag_utils/tag_structs/TagDetection_struct.cpp
  )

# Link the executable to the required ROS2 libraries
ament_target_dependencies(alfred_perception rclcpp std_msgs sensor_msgs cv_bridge OpenCV apriltag)
target_link_libraries(alfred_perception apriltag)


# Add Executables to directory
install(TARGETS
  alfred_perception
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY src/tag_utils/tag_structs/
  DESTINATION src/tag_utils/tag_structs/
)

# Install header files
# install(DIRECTORY include/
#   DESTINATION include/
# )


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
