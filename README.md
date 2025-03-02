# autonomousCar
An autonomous driving vehicle

# Compiling Instructions
## ROS2 Compiling
Ensure the current working directory is ros2_ws
'''
colcon build --symlink-install
'''
## C++ Compiling
Ensure the current working directory is in the src file of the package being tested
'''
g++ *.cpp -o output
'''