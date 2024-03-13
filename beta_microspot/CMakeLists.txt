cmake_minimum_required(VERSION 3.5)
project(beta_microspot)

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(scripts)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executable script
install(PROGRAMS
  scripts/input_detector.py
  scripts/action.py
  scripts/action_2.py
  scripts/action_3.py
  scripts/inverse_kinematics.py
  scripts/trajectory_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ensure setup.py is installed if you have one
ament_package()

