cmake_minimum_required(VERSION 3.8)
project(smart_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smartcar_msgs REQUIRED)

# ---- Install Python executables ----
install(PROGRAMS
  scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry
)

install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_publisher
)

# ---- Install launch, URDF, RViz, and world ----
install(FILES
  launch/gazebo.launch.py
  launch/smartcar.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf rviz world nav2_map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

