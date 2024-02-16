cmake_minimum_required(VERSION 3.5)
project(hamal_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(collision_monitor REQUIRED)

# Install
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
