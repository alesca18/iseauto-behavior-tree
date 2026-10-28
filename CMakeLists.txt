cmake_minimum_required(VERSION 3.8)
project(iseauto_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(brain_node src/brain_node.cpp)
ament_target_dependencies(brain_node
  ament_index_cpp rclcpp std_msgs geometry_msgs behaviortree_cpp_v3)

install(TARGETS brain_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/metadrive_bridge.py
  scripts/scenario.py
  scripts/train_ppo.py
  scripts/eval_ppo.py
  scripts/kill_ghosts.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt_trees launch DESTINATION share/${PROJECT_NAME})

ament_package()
