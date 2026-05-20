cmake_minimum_required(VERSION 3.8)
project(puzzlebot_real_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(PROGRAMS
    scripts/puzzlebot_joint_state_publisher.py
    scripts/puzzlebot_localization.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
