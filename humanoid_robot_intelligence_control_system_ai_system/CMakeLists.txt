# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
# 
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_ai_system)

# Find ROS
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
  )
else()
  find_package(ament_cmake REQUIRED COMPONENTS
    rclcpp
    rclpy
  )
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    rospy
  )
endif()

# Install Python scripts for both ROS 1
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_install_python(
    PROGRAMS
    src/humanoid_robot_intelligence_control_system_response.py
    src/humanoid_robot_intelligence_control_system_command.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# Install Python scripts, configuration files, and launch files
if($ENV{ROS_VERSION} EQUAL "1")
    install(PROGRAMS src/humanoid_robot_intelligence_control_system_response.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(PROGRAMS src/humanoid_robot_intelligence_control_system_command.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
    install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
elseif($ENV{ROS_VERSION} EQUAL "2")
    install(PROGRAMS src/humanoid_robot_intelligence_control_system_response.py DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS src/humanoid_robot_intelligence_control_system_command.py DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
    ament_package()
endif()

