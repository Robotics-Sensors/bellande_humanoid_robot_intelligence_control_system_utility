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
  
  catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    rospy
  )
  
  catkin_install_python(
    PROGRAMS
    src/humanoid_robot_intelligence_control_system_response.py
    src/humanoid_robot_intelligence_control_system_command.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  install(
    PROGRAMS 
    src/humanoid_robot_intelligence_control_system_response.py
    src/humanoid_robot_intelligence_control_system_command.py
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  
  install(
    DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
  
  install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
  
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclpy REQUIRED)
  
  install(
    PROGRAMS 
    src/humanoid_robot_intelligence_control_system_response.py
    src/humanoid_robot_intelligence_control_system_command.py
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  
  ament_package()
endif()
