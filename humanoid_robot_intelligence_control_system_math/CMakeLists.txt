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
project(humanoid_robot_intelligence_control_system_math)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
  )
  find_package(Eigen3 REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    DEPENDS EIGEN3
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(Eigen3 REQUIRED)

  include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
endif()

add_library(
  ${PROJECT_NAME}
  src/humanoid_robot_intelligence_control_system_math_base.cpp
  src/humanoid_robot_intelligence_control_system_trajectory_calculator.cpp
  src/minimum_jerk_trajectory.cpp
  src/minimum_jerk_trajectory_with_via_point.cpp
  src/fifth_order_polynomial_trajectory.cpp
  src/simple_trapezoidal_velocity_profile.cpp
  src/humanoid_robot_intelligence_control_system_linear_algebra.cpp
  src/bezier_curve.cpp
  src/step_data_define.cpp
  src/preview_control.cpp
)

if($ENV{ROS_VERSION} EQUAL 1)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
else()
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
  )
  target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(rclcpp Eigen3)

  ament_package()
endif()
