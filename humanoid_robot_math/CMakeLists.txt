cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_math)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    roscpp
  )
  find_package(Eigen3 REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    DEPENDS EIGEN3
  )
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/humanoid_robot_math_base.cpp
  src/humanoid_robot_trajectory_calculator.cpp
  src/minimum_jerk_trajectory.cpp
  src/minimum_jerk_trajectory_with_via_point.cpp
  src/fifth_order_polynomial_trajectory.cpp
  src/simple_trapezoidal_velocity_profile.cpp
  src/humanoid_robot_linear_algebra.cpp
  src/bezier_curve.cpp
  src/step_data_define.cpp
  src/preview_control.cpp
)
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
