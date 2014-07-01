cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor_calib)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs control_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface kdl_conversions kdl_wrapper kdl_acc_solver)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  DEPENDS eigen
 CATKIN_DEPENDS roscpp sensor_msgs control_msgs tf geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface kdl_conversions kdl_wrapper kdl_acc_solver
  INCLUDE_DIRS include 
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(ft_calib_node src/ft_calib.cpp src/ft_calib_node.cpp)
target_link_libraries(ft_calib_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} orocos-bfl)

install(TARGETS ft_calib_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)