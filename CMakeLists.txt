cmake_minimum_required(VERSION 2.8.3)
project(hydro_landings)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  geometry_msgs
  tf
  tf_conversions
)
  
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
	DEPENDS Eigen
)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(velocity_cmd src/velocity_cmd.cpp)

add_dependencies(velocity_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(velocity_cmd
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
