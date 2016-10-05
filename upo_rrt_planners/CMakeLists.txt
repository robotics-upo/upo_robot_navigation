cmake_minimum_required(VERSION 2.8.3)
project(upo_rrt_planners)
set (CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED
    COMPONENTS
		angles
        cmake_modules
		costmap_2d
		geometry_msgs
  		nav_msgs
        roscpp
		rosconsole
        tf
  		std_msgs
  		visualization_msgs
  		upo_msgs
		navigation_features
		dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED
#    COMPONENTS
#        thread
#)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
	#${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/RRTRosWrapper.cfg
)


## Generate messages in the 'msg' folder
#add_message_files(
#    DIRECTORY msg
#    FILES
#    Position2DInt.msg
#	PointCost.msg
#	FeaturesWeights.msg
# )


#add_service_files(
#   FILES
#   PointCosts.srv
#)

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES 
		upo_rrt_planners
		upo_rrt_planners_ros
    CATKIN_DEPENDS
        angles
        cmake_modules
		costmap_2d
		geometry_msgs
  		nav_msgs
        roscpp
		rosconsole
        tf
  		std_msgs
  		visualization_msgs
  		upo_msgs
		navigation_features
		dynamic_reconfigure
	#DEPENDS system_lib
)


# upo_rrt_planners (ROS independent)
add_library(upo_rrt_planners
	src/planners/Planner.cpp
	src/planners/simple/SimpleRRT.cpp
	src/planners/simple/SimpleRRTstar.cpp
	src/planners/control/RRT.cpp
	src/planners/control/RRTstar.cpp
	src/planners/control/HalfRRTstar.cpp
	src/Action.cpp
	src/State.cpp
	src/Node.cpp
	src/StateSpace.cpp
	src/RandomNumbers.cpp
	src/steering/Steering.cpp
)

add_dependencies(upo_rrt_planners upo_rrt_planners_gencpp)


target_link_libraries(upo_rrt_planners
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	flann
    )



# upo_rrt_planners (ROS wrapper)
add_library(upo_rrt_planners_ros
	src/ros/ValidityChecker.cpp
	src/ros/RRT_ros_wrapper.cpp
)

add_dependencies(upo_rrt_planners_ros upo_rrt_planners_gencpp)
add_dependencies(upo_rrt_planners_ros upo_rrt_planners_ros_gencpp)
add_dependencies(upo_rrt_planners_ros upo_msgs_gencpp)
add_dependencies(upo_rrt_planners_ros geometry_msgs_gencpp)
add_dependencies(upo_rrt_planners_ros nav_msgs_gencpp)
add_dependencies(upo_rrt_planners_ros navigation_features_gencpp)

target_link_libraries(upo_rrt_planners_ros
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	nav_features
	upo_rrt_planners
    )




install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )


install(
    TARGETS
        upo_rrt_planners
		upo_rrt_planners_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
