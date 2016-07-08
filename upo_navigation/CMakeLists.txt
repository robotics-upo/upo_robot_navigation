cmake_minimum_required(VERSION 2.8.3)
project(upo_navigation)
set (CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED
    COMPONENTS
		angles
        cmake_modules
		costmap_2d
		#dynamic_reconfigure
		geometry_msgs
  		nav_msgs
        roscpp
        pluginlib
        actionlib
		rosconsole
  		roscpp
        message_generation
        nav_core
        tf
		rospy
  		std_msgs
  		visualization_msgs
  		voxel_grid
  		upo_msgs
		#social_cost_library
		upo_rrt_planners
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED
    COMPONENTS
        thread
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})


# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/upo_navigation.cfg
#)


## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    Position2DInt.msg
	PointCost.msg
	FeaturesWeights.msg
 )


add_service_files(
   FILES
   PointCosts.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES 
		upo_navigation
    CATKIN_DEPENDS
        angles
        cmake_modules
		costmap_2d
		geometry_msgs
  		nav_msgs
        roscpp
        pluginlib
        actionlib
		rosconsole
  		roscpp
		message_runtime 
        nav_core
        tf
		rospy
  		std_msgs
  		visualization_msgs
  		voxel_grid
  		upo_msgs
		upo_rrt_planners
	#DEPENDS system_lib
)


# upo_navigation
add_library(upo_navigation
	src/pp_controller/trajectory_planner_pp.cpp
	#src/upo_rrt/gmm_sampling.cpp
    #src/upo_rrt/FeatureFunctions.cpp
	#src/upo_rrt/RRTstar.cpp
	#src/upo_rrt/InformedRRTstar.cpp
	#src/upo_rrt/rrt_planner_ros.cpp
	src/footprint_helper.cpp
	src/goal_functions.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/map_grid_cost_function.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/point_grid.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
	src/voxel_grid_model.cpp
  	src/upo_navigation.cpp
)

add_dependencies(upo_navigation upo_navigation_gencpp)
add_dependencies(upo_navigation upo_msgs_gencpp)
add_dependencies(upo_navigation geometry_msgs_gencpp)
add_dependencies(upo_navigation nav_msgs_gencpp)

target_link_libraries(upo_navigation
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
	${OpenCV_LIBRARIES}
	upo_rrt_planners_ros
    )

# Regular navigation
add_executable(upo_navigation_node
  src/upo_navigation_node.cpp
)
target_link_libraries(upo_navigation_node upo_navigation upo_rrt_planners_ros)
set_target_properties(upo_navigation_node PROPERTIES OUTPUT_NAME upo_navigation_node)

add_dependencies(upo_navigation_node ${catkin_EXPORTED_TARGETS})





#install(
#    TARGETS
#        upo_navigation
#        upo_navigation_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
