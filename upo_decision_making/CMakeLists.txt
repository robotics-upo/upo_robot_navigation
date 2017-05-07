cmake_minimum_required(VERSION 2.8.3)
project(upo_decision_making)

SET ( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules )

find_package(catkin REQUIRED dynamic_reconfigure 
                             message_generation
                             actionlib_msgs
                             actionlib )

#find_package( MADP )

add_message_files(
   FILES
   ControlEvent.msg
   IDArray.msg
   HRIControlMode.msg
   HRIFeedbackEvent.msg
   HRIFeedbackFromInterface.msg
   HRIFeedbackStateInfo.msg
)

#add_service_files(
#   FILES
#   TestPolicy.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#   InformedLookAround.action
#   LookAround.action
#   LookForInteractionTargets.action
#   MaintainHeadingToInteractionTargets.action
#   MaintainInteractionRange.action
#   MoveToInteractionRange.action
#   StabilizeHeadPose.action
#   RepositionForConversation.action
#   TurnToInteractionTargets.action
#   NNPolicy.action
#)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/HRIActionParams.cfg
#  cfg/PIDParams.cfg
#)

catkin_package(
  CATKIN_DEPENDS #mdm_library
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#link_directories(/usr/local/lib/)

#if ( MADP_FOUND )
#  add_executable(mopomdp_action_layer src/mopomdp_action_layer.cpp)
#  target_link_libraries(mopomdp_action_layer mdm_action_layer ${catkin_LIBRARIES})

#  add_executable(mopomdp_control_layer src/mopomdp_control_layer.cpp)
#  target_link_libraries(mopomdp_control_layer mdm_control_layer MADPBase ${catkin_LIBRARIES})

#endif ( MADP_FOUND )
