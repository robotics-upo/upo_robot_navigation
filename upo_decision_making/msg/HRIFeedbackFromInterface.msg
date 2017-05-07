#This message encodes feedback information provided by the pilot through the Giraff interface

#standard header, can include frame information
std_msgs/Header header

#The type of feedback or command provided by the pilot
uint8 type

#serialized data
uint8[] data

# WARNING: DO NOT CHANGE THESE IDs (only add new ones)
# THE INTERFACE PROTOCOL DEPENDS ON THEM
uint8 NAV_TO_WAYPOINT_REQUESTED=0
uint8 NAV_TO_PERSON_REQUESTED=1
uint8 CALL_OVER=2
uint8 WALK_WITH_PERSON_REQUESTED=3
uint8 MANUAL_CONTROL_REQUESTED=4
uint8 AUTONOMOUS_CONTROL_REQUESTED=5
uint8 VOLUME_CHANGE_REQUESTED=6
uint8 HEARING_PROBLEMS_INDICATED=7
