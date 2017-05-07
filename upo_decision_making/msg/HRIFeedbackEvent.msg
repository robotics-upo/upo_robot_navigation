#This message provides information about instantaneous detections of HRI-specific feedback events

#standard header, can include frame information
std_msgs/Header header

#The confidence with which the event was detected
float32 confidence

#The type of the event that was detected
uint8 type


uint8 CONVSUB_TOUCHING_EARS=0
uint8 CONVSUB_LEANING_DETECTION=1
uint8 PILOT_FROWN=3
uint8 BACKGROUND_NOISE_TOO_LOUD=4
