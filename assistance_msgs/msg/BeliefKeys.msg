# The different beliefs that can be updated by any node
# Ensure that the value of the message is a lowercase
# version of the key. Some of our task code assumes this
# to be the case

#### TASK state beliefs ####

# State of the large gear
string LARGE_GEAR_IN_SCHUNK=large_gear_in_schunk
string LARGE_GEAR_IN_KIT=large_gear_in_kit

# State of the small gear
string SMALL_GEAR_IN_KIT=small_gear_in_kit

# State of the gearboxes
string GEARBOX_TOP_IN_KIT=gearbox_top_in_kit
string GEARBOX_BOTTOM_IN_KIT=gearbox_bottom_in_kit

# State of the bolts
string ZERO_BOLTS_IN_KIT=zero_bolts_in_kit
string ONE_BOLT_IN_KIT=one_bolt_in_kit
string TWO_BOLTS_IN_KIT=two_bolts_in_kit

# State of the SCHUNK
string SCHUNK_IS_MACHINING=schunk_is_machining
string SCHUNK_IS_READY=schunk_is_ready

# Location of the robot based on knowledge of move tasks
# string TASK_AT_GEAR_PICK_STATION=task_at_gear_pick_station
# string TASK_AT_KIT_STATION=task_at_kit_station
# string TASK_AT_SCREW_BIN_PICK_STATION=task_at_screw_bin_pick_station
# string TASK_AT_GEARBOX_PICK_STATION=task_at_gearbox_pick_station
# string TASK_AT_DROPOFF=task_at_dropoff
# string TASK_AT_SCHUNK_MANIPULATION=task_at_schunk_manipulation

string TASK_AT_KITCHEN_COUNTER=task_at_kitchen_counter
string TASK_AT_DINING_TABLE=task_at_dining_table
string TASK_AT_COUCH=task_at_couch

#### ROBOT state beliefs ####

# Location of the robot
string ROBOT_AT_NAVIGATION_GOAL=robot_at_navigation_goal
# string ROBOT_AT_DROPOFF=robot_at_dropoff
# string ROBOT_AT_SCHUNK_MANIPULATION=robot_at_schunk_manipulation
# string ROBOT_AT_GEAR_PICK_STATION=robot_at_gear_pick_station
# string ROBOT_AT_KIT_STATION=robot_at_kit_station
# string ROBOT_AT_SCREW_BIN_PICK_STATION=robot_at_screw_bin_pick_station
# string ROBOT_AT_GEARBOX_PICK_STATION=robot_at_gearbox_pick_station

string ROBOT_AT_KITCHEN_COUNTER=robot_at_kitchen_counter
string ROBOT_AT_DINING_TABLE=robot_at_dining_table
string ROBOT_AT_COUCH=robot_at_couch
