# A message type to provide information about actions taken during an
# intervention

# The name of the action
string LOOK_UP=look_up
string LOOK_DOWN=look_down
string LOOK_LEFT=look_left
string LOOK_RIGHT=look_right

string MOVE_FORWARD=move_forward
string MOVE_BACKWARD=move_backward
string MOVE_LEFT=move_left
string MOVE_RIGHT=move_right
string MOVE_WAYPOINT=move_waypoint

string RELOCALIZE=relocalize

string TORSO_UP=torso_up
string TORSO_DOWN=torso_down

string ARM_POSITION=arm_position
string ARM_LINEAR_UP=arm_linear_up
string ARM_LINEAR_DOWN=arm_linear_down
string ARM_LINEAR_LEFT=arm_linear_left
string ARM_LINEAR_RIGHT=arm_linear_right
string ARM_LINEAR_FORWARD=arm_linear_forward
string ARM_LINEAR_BACKWARD=arm_linear_backward

string CROP_IMAGE=crop_image
string SEGMENT_IMAGE=segment_image

string UPDATE_BELIEFS=update_beliefs

string TRY_FAILED_ACTION=try_failed_action

string type

# Args to the action. This is a pickled byte array
string args
