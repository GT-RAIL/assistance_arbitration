# Local Strategy

This package coordinates the different methods of soliciting help from a local human (once such a person is identified).

The current method of soliciting help:

1. Looks around for a person (potentially forever)
1. Enters compliant mode - all joints can be moved by a human.
1. When a person is found, starts a dialogue:
    1. Tells the person what failed and the likely cause of that failure.
    1. Can respond to questions of what else might have failed.
    1. Can explain that safety is the reason we're in compliant mode.
1. When person indicates that the problem should be resolved, chirp happily and return.


## Notes

The different keys and values in the errors that are sent with the assistance request are:

- **task** - name of the task
- **action** - name of action
- **status** - GoalStatus code of action client
- **context** - the pickled details from the lower levels of the tree
- **goal** - transformed version of argument sent to run
    - arm:pose - the pose of the arm that failed
    - beep:beep - name of the beep key
    - find_closest_person:timeout - the max amount of time, float, within which to find a person
    - find_grasps:segmented_obj - of type rail_manipulation_msgs/SegmentedObject
    - find_object:obj - the string name of the object in the DB
    - gripper:command - open/close command sent
    - listen:expected_cmd - the speech command that we were expecting
    - look:pose - dictionary of the pose to look at
    - move:coord - the waypoint coordinate that failed
    - pick:cube_idx - the index of the cube in the segmented objects list
    - speak:text - the text to speak
    - torso:heigh - the height in float to raise the torso to
- **result** -  action client result
    - arm:PresetJointsMoveResult - bool success, int MoveItErrorCode
    - beep:SoundRequestResult - bool playing, time stamp
    - result:GripperCommandResult - position, effort, bool stalled, bool reached_goal
    - look:PointHeadResult - empty
    - move:MoveBaseResult - empty
    - pick:ExecuteGraspResult - bool success, int MoveItErrorCode, int failure_point in grasp execution
    - speak:SoundRequestResult - bool playing, time stamp
    - torso:FollowJointTrajectoryResult - int error_code, string error_string
- **srv** - name of the service that failed
    - find_grasps:suggester/suggest_grasps
    - find_grasps:suggester/pairwise_rank
    - find_object:rail_segmentation/segment_objects
    - find_object:grasp_executor/clear_objects
    - find_object:grasp_executor/add_object
    - place:grasp_executor/drop_object
- **attempt_num** - idx of the arm movement attempt when run stopped. Only arm
- **num_grasps** - number of grasps found be suggester/suggest_grasps. Only find_grasps and pick
- **segmented_objects** - of type rail_manipulation_msgs/SegmentedObjectList; segmented objects returned by RAIL segmentation. Only find_object
- **found_idx** - index of the object found in the SegmentedObjectList based on data in the object DB. Only find_object
- **found_obj** - of type rail_manipulation_msgs/SegmentedObject; object with idx found_idx in the SegmentedObjectLst. Only find_object
- **coord_num** - the index of the waypoint that we were moving to when we failed. Only move
- **grasp_num** - the index of the grasp that we were attempting when we failed. Only pick
- **grasps** - all the grasps that we were planning to/did attempt. Only pick
- **affect** - the affect key that we want the text to be said with. Only speak
- **step_idx** - the index of the task step that was executing when we exited. Only task
- **step_name** - the name of the task step that was executing when we exited. Only task
- **search_duration** - the duration (in seconds, float) spent searching for the nearest person. Only find_closest_person
- **received_cmd** - the command received from the speech listener. Only listen
