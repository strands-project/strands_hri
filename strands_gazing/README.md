# Strands gazing package
Since gazing is one important part of human-robot interaction this package will provide general applications to perform 
gazing behaviours.

## Gaze at pose
This node takes a given geometry_msgs/PoseArry and moves the head to gaze at the closest of these poses. This is 
implemented as an action server.

### Parameters
* `pose_array` _Default: /gaze_at_pose/pose_array_: The subsribed topic for the incoming poses.
* `head_pose` _Default: /head/commanded_state_: The topic to which the joint state messages for the head position are 
published.
* `head_frame` _Default: /head_base_frame_: The target coordinate frame.

### Running
```
rosrun strands_gazing gaze_at_pose [_parameter:=value]
```
To start the actual gazing you have to use the actionlib client architecture, 
e.g. `rosrun actionlib axclient.py /gaze_at_pose`
* goal
 * `int32 runtime_sec`: The time the gazing should be executed in seconds. Set to 0 for infinit run time (has to be 
cancelled to stop). _Cancelling a goal will trigger the head to move to the zero position._
* result
 * `bool expired`: true if run time is up, false if cancelled.
* feedback
 * `float32 remaining_time`: The remaining run time.
 * `geometry_msgs/Pose target`: The pose at which the robot currently gazes.


