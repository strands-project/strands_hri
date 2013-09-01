# Strands gazing package
Since gazing is one important part of human-robot interaction this package will provide general applications to perform gazing behaviours.

## Gaze at pose
This node takes a give geometry_msgs/PoseArry and moves the head to gaze at the closest of these poses.

### Parameters
* `pose_array` _Default: /gaze_at_pose/pose_array_: The subsribed topic for the incoming poses.
* `head_pose` _Default: /head/commanded_state_: The topic to which the joint state message for the head position are published.
* `head_frame` _Default: /head_base_frame_: The target coordinate frame.

### Running
```
rosrun strands_gazing gaze_at_pose [_parameter:=value]
```
