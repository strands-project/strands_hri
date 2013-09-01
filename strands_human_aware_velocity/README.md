# Human aware velocities
This package adjusts the velocity of the robot in the presence of humans.

## Human aware planner velocity
This node adjusts the velocity move_base/DWAPlannerROS uses to move the robot to a goal. This is achieved by using the dynamic reconfigure callback of the DWAPlannerROS to set the `max_vel_x`, `max_rot_vel`, and `max_trans_vel`. As a consequence the robot will not be able to move linear or angular if a human is too close and will graduately slow down when approaching humans. The rotation speed is only adjusted to prevent the robot from spinning in place when not beeing able to move forward because this behaviour was observed even when the clearing rotation recovery behaviour is turnd off.

This is implemented asa an action server.

### Parameters
* `pedestrian_locations` _Default: /pedestrian_localisation/localisations_: The topic on which the actual pedestrian locations are published by the pedestrian localisation (strands_perception_people_msgs/PedestrianLocations).

### Running
```
rosrun strands_human_aware_velocity human_aware_planner_velocity.py
```
To start the actual functionality you have to use the actionlib client architecture, e.g. `rosrun actionlib axclient.py /human_aware_planner_velocities`
* goal _Cancelling a goal will reset the speeds to the given max values._
 * `int32 seconds`: Run time of the functionality. Set to 0 for infinit runtime (has to be cancelled to stop).
 * `float32 max_vel_x`: Maximum translation velocity.
 * `float32 max_rot_vel`: Maximum rotation velocity.
 * `float32 max_dist`: Maximum distance of detected humans. Everything above will be ignored and the max_vel_x value will be used.
 * `float32 min_dist`: The distance to a human at which the robot will stop (have 0.0 velocity, it might stop before reaching 0.0 according to your min_trans_vel value).
 * `float32 time_to_reset`: The time it takes for no human to be detected to resume to 'max_vel_x' speed.
* result
 * `bool expired`: true if run time is up, false if cancelled.
* feedback
 * `int32 num_found_humans`: Number of currently detected humans.
 * `float64 min_dist`: The distance of the closest human to the robot.
 * `float32 current_speed`: The current speed of the robot.
 * `float32 current_rot`: The current rotaional speed of the robot.
 * `float32 remaining_runtime`: The remaining run time. Inf if indefinite.
 * `float32 time_to_reset`: The time until the speed will be reset to 'max_vel_x' (see goal)

## Human aware cmd_vel
A node to adjust the velocity of the robot by taking the /cmd_vel topic and republishing it. In order for this to work you have to remap the /cmd_vel output of your node or navigation stack to the input topic of this node. It relies on the ouptu of the strands_pedestrian_localisation package to provide the actual position of humans in the vicinity of the robot.

### Parameters
* `pedestrian_location` _Default: /pedestrian_localisation/localisations_: The topic on which the actual pedestrian locations are published by the pedestrian localisation (strands_perception_people_msgs/PedestrianLocations).
* `cmd_vel_in` _Default: /human_aware_cmd_vel/input/cmd_vel_: The topic to which the /cmd_vel should be published.
* `cmd_vel_out` _Default: /cmd_vel_: The modified /cmd_vel.
* `threshold` _Default: 3_: Threshold in seconds to determine which person detections from the cache are too old to use.
* `max_speed` _Default: 0.7_: The maximum speed the robot should achiev.
* `max_dist` _Default: 5.0_: The distance at which the node starts taking detections of humans into account.
* `min_dist` _Default: 1.5_: The cmd_vel will be 0.0 if there is a person closer to the robot than this.

### Running
```
rosrun strands_human_aware_velocity human_aware_cmd_vel [_parameter:=value]
```
