^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_gazing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.11 (2015-04-17)
-------------------

0.0.10 (2015-04-10)
-------------------

0.0.9 (2015-04-10)
------------------

0.0.7 (2014-12-01)
------------------

0.0.6 (2014-11-21)
------------------

0.0.5 (2014-11-11)
------------------

0.0.3 (2014-11-06)
------------------

0.0.2 (2014-10-31)
------------------

0.0.1 (2014-10-31)
------------------
* Moved utility scripts out of utils into corresponding packages.
* Added missing webpage
* Prepared strands_gazing for release
* moved the sleep() such that it always gets called in the while loop, irrespective of if-statements
* move some outputs to debug level.
* [strands_gazing]Removed unnecessary subscription to a placeholder topic.
* Removed the pose reset in the gaze_at_pose node to not have it spin around if looking to the back.
* Bugfix: Needs absolute x value to calculate Tilt using atan2 instead of negative one.
* Removing look up tf warning by looking for the right transformation.
* trying to prevent some warning while transforming.
* Using EyeLids joint state to prevent occasional "blinking".
* Changed blink timing to blink every 5 to 20 seconds.
* Bugfix: Added missing `*_generate_messages_cpp` for the action servers.
* Using ${catkin_EXPORTED_TARGETS} in all packages for tyhe dependecies.
* Now able to look at any given Stamped pose continuously, e.g. /move_base/current_goal
* gaze_at_pose subscribes and unsubscribes automatically depending on whether there is in an active goal or not.
* Random eyeblinking added
* Update README.md
  Fixing typos
* Updated readme with action server info.
* Added a README, updated the package.xml, and changed the standard subscription topic to a more generic one.
* Merge branch 'not_master' of https://github.com/cdondrup/strands_hri
* Added comments and changed info outputs to debug.
* The eye movement does not work as inteded and has been removed for the time being.
* Combining gazing and velocity in one branch.
  Adding dependencies to velocity package.
* First implementation of a node which focuses a 3d pose with the head. Needs testing and some more functionality.
* Contributors: Christian Dondrup, ToMadoRe, Tom Krajnik, cdondrup
