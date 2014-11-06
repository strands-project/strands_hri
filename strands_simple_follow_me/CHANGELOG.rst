^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_simple_follow_me
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.0.2 (2014-10-31)
------------------

0.0.1 (2014-10-31)
------------------
* Prepared strands_simple_follow_me for release.
* Renamed pedestrian_tracker launch files
* strands_perception_people_launch is now called perception_people_launch
* Renamed strands_people_tracker to bayes_people_tracker
* strands_perception_people_msgs has been removed
* Adapting to new people_tracker message.
* Bugfix: Added missing `*_generate_messages_cpp` for the action servers.
* Using ${catkin_EXPORTED_TARGETS} in all packages for tyhe dependecies.
* Dependency bug corrected
* Added readme with a bit of explanation.
* Delete CMakeLists.txt.user
  Rubbish file from qtcreator
* Now uses the move_base server to plan a path to the human in front of it.
* Added convenience scale parameters for velocity and did some bug fixing which occured during velocity calculation.
* First attempt of a very simple follow me.
* Contributors: Christian Dondrup, Tom Krajnik, cdondrup
