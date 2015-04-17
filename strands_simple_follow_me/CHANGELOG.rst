^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_simple_follow_me
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
