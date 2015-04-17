^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_interaction_behaviours
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Removed calls to strands_head_orientation as those won't work in a release version.
* Merge branch 'hydro-devel' into release
  Conflicts:
  strands_hri_utils/scripts/set_voice.py
  strands_visualise_speech/scripts/visual_speech.py
* Moved utility scripts out of utils into corresponding packages.
* Typo
* Added license
* Prepared strands_interaction_behaviours for release
  Needs https://github.com/strands-project/strands_ui/pull/39 merged.
* Added missing webpage
* Renaming dependencies: ros_mary_tts is now mary_tts
* strands_perception_people_msgs has been removed
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs strands-project/ros_datacentre#76 to be merged first.
* running head orientation while human_aware_navigation is running. Only if present. Changed default timeout of engaged server.
* Changing ques size and starting review action server in idle launch file
* Merge branch 'hydro-devel' of https://github.com/cdondrup/strands_hri into hydro-devel
* Renamed topic.
* Forgot a 'self'
* Bug fix
* added stop service call to preempt callback.
* Adding an action server for WP3/6 demo.
* Copy and paste error.
* Exposing parameters via launch file.
* More parameters to make more configurable
  Bug fix.
* Made idle behaviour more generic for review.
* Dialogue options for idle_nhm behaviour
* Mixed up z and y
* Using the titl motor tf to make the look around ignore the tilt value of the ptu but only take the pan.
* Adding orientation
* Switched coordinate frame for look around to ignore tilt values of ptu.
* Publish said sentences for nhm
* Setting initial gaze goal for the nav_goal
  resetting the mode in the behaviour switch to prevent it from looking away.
* Adding the possibility of disabling the looking around and speaking for the idle behaviour.
* Forgot to comment a call to create a page
* Some alteration taking out the webpages to make it more generic.
* Adding verbal output to signal that the behaviour was preempted and the robot will drive away now.
* Only creating the idle page when the engaed mode times out if the idle goal is still active.
* Fixing some web page issues and preempting
* -
* Changed spoken text.
* Removed the game and lights for now.
* Cancelling the executor goal if preempted.
* Adding the executor and removing the timeout for the engagedServer
* Added info behaviour. Only trigger for game is missing.
* Bug fixes
* Calling the right function for webpage.
* More typos.... It's getting late...
* Typo
* Added encoding info
* New webpage.
* renamed engagement_checker
* * Create engagement action server to handle interaction after engagement was detected
  * Incorporated engagemant_checker in the system to trigger engagement_server
  * Moved magic numbers to parameters
  * Moved some output to debug level
  * Moved setting of voice for mary to top level server
* First version of idle behaviour for robot.
  Still needs:
  * Testing
  * Incorporation of engagement check
  * Reaction to engagement
  * PTU turning when error is fixed.
* Contributors: Christian Dondrup, Marc Hanheide, Nick Hawes, Rares Ambrus, ToMadoRe, cdondrup
