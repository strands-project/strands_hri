## HRI utils
At the the moment this package only containes a convenience node to simplify the usage of mary togther with the head 
light control from the strands_visualise_speech package (see package for detailed description). This action server 
forwards the sent text to mary while also running the sound_to_light action server to visualise the speech output. 
After the sent text has been uttered the lights are switched back to the spinning behaviour until a new text is received.



### Usage:
* `roslaunch strands_hri_launch speech.launch`: this launches mary, the sound_to_light server and the visualise_speech 
server.
* Send a goal to /visual_speech, e.g. with `rosrun actionlib axclient.py /visual_speech`
 * The goal just takes a text which is forwarded to mary.
 * The lights will visualise the spoken words while mary is synthesing the speech output
 * After the text has been uttered the lights will go back to the spinning behaviour.

_Cannot be run remotely. Needs to access the robots hardware_

### Speak Web Service
Another tool is an AJAX webservice to amke the robot speak for demos. Just `rosrun strands_hri_utils webserver.py` and the `roslaunch strands_hri_launch speech.launch`, and then connect to port 8080 on the robot to have a simple to use interface.
