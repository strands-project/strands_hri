#!/usr/bin/env python

from flask import Flask
from flask import render_template
from flask import request, redirect
import roslib
import rospy
import actionlib
import ros_mary_tts.msg

roslib.load_manifest('strands_hri_utils')

app = Flask(__name__)


@app.route("/")
def hello():
    return render_template('index.html')


@app.route('/speak')
def speak():
    text = request.args.get('data')
    # the code below is executed if the request method
    # was GET or the credentials were invalid
    rospy.loginfo('trying to say ' + text)
    rospy.loginfo('waiting for say server')
    client = actionlib.SimpleActionClient(
        '/speak', ros_mary_tts.msg.maryttsAction)

    client.wait_for_server()
    g = ros_mary_tts.msg.maryttsGoal()
    g.text = text
    rospy.loginfo('start speaking')
    client.send_goal(g)
    rospy.loginfo('started speaking')
    client.wait_for_result()
    rospy.loginfo('finished speaking')
    return redirect('/')


if __name__ == "__main__":
	rospy.init_node('speak_webserver')
	app.run(host='0.0.0.0', port=5000)
