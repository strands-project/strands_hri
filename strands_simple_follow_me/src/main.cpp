#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

#include <math.h>

#include "bayes_people_tracker/PeopleTracker.h"

using namespace std;
using namespace bayes_people_tracker;

geometry_msgs::Point polarToCartesian(float dist, float angle) {
    geometry_msgs::Point output;
    output.x = dist * cos(angle);
    output.y = dist * sin(angle);
    output.z = 0.0;
    return output;
}

void locationCallback(const PeopleTracker::ConstPtr &pl)
{
    //tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    geometry_msgs::Point point = polarToCartesian(pl->min_distance, pl->min_distance_angle);

    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;


    tf::Quaternion quat(tf::Vector3(0,0,1),pl->min_distance_angle);
    goal.target_pose.pose.orientation.w=quat.getW();
    goal.target_pose.pose.orientation.x=quat.getX();
    goal.target_pose.pose.orientation.y=quat.getY();
    goal.target_pose.pose.orientation.z=quat.getZ();

    ROS_DEBUG_STREAM("Sending goal " << goal);
    ac.sendGoal(goal);

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "simple_follow_me");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pl_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("people_tracker", pl_topic, string("/people_tracker/positions"));

    // Create a subscriber.
    ros::Subscriber pl_sub = n.subscribe(pl_topic.c_str(), 10, &locationCallback);

    ros::spin();
    return 0;
}


