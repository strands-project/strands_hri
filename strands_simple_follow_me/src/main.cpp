#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "strands_perception_people_msgs/PedestrianLocations.h"

using namespace std;
using namespace strands_perception_people_msgs;

ros::Publisher pub_cmd_vel;

void locationCallback(const PedestrianLocations::ConstPtr &pl)
{
    bool subs = pub_cmd_vel.getNumSubscribers();
    if(!subs) {
        ROS_DEBUG("No subscribers. Skipping calculation.");
        return;
    }

    geometry_msgs::Twist twist;

    twist.angular.z = pl->min_distance_angle*0.7;
    twist.linear.x  = (pl->min_distance-1.0)*0.7;

    pub_cmd_vel.publish(twist);

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "simple_follow_me");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pl_topic;
    string pub_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pedestrian_location", pl_topic, string("/pedestrian_localisation/localisations"));

    // Create a subscriber.
    ros::Subscriber pl_sub = n.subscribe(pl_topic.c_str(), 10, &locationCallback);

    private_node_handle_.param("cmd_vel", pub_topic, string("/cmd_vel"));
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>(pub_topic.c_str(), 10);

    ros::spin();
    return 0;
}


