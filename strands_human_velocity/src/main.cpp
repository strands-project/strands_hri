#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include "strands_perception_people_msgs/PedestrianLocations.h"

using namespace std;
using namespace strands_perception_people_msgs;

int threshold;
double scale;
double min_dist;

ros::Publisher cmd_pub;
message_filters::Cache<PedestrianLocations> *cache;

void callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_DEBUG_STREAM("Received cmd_vel:\n" << *msg);
    PedestrianLocations::ConstPtr pl = cache->getElemBeforeTime(ros::Time::now());
    if(pl != NULL) {
        if(pl->header.stamp.sec > ros::Time::now().sec-threshold) {
            ROS_DEBUG_STREAM("Found pedestrian localisation msg:\n" << *pl);
            double speed = pl->min_distance-1.5;
            speed = speed > 0.0 ? speed : 0.0;
            speed = speed > 1.0 ? 1.0 : speed;
            speed *= scale;
            ROS_DEBUG_STREAM("New speed: " << speed);
            if(speed < msg->linear.x) {
                geometry_msgs::Twist new_msg;
                new_msg.linear.x = speed;
                new_msg.angular.z  = msg->angular.z;
                ROS_DEBUG_STREAM("Publishing: " << new_msg);
                //Publish and return to skip publishing at the end
                cmd_pub.publish(new_msg);
                return;
            }
        } else {
            ROS_DEBUG("No current observation");
        }
    } else {
        ROS_DEBUG("No observation");
    }
    //Publish something in any case
    cmd_pub.publish(*msg);
}

void cacheCallback(const PedestrianLocations::ConstPtr &msg)
{
    ROS_INFO_STREAM("Got ppl loc:\n" << *msg);
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "human_aware_cmd_vel");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pl_topic;
    string cmd_vel_in_topic;
    string cmd_vel_out_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pedestrian_location", pl_topic, string("/pedestrian_localisation/localisations"));
    private_node_handle_.param("cmd_vel_in", cmd_vel_in_topic, string("/human_aware_cmd_vel/input/cmd_vel"));
    private_node_handle_.param("cmd_vel_out", cmd_vel_out_topic, string("/human_aware_cmd_vel/output/cmd_vel"));
    private_node_handle_.param("threshold", threshold, 3);
    private_node_handle_.param("max_speed", scale, 0.7);
    private_node_handle_.param("min_dist", min_dist, 1.5);

    message_filters::Subscriber<PedestrianLocations> sub(n, pl_topic.c_str(), 1);
    cache = new message_filters::Cache<PedestrianLocations>(sub, 10);
    const PedestrianLocations::ConstPtr dummy;
//    cache->add(dummy);
//    cache->registerCallback(boost::bind(&cacheCallback, _1));

    // Create a subscriber.
    ros::Subscriber cmd_sub = n.subscribe(cmd_vel_in_topic.c_str(), 10, &callback);
//    ros::Subscriber test_sub = n.subscribe(pl_topic.c_str(), 10, &cacheCallback);

    cmd_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_out_topic.c_str(), 10);

    ros::spin();
    return 0;
}

