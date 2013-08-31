#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher head_pose_pub;
tf::TransformListener* listener;

string target_frame;


void callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    ROS_INFO_STREAM("Received posearray:\n" << *msg);

    bool found = false;

    geometry_msgs::PoseStamped closest;
    closest.header = msg->header;
    closest.pose.position.z = 10000;
    closest.pose.orientation.w = 1;
    for(int i = 0; i < msg->poses.size(); i++) {
        if(closest.pose.position.z > msg->poses[i].position.z) {
            closest.pose.position = msg->poses[i].position;
            found = true;
        }
    }

    geometry_msgs::PoseStamped closest_head_coord;
    if(found) {
        ROS_INFO_STREAM("Found closest point at:\n" << closest);

        //Transform
        try {
            ROS_DEBUG("Transforming received position into %s coordinate system.", target_frame.c_str());
            listener->waitForTransform(closest.header.frame_id, target_frame, ros::Time(), ros::Duration(3.0));
            listener->transformPose(target_frame, closest, closest_head_coord);
        }
        catch(tf::TransformException ex) {
            ROS_WARN("Failed transform: %s", ex.what());
            return;
        }

        ROS_INFO_STREAM("Transformed into:\n" << closest_head_coord);
    } else {
        closest_head_coord = closest;
        closest_head_coord.header.frame_id = target_frame;
        closest_head_coord.pose.position.z = 0;
    }

    sensor_msgs::JointState state;
    state.header = closest_head_coord.header;
    state.name.push_back("HeadPan");
    state.name.push_back("HeadTilt");
    state.position.push_back(std::atan2(closest_head_coord.pose.position.y, closest_head_coord.pose.position.x) * 180.0 / M_PI);
    state.position.push_back(std::atan2(closest_head_coord.pose.position.z, closest_head_coord.pose.position.x) * 180.0 / M_PI);


    head_pose_pub.publish(state);
}



int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "gaze_at_pose");
    ros::NodeHandle n;

    listener = new tf::TransformListener();

    // Declare variables that can be modified by launch file or command line.
    string pose_array_topic;
    string head_pose_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pose_array", pose_array_topic, string("/upper_body_detector/bounding_box_centres"));
    private_node_handle_.param("head_pose", head_pose_topic, string("/gaze_at_pose/head_pose"));
    private_node_handle_.param("head_frame", target_frame, string("/head_base_frame"));

    // Create a subscriber.
    ros::Subscriber pose_array_sub = n.subscribe(pose_array_topic.c_str(), 10, &callback);

    head_pose_pub = n.advertise<sensor_msgs::JointState>(head_pose_topic.c_str(), 10);

    ros::spin();
    return 0;
}
