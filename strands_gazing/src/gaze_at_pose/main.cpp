#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <strands_gazing/GazeAtPoseAction.h>

#include <limits>

using namespace std;

ros::Publisher head_pose_pub;
tf::TransformListener* listener;
actionlib::SimpleActionServer<strands_gazing::GazeAtPoseAction> *as_;
boost::shared_ptr<const strands_gazing::GazeAtPoseGoal> goal_;
strands_gazing::GazeAtPoseFeedback feedback_;
strands_gazing::GazeAtPoseResult result_;
std::string action_name_;
bool once = false;
string target_frame;
double end_time;

void goalCallback() {
    goal_ = as_->acceptNewGoal();
    ROS_INFO_STREAM("Received goal:\n" << *goal_);
    end_time = goal_->runtime_sec > 0 ? ros::Time::now().toSec() + goal_->runtime_sec : 0.0;
}

void preemptCallback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    sensor_msgs::JointState state;
    state.header.frame_id = "/head_base_frame";
    state.header.stamp = ros::Time::now();
    state.name.push_back("HeadPan");
    state.name.push_back("HeadTilt");
    state.name.push_back("EyesPan");
    state.position.push_back(0.0);
    state.position.push_back(0.0);
    state.position.push_back(0.0);

    head_pose_pub.publish(state);
    result_.expired = false;
    as_->setPreempted(result_);
}

void inline feedback(geometry_msgs::Pose pose) {
    feedback_.target = pose;
    feedback_.remaining_time = end_time > 0 ? end_time - ros::Time::now().toSec() : INFINITY;
    as_->publishFeedback(feedback_);
}

void inline checkTime() {
    if(ros::Time::now().toSec() > end_time && end_time > 0.0) {
        ROS_INFO("Execution time has been reached. Goal terminated successfully");
        result_.expired = true;
        as_->setSucceeded(result_);
    }
}


void callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    if (!as_->isActive())
        return;

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
            once = false;
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
        feedback(closest_head_coord.pose);
        checkTime();
    } else if(!once){
        closest_head_coord = closest;
        closest_head_coord.header.frame_id = target_frame;
        closest_head_coord.pose.position.z = 0;
        feedback(closest_head_coord.pose);
        once = true;
        checkTime();
    } else {
        feedback(closest_head_coord.pose);
        checkTime();
        return;
    }


    sensor_msgs::JointState state;
    state.header = closest_head_coord.header;
    state.name.push_back("HeadPan");
    state.name.push_back("HeadTilt");
    state.position.push_back(std::atan2(closest_head_coord.pose.position.y, closest_head_coord.pose.position.x) * 180.0 / M_PI * 0.8);
    state.position.push_back(std::atan2(closest_head_coord.pose.position.z, closest_head_coord.pose.position.x) * 180.0 / M_PI);
    state.position.push_back(std::atan2(closest_head_coord.pose.position.y, closest_head_coord.pose.position.x) * 180.0 / M_PI * 0.2);

    head_pose_pub.publish(state);
}



int main(int argc, char **argv)
{
    // Set up ROS.
    action_name_ = "gaze_at_pose";
    ros::init(argc, argv, action_name_.c_str());
    ros::NodeHandle n;

    listener = new tf::TransformListener();

    as_ = new actionlib::SimpleActionServer<strands_gazing::GazeAtPoseAction>(n, action_name_, false);
    as_->registerGoalCallback(boost::bind(&goalCallback));
    as_->registerPreemptCallback(boost::bind(&preemptCallback));

    // Declare variables that can be modified by launch file or command line.
    string pose_array_topic;
    string head_pose_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("pose_array", pose_array_topic, string("/upper_body_detector/bounding_box_centres"));
    private_node_handle_.param("head_pose", head_pose_topic, string("/head/commanded_state"));
    private_node_handle_.param("head_frame", target_frame, string("/head_base_frame"));

    // Create a subscriber.
    ros::Subscriber pose_array_sub = n.subscribe(pose_array_topic.c_str(), 10, &callback);

    head_pose_pub = n.advertise<sensor_msgs::JointState>(head_pose_topic.c_str(), 10);

    as_->start();

    ros::spin();
    return 0;
}
