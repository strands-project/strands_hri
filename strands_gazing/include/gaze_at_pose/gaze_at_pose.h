#ifndef GAZE_AT_POSE_H
#define GAZE_AT_POSE_H

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

#include <boost/thread.hpp>

class GazeAtPose
{
public:
    GazeAtPose(std::string name);
    ~GazeAtPose();
    void callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void goalCallback(ros::NodeHandle &n);
    void preemptCallback();

private:
    void init();
    void inline feedback(geometry_msgs::Pose pose);
    void inline checkTime();
    void transform();
    void setPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    const geometry_msgs::PoseStamped::ConstPtr getPose();
    void resetHead();

    ros::Publisher head_pose_pub;
    ros::Subscriber pose_array_sub;
    tf::TransformListener* listener;
    actionlib::SimpleActionServer<strands_gazing::GazeAtPoseAction> *as_;
    boost::shared_ptr<const strands_gazing::GazeAtPoseGoal> goal_;
    strands_gazing::GazeAtPoseFeedback feedback_;
    strands_gazing::GazeAtPoseResult result_;
    std::string action_name_;
    std::string target_frame;
    double end_time;
    int timeToBlink;
    int timeToUnBlink;
    geometry_msgs::PoseStamped::ConstPtr pose;
    boost::mutex mutex;
    boost::thread transform_thread;
};

#endif // GAZE_AT_POSE_H
