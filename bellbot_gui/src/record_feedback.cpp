#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <fstream>

ros::ServiceClient client;

void recordCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Feedback received: [%s]", msg->data.c_str());
  std::ofstream out;

  out.open("bellbot_feedback_results.txt",std::ios_base::app);
  out << msg->data.c_str() << "\n";
  out.close();
  ROS_INFO("Wrote feedback to file");
  //bellbot_gui::FeedbackDone srv;
  std_srvs::Empty srv;
  if (client.call(srv))
  {
    ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("fail");
  }
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "bellbot_gui_feedback_RECORDER");
  ros::NodeHandle n;

  client = n.serviceClient<std_srvs::Empty>("/bellbot/gui/feedback_done");
  ros::Subscriber sub = n.subscribe("/bellbot_gui_feedback", 1000, recordCallBack);

  ros::spin();
  ROS_INFO("Atomic batteries to power. Bellbot feedback recorder v0.445.14 online");

  return 0;
}
