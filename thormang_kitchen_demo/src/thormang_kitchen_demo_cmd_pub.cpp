#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

ros::Publisher g_start_cmd_pub;

void demoStatusCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  std_msgs::String demo_cmd_msg;
  demo_cmd_msg.data = "demo_start";
  if (msg->module_name == "Manipulation")
  {
    if(msg->status_msg == "End Trajectory"){
      g_start_cmd_pub.publish(demo_cmd_msg);
    }
  }
  else if (msg->module_name == "Head Control")
  {
    if(msg->status_msg == "Head movement is finished."){
      g_start_cmd_pub.publish(demo_cmd_msg);
    }
  }
  else if (msg->module_name == "Gripper")
  {
    if(msg->status_msg == "End Trajectory"){
      g_start_cmd_pub.publish(demo_cmd_msg);
    }
  }else{
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kitchen_demo_start_cmd_publisher");
  ros::NodeHandle nh("~");

  g_start_cmd_pub = nh.advertise<std_msgs::String>("/robotis/thormang_kitchen_demo/command", 0);

  ros::Subscriber status_msg_sub   = nh.subscribe("/robotis/status", 10, demoStatusCallback);

  ros::spin();

  return 0;
}
