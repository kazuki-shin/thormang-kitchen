// run this node first then send msg put head_down.
// then run perception.
// then run thormang_kitchen_demo_cmd_pub.
// then send message to demo_start.

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <pthread.h>
#include <geometry_msgs/Pose.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/JointPose.h"

ros::Publisher g_base_ini_pose_pub;
ros::Publisher g_enable_ctrl_module_pub;
ros::Publisher g_kinematics_msg_pub;
ros::Publisher g_manipulation_ini_pose_pub;
ros::Publisher g_gripper_pub;
ros::Publisher g_set_head_joint_angle_pub;
ros::Publisher g_joint_value_msg_pub;

int Taskindex=1;
double cup_grasp_angle = 30; //deg
double head_pan = 0; //deg
double head_tilt = -45; //deg
bool   is_ready = false;
bool   is_l_arm = true;
int    vision_count = 1;
bool   is_vision_get = true;

double goal_x;
double goal_y;
double goal_z=0.82;
double ready_x=0.2;
double ready_y=0.3;
double RotationAngle;
double goal_orientation_z;
double goal_orientation_w;

void demoCupPosCallback(const geometry_msgs::Pose::ConstPtr &cupPosition)
{
  if (is_vision_get) {
    goal_x=cupPosition->position.x;
    goal_y=cupPosition->position.y;
    goal_x=goal_x/1000;
    goal_y=goal_y/1000;
    ROS_INFO("(%f, %f)",goal_x,goal_y);
    goal_x = std::max(goal_x, 0.3);
    goal_x = std::min(goal_x, 0.7);
    goal_y = std::max(goal_y, -0.2);
    goal_y = std::min(goal_y, 0.2);
    vision_count++;
  }
}

void demoStatusCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  ROS_INFO("Checking");
  if (msg->module_name == "Manipulation")
  {
    if(msg->status_msg == "End Trajectory"){
      is_ready = true;
    }else {
      is_ready = false;
    }
  }
  else if (msg->module_name == "Head Control")
  {
    if(msg->status_msg == "Head movement is finished."){
      is_ready = true;
    }else {
      is_ready = false;
    }
  }
  // else if (msg->module_name == "Base")
  // {
  //   if(msg->status_msg == "Finish Init Pose"){
  //     is_ready = true;
  //   }else {
  //     is_ready = false;
  //   }
  // }
  else if (msg->module_name == "Gripper")
  {
    if(msg->status_msg == "End Trajectory"){
      is_ready = true;
    }else {
      is_ready = false;
    }
  }else{
    is_ready = false;
  }
}

void demoCommandCallback(const std_msgs::String& msg)
{
  ros::Rate loop_rate(1);

  if (msg.data == "head_down"){
    std_msgs::String demo_mode_msg;
    demo_mode_msg.data = "head_control_module";
    g_enable_ctrl_module_pub.publish(demo_mode_msg);
    loop_rate.sleep();

    sensor_msgs::JointState head_angle_msg;
    // head_angle_msg.name.push_back("head_y");
    head_angle_msg.name.push_back("head_p");
    // head_angle_msg.position.push_back(-head_pan);
    head_angle_msg.position.push_back(-head_tilt*3.14/180);
    // head_angle_msg.effort.push_back((double)250);
    g_set_head_joint_angle_pub.publish(head_angle_msg);
  }

  // rosrun thormang_kitchen_demo thormang_kitchen_demo_cmd_pub

  double goal_x=0.5;// Testing uncomment
  double goal_y=0.15;// Testing uncomment
  // double goal_z=0.82;
  // double ready_x=0.2;
  // double ready_y=0.3;
  // double RotationAngle;
  // double goal_orientation_z;
  // double goal_orientation_w;
  // RotationAngle = atan2((goal_y-ready_y),(goal_x-ready_x));
  // goal_orientation_z = sin(RotationAngle/2); // no idea why it works but it gives the right results.
  // goal_orientation_w = cos(RotationAngle/2);


  // loop_rate.sleep();

  ROS_INFO("Demo start");

  if (msg.data == "demo_start"){

    if (goal_y>=0){is_l_arm=true;}
    else{is_l_arm=false;}

    ROS_INFO("%d",Taskindex);
    // rostopic pub -1 /robotis/thormang_kitchen_demo/command std_msgs/String "demo_start"
    ROS_INFO("Set Control Mode");
    std_msgs::String demo_mode_msg;
    // demo_mode_msg.data = "head_control_module";
    // g_enable_ctrl_module_pub.publish(demo_mode_msg);
    demo_mode_msg.data = "manipulation_module";
    g_enable_ctrl_module_pub.publish(demo_mode_msg);
    demo_mode_msg.data = "gripper_module";
    g_enable_ctrl_module_pub.publish(demo_mode_msg);
    loop_rate.sleep();
    if (Taskindex==1 && is_ready)
    {
      /*manipulation ready pose*/
      ROS_INFO("To Manipulation Ready Pose");
      std_msgs::String demo_init_msg;
      demo_init_msg.data = "ini_pose";
      g_manipulation_ini_pose_pub.publish(demo_init_msg);
      Taskindex=2;
      is_vision_get=false;
    }else if (Taskindex==2 && is_ready && goal_y>0)
    {
      /*angle positioning*/
      ROS_INFO("Angle Positioning With Left Arm");
      RotationAngle = atan2((goal_y-ready_y),(goal_x-ready_x));
      goal_orientation_z = sin(RotationAngle/2); // no idea why it works but it gives the right results.
      goal_orientation_w = cos(RotationAngle/2);
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      // if (goal_y>=0){
        demo_pose_msgs.name = "left_arm_with_torso";
      // }else{
      //   demo_pose_msgs.name = "right_arm_with_torso";
      // }
      demo_pose_msgs.pose.position.x = ready_x;
      demo_pose_msgs.pose.position.y = ready_y;
      demo_pose_msgs.pose.position.z = goal_z;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 1;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=3;
    }else if (Taskindex==3 && is_ready && goal_y>0)
    {
      /*approaching*/
      ROS_INFO("Approaching With Left Arm");
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      // if (goal_y>=0){
        demo_pose_msgs.name = "left_arm_with_torso";
      // }else{
      //   demo_pose_msgs.name = "right_arm_with_torso";
      // }
      demo_pose_msgs.pose.position.x = goal_x;
      demo_pose_msgs.pose.position.y = goal_y;
      demo_pose_msgs.pose.position.z = goal_z;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 2;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=4;
    }else if (Taskindex==2 && is_ready && goal_y<0)
    {
      /*angle positioning r*/
      ROS_INFO("Angle Positioning With Rgiht Arm");
      RotationAngle = atan2((goal_y+ready_y),(goal_x-ready_x));
      goal_orientation_z = sin(RotationAngle/2); // no idea why it works but it gives the right results.
      goal_orientation_w = cos(RotationAngle/2);
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      // if (goal_y>=0){
        demo_pose_msgs.name = "right_arm_with_torso";
      // }else{
      //   demo_pose_msgs.name = "right_arm_with_torso";
      // }
      demo_pose_msgs.pose.position.x = ready_x;
      demo_pose_msgs.pose.position.y = -ready_y;
      demo_pose_msgs.pose.position.z = goal_z;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 2;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=23;
    }else if (Taskindex==23 && is_ready && goal_y<0)
    {
      /*approaching r*/
      ROS_INFO("Approaching With Right Arm");
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      // if (goal_y>=0){
        demo_pose_msgs.name = "right_arm_with_torso";
      // }else{
      //   demo_pose_msgs.name = "right_arm_with_torso";
      // }
      demo_pose_msgs.pose.position.x = goal_x;
      demo_pose_msgs.pose.position.y = goal_y;
      demo_pose_msgs.pose.position.z = goal_z+0.04;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 2;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=24;
    }else if (Taskindex==4 && is_ready)
    {
      /*grasping*/
      ROS_INFO("Gripper Closing");
      sensor_msgs::JointState demo_joint_msgs;
      demo_joint_msgs.name.push_back("l_arm_grip");
      demo_joint_msgs.position.push_back((double)(cup_grasp_angle*3.14/180));
      demo_joint_msgs.effort.push_back((double)250);
      g_gripper_pub.publish(demo_joint_msgs);
      Taskindex=5;
    }else if (Taskindex==24 && is_ready)
    {
      /*grasping*/
      ROS_INFO("Gripper Closing");
      sensor_msgs::JointState demo_joint_msgs;
      demo_joint_msgs.name.push_back("r_arm_grip");
      demo_joint_msgs.position.push_back((double)(cup_grasp_angle*3.14/180));
      demo_joint_msgs.effort.push_back((double)250);
      g_gripper_pub.publish(demo_joint_msgs);
      Taskindex=25;
    }else if (Taskindex==5 && is_ready)
    {
      /*lift*/
      ROS_INFO("Lifting");
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      demo_pose_msgs.name = "left_arm_with_torso";
      demo_pose_msgs.pose.position.x = goal_x;
      demo_pose_msgs.pose.position.y = goal_y;
      demo_pose_msgs.pose.position.z = goal_z+0.1;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 1;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=6;
    }else if (Taskindex==25 && is_ready)
    {
      /*lift*/
      ROS_INFO("Lifting");
      thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
      demo_pose_msgs.name = "right_arm_with_torso";
      demo_pose_msgs.pose.position.x = goal_x;
      demo_pose_msgs.pose.position.y = goal_y;
      demo_pose_msgs.pose.position.z = goal_z+0.14;
      demo_pose_msgs.pose.orientation.w = goal_orientation_w;
      demo_pose_msgs.pose.orientation.x = 0.0;
      demo_pose_msgs.pose.orientation.y = 0.0;
      demo_pose_msgs.pose.orientation.z = goal_orientation_z;
      demo_pose_msgs.time = 1;
      g_kinematics_msg_pub.publish(demo_pose_msgs);
      Taskindex=26;
    }

    // else if (Taskindex==26 && is_ready)
    // {
    //   /*torso zero*/
    //   ROS_INFO("Torso Joint Goes To Zero");
    //   thormang3_manipulation_module_msgs::JointPose demo_joint_msg;
    //   demo_joint_msg.name = "torso_y";
    //   demo_joint_msg.value = 0.0;
    //   g_joint_value_msg_pub.publish(demo_joint_msg);
    //   Taskindex=27;
    // }else if (Taskindex==27 && is_ready)
    // {
    //   /*right arm exchange*/
    //   ROS_INFO("Right Arm Goes To Exchange");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "right_arm";
    //   demo_pose_msgs.pose.position.x = 0.36;
    //   demo_pose_msgs.pose.position.y = 0;
    //   demo_pose_msgs.pose.position.z = goal_z+0.14;
    //   demo_pose_msgs.pose.orientation.w = cos(3.14/4);
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = sin(3.14/4);
    //   demo_pose_msgs.time = 2;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=28;
    // }else if (Taskindex==28 && is_ready)
    // {
    //   /*left arm exchange*/
    //   ROS_INFO("Left Arm Goes To Exchange");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "left_arm";
    //   demo_pose_msgs.pose.position.x = 0.36;
    //   demo_pose_msgs.pose.position.y = 0.025;
    //   demo_pose_msgs.pose.position.z = goal_z+0.065;
    //   demo_pose_msgs.pose.orientation.w = cos(-3.14/4);
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = sin(-3.14/4);
    //   demo_pose_msgs.time = 2;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=29;
    // }else if (Taskindex==29 && is_ready)
    // {
    //   /*left gripper grasping*/
    //   ROS_INFO("Left Gripper Closing");
    //   sensor_msgs::JointState demo_joint_msgs;
    //   demo_joint_msgs.name.push_back("l_arm_grip");
    //   demo_joint_msgs.position.push_back((double)(cup_grasp_angle*3.14/180));
    //   demo_joint_msgs.effort.push_back((double)250);
    //   g_gripper_pub.publish(demo_joint_msgs);
    //   Taskindex=30;
    // }else if (Taskindex==30 && is_ready)
    // {
    //   /*right gripper loosing*/
    //   ROS_INFO("Gripper Opening");
    //   sensor_msgs::JointState demo_joint_msgs;
    //   demo_joint_msgs.name.push_back("r_arm_grip");
    //   demo_joint_msgs.position.push_back((double)(3*3.14/180));
    //   demo_joint_msgs.effort.push_back((double)250);
    //   g_gripper_pub.publish(demo_joint_msgs);
    //   Taskindex=31;
    // }else if (Taskindex==31 && is_ready)
    // {
    //   /*to washer*/
    //   ROS_INFO("To Washer 1");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "left_arm";
    //   demo_pose_msgs.pose.position.x = 0.5;
    //   demo_pose_msgs.pose.position.y = 0.44;
    //   demo_pose_msgs.pose.position.z = goal_z+0.1;
    //   demo_pose_msgs.pose.orientation.w = 1;
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = 0;
    //   demo_pose_msgs.time = 3;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=32;
    // }else if (Taskindex==32 && is_ready)
    // {
    //   /*to washer*/
    //   ROS_INFO("To Washer 2");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "left_arm";
    //   demo_pose_msgs.pose.position.x = 0;
    //   demo_pose_msgs.pose.position.y = 0.8;
    //   demo_pose_msgs.pose.position.z = goal_z+0.1;
    //   demo_pose_msgs.pose.orientation.w = cos(3.14/4);
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = sin(3.14/4);
    //   demo_pose_msgs.time = 3;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=33;
    // }else if (Taskindex==33 && is_ready)
    // {
    //   /*loose*/
    //   ROS_INFO("Gripper Opening");
    //   sensor_msgs::JointState demo_joint_msgs;
    //   demo_joint_msgs.name.push_back("l_arm_grip");
    //   demo_joint_msgs.position.push_back((double)(3*3.14/180));
    //   demo_joint_msgs.effort.push_back((double)250);
    //   g_gripper_pub.publish(demo_joint_msgs);
    //   Taskindex=34;
    // }else if (Taskindex==34 && is_ready)
    // {
    //   /*r_arm_sh_p2zero*/
    //   ROS_INFO("r_arm_sh_p2 Joint Goes To Zero");
    //   thormang3_manipulation_module_msgs::JointPose demo_joint_msg;
    //   demo_joint_msg.name = "r_arm_sh_p2";
    //   demo_joint_msg.value = 15.0*3.14/180;
    //   g_joint_value_msg_pub.publish(demo_joint_msg);
    //   Taskindex=9;
    // }
    //
    // else if (Taskindex==6 && is_ready)
    // {
    //   /*to washer*/
    //   ROS_INFO("To Washer 1");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "left_arm_with_torso";
    //   demo_pose_msgs.pose.position.x = 0.5;
    //   demo_pose_msgs.pose.position.y = 0.44;
    //   demo_pose_msgs.pose.position.z = goal_z+0.1;
    //   demo_pose_msgs.pose.orientation.w = 1;
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = 0;
    //   demo_pose_msgs.time = 2;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=7;
    // }else if (Taskindex==7 && is_ready)
    // {
    //   /*to washer*/
    //   ROS_INFO("To Washer 2");
    //   thormang3_manipulation_module_msgs::KinematicsPose demo_pose_msgs;
    //   demo_pose_msgs.name = "left_arm_with_torso";
    //   demo_pose_msgs.pose.position.x = 0;
    //   demo_pose_msgs.pose.position.y = 0.8;
    //   demo_pose_msgs.pose.position.z = goal_z+0.1;
    //   demo_pose_msgs.pose.orientation.w = cos(3.14/4);
    //   demo_pose_msgs.pose.orientation.x = 0.0;
    //   demo_pose_msgs.pose.orientation.y = 0.0;
    //   demo_pose_msgs.pose.orientation.z = sin(3.14/4);
    //   demo_pose_msgs.time = 2;
    //   g_kinematics_msg_pub.publish(demo_pose_msgs);
    //   Taskindex=8;
    // }else if (Taskindex==8 && is_ready)
    // {
    //   /*loose*/
    //   ROS_INFO("Gripper Opening");
    //   sensor_msgs::JointState demo_joint_msgs;
    //   demo_joint_msgs.name.push_back("l_arm_grip");
    //   demo_joint_msgs.position.push_back((double)(3*3.14/180));
    //   demo_joint_msgs.effort.push_back((double)250);
    //   g_gripper_pub.publish(demo_joint_msgs);
    //   Taskindex=9;
    // }else if (Taskindex==9 && is_ready)
    // {
    //   /*manipulation ready pose*/
    //   ROS_INFO("To Manipulation Ready Pose");
    //   std_msgs::String demo_init_msg;
    //   demo_init_msg.data = "ini_pose";
    //   g_manipulation_ini_pose_pub.publish(demo_init_msg);
    //   Taskindex=10;
    // }else
    {
      return;
    }
    is_vision_get=true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kitchen_demo_publisher");
  ros::NodeHandle nh("~");

  g_base_ini_pose_pub         = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_module_pub    = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  g_kinematics_msg_pub        = nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/manipulation/kinematics_pose_msg", 0);
  g_manipulation_ini_pose_pub = nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);
  g_gripper_pub               = nh.advertise<sensor_msgs::JointState>("/robotis/gripper/joint_pose_msg", 0);
  g_set_head_joint_angle_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);
  g_joint_value_msg_pub       = nh.advertise<thormang3_manipulation_module_msgs::JointPose>("/robotis/manipulation/joint_pose_msg", 0);

  ros::Rate loop_rate(0.5);
  loop_rate.sleep();

    /*initial pose*/
  ROS_INFO("Initial Pose");
  std_msgs::String demo_init_msg;
  demo_init_msg.data = "ini_pose";
  g_base_ini_pose_pub.publish(demo_init_msg);

  ros::Subscriber demo_command_sub = nh.subscribe("/robotis/thormang_kitchen_demo/command", 5, demoCommandCallback);
  ros::Subscriber status_msg_sub   = nh.subscribe("/robotis/status", 10, demoStatusCallback);
  ros::Subscriber cupPos_msg_sub   = nh.subscribe("/viz/coords", 10, demoCupPosCallback);

  ros::spin();

  return 0;
}
