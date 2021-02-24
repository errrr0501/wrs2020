#include <stdio.h>
#include <vector>

//#include "manipulator_h_base_module/base_module.h"
#include "robotiq_2f_gripper_control/gripper_control.h"

#include <cstdlib>
#include <vector>
#include <boost/thread.hpp>

using namespace robotiq_2f_gripper;

bool grap ;
bool release ;
bool grap_alc ;
bool grap_scraper ; 



// GripperControl::GripperControl()
// {
//   grap = false;
//   release = false;
//   grap_alc = false;
//   grap_scraper = false;
// }

//==========================================roboticq2f_85======================================================
void robotiq_2f_gripper_Client(){
  actionlib::SimpleActionClient<robotiq_2f_gripper_msgs::CommandRobotiqGripperAction> grap_action_client_("/command_robotiq_action");
  //grap_action_client_.waitForServer(ros::Duration(0.0));
  ROS_INFO("Connected to grap_action server");

  ROS_INFO("Action server started.");
  robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal goal;

  bool False = false;
  goal.emergency_release = False;
  goal.stop = False;
  std::cout<<grap_alc<<std::endl;
  if(grap_alc == true){
      goal.position = 0.065;
      goal.speed = 0.1;
      goal.force = 0.5;
  }
  else if(grap_scraper == true){
      goal.position = 0.04;
      goal.speed = 0.1;
      goal.force = 400;
  }
  else if(release == true){
      goal.position = 1.001;
      goal.speed = 1.1;
      goal.force = 400.0;
  }
  else{
      goal.position = 1.001;
      goal.speed = 1.1;
      goal.force = 400.0;
  }

  grap_action_client_.sendGoalAndWait(goal);
  ROS_INFO("start send goal to Gripper");
  bool finished_before_timeout = grap_action_client_.waitForResult(ros::Duration(0.0));

  if (finished_before_timeout)                             
  {
    actionlib::SimpleClientGoalState state = grap_action_client_.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    //Result = *move_action_client_.getResult();
    return;
  }

  else
  {
    ROS_ERROR("Failed to call service");
    return;
  }
  ROS_INFO("Action did not finish before the time out.");

  return;
}
//===============================================================
void releasePoseMsgCallback(const std_msgs::String::ConstPtr& msg){

  //release_action_client_.waitForServer(ros::Duration(0.0));
  ROS_INFO("Connected to release_action server");

  ROS_INFO("Action server started.");

  if(msg->data=="Gripper_release")
  {
    release = true;
    grap = false;
    ROS_INFO("start send Release gripper");
    if(release == true && grap == false){
      robotiq_2f_gripper_Client();
      // tra_gene_thread_ = new boost::thread(boost::bind(&GripperControl::robotiq_2f_gripper_Client, this));
      // delete tra_gene_thread_;
    }
  }
}
//===============================================================
void grapPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  std::cout<<"HHHHHHHHHHHHHH"<<std::endl;
  ROS_INFO("Connected to grap_action server");

  ROS_INFO("Action server started.");
  //robotis_->is_ik = false;

  
  if(msg->data=="Gripper_grap")
  {
    grap = true;
    grap_alc = true;
    ROS_INFO("start send Release gripper");
    if(grap == true && grap_alc == true){
      robotiq_2f_gripper_Client();
      // tra_gene_thread_ = new boost::thread(boost::bind(&GripperControl::robotiq_2f_gripper_Client, this));
      // delete tra_gene_thread_;
    }
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv,"gripper_node");

  //ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle    gripper_node;
  // ros::CallbackQueue callback;
  
  //////////////////////////robitq2f_85////////////////////////////////////////////////////////////////////////                                                        
  // ros::Subscriber grap_pose_msg_sub = gripper_node.subscribe("grap_pose_msg", 5,
  //                                                              &GripperControl::grapPoseMsgCallback, this); 
  // ros::Subscriber release_pose_msg_sub = gripper_node.subscribe("release_pose_msg", 5,
  //                                                              &GripperControl::releasePoseMsgCallback, this);    
  ros::Subscriber grap_pose_msg_sub = gripper_node.subscribe("grap_pose_msg", 0, &grapPoseMsgCallback); 
  ros::Subscriber release_pose_msg_sub = gripper_node.subscribe("release_pose_msg", 0, &releasePoseMsgCallback);                                                            
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // while (gripper_node.ok())
  // {
  //   callback.callAvailable();
  //   usleep(1000);
  // }
  ros::spin();

  //return 0;
}
// //==========================================roboticq2f_85======================================================
// void GripperControl::robotiq_2f_gripper_Client(){
//   actionlib::SimpleActionClient<robotiq_2f_gripper_msgs::CommandRobotiqGripperAction> grap_action_client_("/command_robotiq_action");
//   //grap_action_client_.waitForServer(ros::Duration(0.0));
//   ROS_INFO("Connected to grap_action server");

//   ROS_INFO("Action server started.");
//   robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal goal;

//   bool False = false;
//   goal.emergency_release = False;
//   goal.stop = False;
//   std::cout<<grap_alc<<std::endl;
//   if(grap_alc == true){
//       std::cout<<"GGGGG"<<std::endl;
//       goal.position = 0.065;
//       goal.speed = 0.1;
//       goal.force = 0.5;
//   }
//   else if(grap_scraper == true){
//       goal.position = 0.04;
//       goal.speed = 0.1;
//       goal.force = 400;
//   }
//   else if(release == true){
//       goal.position = 1.001;
//       goal.speed = 1.1;
//       goal.force = 400.0;
//   }
//   else{
//       std::cout<<"FFFFFF"<<std::endl;
//       goal.position = 1.001;
//       goal.speed = 1.1;
//       goal.force = 400.0;
//   }

//   grap_action_client_.sendGoalAndWait(goal);
//   ROS_INFO("start send goal to Gripper");
//   bool finished_before_timeout = grap_action_client_.waitForResult(ros::Duration(0.0));

//   if (finished_before_timeout)                             
//   {
//     actionlib::SimpleClientGoalState state = grap_action_client_.getState();
//     ROS_INFO("Action finished: %s",state.toString().c_str());
//     //Result = *move_action_client_.getResult();
//     return;
//   }

//   else
//   {
//     ROS_ERROR("Failed to call service");
//     return;
//   }
//   ROS_INFO("Action did not finish before the time out.");

//   return;
// }
//===============================================================
// void GripperControl::releasePoseMsgCallback(const std_msgs::String::ConstPtr& msg){

//   //release_action_client_.waitForServer(ros::Duration(0.0));
//   ROS_INFO("Connected to release_action server");

//   ROS_INFO("Action server started.");

//   if(msg->data=="Gripper_release")
//   {
//     release = true;
//     grap = false;
//     ROS_INFO("start send Release gripper");
//     if(release == true && grap == false){
//       robotiq_2f_gripper_Client();
//       // tra_gene_thread_ = new boost::thread(boost::bind(&GripperControl::robotiq_2f_gripper_Client, this));
//       // delete tra_gene_thread_;
//     }
//   }
// }
// //===============================================================
// void GripperControl::grapPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
// {
//   std::cout<<"HHHHHHHHHHHHHH"<<std::endl;
//   ROS_INFO("Connected to grap_action server");

//   ROS_INFO("Action server started.");
//   //robotis_->is_ik = false;

  
//   if(msg->data=="Gripper_grap")
//   {
//     grap = true;
//     grap_alc = true;
//     ROS_INFO("start send Release gripper");
//     if(grap == true && grap_alc == true){
//       robotiq_2f_gripper_Client();
//       // tra_gene_thread_ = new boost::thread(boost::bind(&GripperControl::robotiq_2f_gripper_Client, this));
//       // delete tra_gene_thread_;
//     }
//   }
// }
//=========================================================================================================
// //==============ScrapClean=====================================================================================
// void BaseModule::ScrapCleanPoseMsgCallback(const manipulator_h_base_module_msgs::P2PPose::ConstPtr& msg)
// {

// }
// //==============================================================================================================
