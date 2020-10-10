#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//#include "moveit_task_constructor/ExecuteTrajectoryAction.h"
#include "moveit_task_constructor/ExecuteTaskSolutionActionGoal.h"
#include "moveit_task_constructor/ExecuteTaskSolutionActionResult.h"
// #include <robot_interaction/interaction.h>  // Note: "Action" is appended
// #include <actionlib/server/simple_action_server.h>

// typedef actionlib::SimpleActionServer<chores::DoDishesAction> Server;

// void execute(const chores::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
// {
//   // Do lots of awesome groundbreaking robot stuff here
//   as->setSucceeded();
//   }
  
//   int main(int argc, char** argv)
//   {
//     ros::init(argc, argv, "do_dishes_server");
//     ros::NodeHandle n;
//     Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
//     server.start();
//     ros::spin();
//     return 0;
//   }
class ExecuteTaskSolutionAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::ExecuteTaskSolutionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // 建立用作反饋/結果的訊息
  actionlib_tutorials::ExecuteTaskSolutionActionFeedback feedback_;
  actionlib_tutorials::ExecuteTaskSolutionActionResult result_;

public:

  ExecuteTaskSolutionAction(std::string name) :
    as_(nh_, name, boost::bind(&ExecuteTaskSolutionAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ExecuteTaskSolutionAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::ExecuteTaskSolutionActionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the ExecuteTaskSolutionAction sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // 將資訊對映到螢幕上
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // 開始執行行為
    for(int i=1; i<=goal->order; i++)
    {
      // 檢查客戶端是否未請求搶佔
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        //將行為狀態設定為preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // 釋出反饋
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // 將行為狀態設定為succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ExecuteTaskSolution");

  FibonacciAction ExecuteTaskSolution("ExecuteTaskSolution");
  ros::spin();

  return 0;
}