/**
 * @file simple_action.cpp
 *
 * @brief A simple node to handle Controller.action goals
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include <ros/ros.h>
#include "my_controller/ControllerAction.h"
#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

class MyController
{
private:
  ros::NodeHandle nodeH_;
  actionlib::SimpleActionServer<my_controller::ControllerAction> actionServer_;
  my_controller::ControllerFeedback feedback_;
  my_controller::ControllerResult result_;
  geometry_msgs::Pose2D robotGoal_;
  ros::Subscriber sub_;
  ros::Publisher twistPub_;
  geometry_msgs::Twist robotTwist_;

public:
  MyController(std::string passedName) :
    actionServer_(nodeH_, passedName, false)
  {
    //register the goal and feeback callbacks
    actionServer_.registerGoalCallback(boost::bind(&MyController::GoalCB, this));
    actionServer_.registerPreemptCallback(boost::bind(&MyController::PreemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nodeH_.subscribe<nav_msgs::Odometry>("/odom", 1, &MyController::OdomCB, this);
    twistPub_ = nodeH_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    actionServer_.start();
  }

  ~MyController(void)
  {
  }

  void GoalCB()
  {
    robotGoal_ = actionServer_.acceptNewGoal()->robot_goal;
    std::cout << robotGoal_.x << "  " << robotGoal_.y << std::endl;
  }

  void PreemptCB()
  {
    ROS_INFO("Preempted");
    actionServer_.setPreempted();
  }

  void OdomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
  {
    feedback_.actual_pose.x = odomMsg->pose.pose.position.x;
    feedback_.actual_pose.y = odomMsg->pose.pose.position.y;
    if (!actionServer_.isActive())
      return;
    actionServer_.publishFeedback(feedback_);

    if(feedback_.actual_pose.x > robotGoal_.x)
    {
      ROS_INFO("Succeeded");
      result_.reached_goal = true;
      robotTwist_.linear.x = 0.0;
      actionServer_.setSucceeded(result_);
      
    }
    else robotTwist_.linear.x = 0.1;
    twistPub_.publish(robotTwist_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_controller_action_server");

  MyController simpleAction(ros::this_node::getName());
  ros::spin();

  return 0;
}
