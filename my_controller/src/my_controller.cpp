/**
 * @file my_controller.cpp
 *
 * @brief A simple controller for a TurtleBot equipped with hokuyo laser
 *        The user enters the x,y goal to reach and the controller will
 *        adjust the heading as first task, going straigth later adjusting the heading
 *        when necessary. It's also implemented an obstacle avoidance routine based
 *        on the laser scans: the robot will take the closest free path to its front side
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>
#include <iostream>
#include <math.h>
#include <algorithm>

#define DEBUGP 0
#define STRAIGHTPATH "frontB"
#define YAWPRECISION M_PI / 90
#define POSEPRECISION 0.2f
#define STATE0 "[STATE MACHINE] State changed to Adjust Heading"
#define STATE1 "[STATE MACHINE] State changed to Straight To The Goal"
#define STATE2 "[STATE MACHINE] State changed to Done \"GOAL REACHED\""

using namespace std;

class MyController {
public:
  MyController(const geometry_msgs::Pose2D passedGoal) 
    : goal_(passedGoal)
  {
    ChangeStateMachine(0);
    odomSub_ = myNodeH_.subscribe<nav_msgs::Odometry>(
      "/odom", 5, &MyController::OdomCallback, this);
    laserSub_ = myNodeH_.subscribe<sensor_msgs::LaserScan>(
      "/kobuki/laser/scan", 10, &MyController::LaserCallback, this);
    twistPub_ = myNodeH_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  // the list of the sides/front regions of the robot 
  vector<string> robotRegions =
  {
    "rightA",  "rightB",  "rightC",
    "frontA", "frontB", "frontC",
    "leftA", "leftB", "leftC"
  };

  // the laser distances detected in every regions
  std::map<string, vector<double>> myLaserDistanceMap =
  {
    {"rightA", {0}},  {"rightB", {0}},  {"rightC", {0}},
    {"frontA", {0}}, {"frontB", {0}}, {"frontC", {0}},
    {"leftA", {0}}, {"leftB", {0}}, {"leftC", {0}}
  };
  
  // the cost map of the robot regions related to the font-center one
  std::map<string, int> myCostMap =
  {
    {"rightA", -4},   {"rightB", -3},   {"rightC", -2},
    {"frontA", -1},  {"frontB", 0},  {"frontC", 1},
    {"leftA", 2}, {"leftB", 3}, {"leftC", 4}
  };
  
private:
  ros::NodeHandle myNodeH_;
  ros::Subscriber odomSub_;
  ros::Subscriber laserSub_;
  ros::Publisher twistPub_;
  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D goal_;
  geometry_msgs::Twist robotTwist_;
  double roll_, pitch_, yaw_;
  float diffX_, diffY_, angleToGoal_, yawError_, posError_;
  int state_;
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void MakePairDistances(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  float NormalizeAngleToGoal(float angleToNormalize);
  void ChangeStateMachine(int newState);
  void ManageYawToGoal();
  void ManageGoStraight();
  void DoneTask();
  void DoAvoidance();
};

// method to change the state of the robot
void MyController::ChangeStateMachine(int newState)
{
  state_ = newState;
  if( newState == 0) cout << STATE0 << endl;
  else if ( newState == 1) cout << STATE1 << endl;
  else if ( newState == 2) cout << STATE2 << endl;

}

// laser scan callback which fills the myLaserDistanceMap
void MyController::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  int index = 0;
  int indexOld = 0;
  for (auto val : robotRegions)
  {
    std::vector<double> myVector;
    for (index = indexOld; index <= indexOld + 79 && index <= 720; index++)
    {
      if (laserMsg->ranges[index] < 2.0f)
      {
        myVector.push_back(laserMsg->ranges[index]);
      }
    }
    if (myLaserDistanceMap.find(val) == myLaserDistanceMap.end())
    {
      myLaserDistanceMap.insert(std::make_pair(val, myVector));
    }
    else
    {
      myLaserDistanceMap[val] = myVector;
    }
    indexOld = index;
  }
}

// obstacle avoidance task
void MyController::DoAvoidance()
{
    float cheapestCost = 10;
    string destinationPath;
    float warningDistance=0;
    float angValue;
    float runtimeCost = 0;
    for (auto val : myLaserDistanceMap)
    {
      // calculation of the direction cost related to the front center
      // to identify the closest free path to the robot front center
      runtimeCost = abs(myCostMap[val.first] - myCostMap[STRAIGHTPATH]);
      if (val.second.size() < 1)
      {
        if (runtimeCost < cheapestCost)
        {
          cheapestCost = runtimeCost;
          warningDistance = 1.0f;
          destinationPath = val.first;
        }
      }
      else if (*(max_element(std::begin(val.second), std::end(val.second))) >
                warningDistance)
      {
        warningDistance = *(max_element(std::begin(val.second), std::end(val.second)));
        destinationPath = val.first;
      }
    }

    if(DEBUGP)
    {
      cout << "Destination : " << destinationPath << endl;
      cout << "Cheapest cost : " << cheapestCost << endl;
    }
    runtimeCost = myCostMap[destinationPath] - myCostMap[STRAIGHTPATH]; //2

    // if there's a free path, rotate to the closest to the front center
    if(runtimeCost!=0)
    {
      angValue=( runtimeCost / (abs(runtimeCost)) )*1.75;
    }
    else
    {
      angValue=0;
    }

    if(DEBUGP)
    {
      cout << "angValue : " << angValue << endl;
    }
    
    // rotate if the closest free path is not the front center
    // otherwise go straight
    if (cheapestCost!=0)
    {
      robotTwist_.linear.x = 0.0;
      robotTwist_.angular.z = cheapestCost*angValue;
    }
    else
    {
      robotTwist_.linear.x = 0.2;
      robotTwist_.angular.z = 0.0;
    }
    twistPub_.publish(robotTwist_);
}

// maps the angle full range to the range zero to one
float MyController::NormalizeAngleToGoal(float angleToNormalize)
{
  if(angleToNormalize > M_PI)
    angleToNormalize = angleToNormalize - (2 * M_PI * angleToNormalize) / (abs(angleToNormalize));
  return angleToNormalize;
}

// adjust the heading based on the goal position
void MyController::ManageYawToGoal()
{
  angleToGoal_ = atan2( (goal_.y - pose_.y) , (goal_.x - pose_.x) );
  yawError_ = NormalizeAngleToGoal(angleToGoal_ - yaw_);

  if (DEBUGP)
  {
    cout << "yawError_ not norm = " << (angleToGoal_ - yaw_) << endl;
    cout << "yawError_ = " << yawError_ << endl;
    cout << "angleToGoal_ = " << angleToGoal_ << endl;
  }

  if ( abs(yawError_) > YAWPRECISION )
    robotTwist_.angular.z = (yawError_> 0) ? -0.3 : 0.3;
  else
  {
    robotTwist_.angular.z = 0.0;
    ChangeStateMachine(1);
  }
  robotTwist_.linear.x = 0.0;  
  
  twistPub_.publish(robotTwist_);
}

// go straight until the goal is reached
// adjusting the heading
void MyController::ManageGoStraight()
{
  angleToGoal_ = atan2( (goal_.y - pose_.y) , (goal_.x - pose_.x) );
  yawError_ = NormalizeAngleToGoal(angleToGoal_ - yaw_);
  posError_ = sqrt( pow((goal_.y - pose_.y), 2) + pow((goal_.x - pose_.x), 2) );

  if (posError_ > POSEPRECISION)
  {
    robotTwist_.linear.x = 0.3;
    robotTwist_.angular.z = 0.0;
  }
  else if (posError_ <= POSEPRECISION)
  {
    ChangeStateMachine(2);
  }

  if ( abs(yawError_) > (YAWPRECISION) )
    ChangeStateMachine(0);

  twistPub_.publish(robotTwist_);
}


// goal is reached
void MyController::DoneTask()
{
  robotTwist_.linear.x = 0.0;
  robotTwist_.angular.z = 0.0;
  twistPub_.publish(robotTwist_);
}


// callback function of the odometry message where the robot state is handled
void MyController::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
  pose_.x = odomMsg->pose.pose.position.x;
  pose_.y = odomMsg->pose.pose.position.y;

  tf2::Quaternion q(
      odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
      odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  if(state_ == 0)
    ManageYawToGoal();
  else if(state_ == 1)
    ManageGoStraight();
  else if(state_ == 2)
    DoneTask();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_controller_node");
  geometry_msgs::Pose2D goal;
  cout << "Insert the X coordinate" << endl;
  cin >> goal.x;
  cout << "Insert the Y coordinate" << endl;
  cin >> goal.y;
  MyController myControllerObj(goal);
  ros::spin();
  return 0;
}
