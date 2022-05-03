#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <iostream>
#include <math.h>

using namespace std;

class MyController {
public:
  MyController(const geometry_msgs::Pose2D passedGoal) : goal_(passedGoal)
  {
    //odomSub_ = myNodeH_.subscribe<nav_msgs::Odometry>(
    //    "/odom", 5, &MyController::OdomCallback, this);
    //twistPub_ = myNodeH_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }
  ros::NodeHandle myNodeH_;

private:
  ros::Subscriber odomSub_;
  ros::Subscriber laserSub_;
  ros::Publisher twistPub_;
  geometry_msgs::Pose2D pose_;
  geometry_msgs::Pose2D goal_;
  geometry_msgs::Twist robotTwist_;
  double roll_, pitch_, yaw_, diffX_, diffY_, angleToGoal_;
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);
};

class AvoidObst {
public:
  AvoidObst(ros::NodeHandle nodeHandle)
  {
    laserSub_ = nodeHandle.subscribe<sensor_msgs::LaserScan>(
        "/kobuki/laser/scan", 10, &AvoidObst::LaserCallback, this);
    twistAvoidPub_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  };
  vector<string> robotRegions =
  {
    "rightA",  "rightB",  "rightC",
    "frontA", "frontB", "frontC",
    "leftA", "leftB", "leftC"
  };
  std::map<string, vector<double>> myLaserDistanceMap =
  {
    {"rightA", {0}},  {"rightB", {0}},  {"rightC", {0}},
    {"frontA", {0}}, {"frontB", {0}}, {"frontC", {0}},
    {"leftA", {0}}, {"leftB", {0}}, {"leftC", {0}}
  };
  
  std::map<string, int> myCostMap =
  {
    {"rightA", -4},   {"rightB", -3},   {"rightC", -2},
    {"frontA", 1},  {"frontB", 0},  {"frontC", 1},
    {"leftA", 2}, {"leftB", 3}, {"leftC", 4}
  };

private:
  ros::Subscriber laserSub_;
  ros::Publisher twistAvoidPub_;
  double laserValue_;
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void MakePairDistances(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  geometry_msgs::Twist robotTwistAvoid_;
};

void AvoidObst::MakePairDistances(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  int index = 0;
  int indexOld = 0;
  for (auto val : robotRegions)
  {
    std::vector<double> myVector;
    for (index = indexOld; index <= indexOld + 79 && index <= 720; index++)
    {
      if (laserMsg->ranges[index] < 1.5)
        myVector.push_back(laserMsg->ranges[index]);
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

void AvoidObst::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  string straightPath = "frontB";
  float cheapestCost = 10;
  string destinationPath;
  float warningDistance=0;
  float angValue;
  MakePairDistances(laserMsg);
  float runtimeCost = 0;
  for (auto val : myLaserDistanceMap)
  {
    runtimeCost = abs(myCostMap[val.first] - myCostMap[straightPath]);
    if (val.second.size() <= 1)
    {
      if (runtimeCost < cheapestCost) {
        cheapestCost = runtimeCost;
        warningDistance = 2;
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
  runtimeCost = myCostMap[destinationPath] - myCostMap[straightPath];

  if(runtimeCost!=0)
  {
    angValue=( runtimeCost / (abs(runtimeCost)) )*1.75;
  }
  else
  {
    angValue=0;
  }
  if (cheapestCost!=0)
  {
    robotTwistAvoid_.linear.x = 0.0;
    robotTwistAvoid_.angular.z = cheapestCost*angValue;
  }
  else
  {
    robotTwistAvoid_.linear.x = 0.1;
    robotTwistAvoid_.angular.z = 0.0;
  }
  twistAvoidPub_.publish(robotTwistAvoid_);
}

void MyController::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  pose_.x = odomMsg->pose.pose.position.x;
  pose_.y = odomMsg->pose.pose.position.y;

  tf2::Quaternion q(
      odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
      odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  diffX_ = goal_.x - pose_.x;
  diffY_ = goal_.y - pose_.y;

  // cout << "Diff X = " << diffX_ << endl;
  // cout << "Diff Y = " << diffY_ << endl;

  angleToGoal_ = atan2(diffY_, diffX_);

  // cout << "angleToGoal_ = " << angleToGoal_ << endl;
  // cout << "yaw_ = " << yaw_ << endl;

  // cout << "angleToGoal_ - yaw = " << (angleToGoal_ - yaw_) << endl;

  if ((abs(angleToGoal_ - yaw_) > 0.07)) {
    robotTwist_.linear.x = 0.0;
    robotTwist_.angular.z = 0.2;
  } else {
    if ((abs(diffX_) > 0.1) || (abs(diffY_) > 0.1)) {
      robotTwist_.linear.x = 0.2;
      robotTwist_.angular.z = 0.0;
    } else {
      robotTwist_.linear.x = 0.0;
      robotTwist_.angular.z = 0.0;
    }
  }
  twistPub_.publish(robotTwist_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_controller_node");
  geometry_msgs::Pose2D goal;
  cout << "Insert the X coordinate" << endl;
  cin >> goal.x;
  cout << "Insert the Y coordinate" << endl;
  cin >> goal.y;
  MyController myControllerObj(goal);
  AvoidObst myAvoidObj(myControllerObj.myNodeH_);
  ros::spin();
  return 0;
}
