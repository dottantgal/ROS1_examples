#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "ros/subscriber.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/LaserScan.h"

#include <math.h>
#include <iostream>

using namespace std;

class MyController
{
public:
  MyController(const geometry_msgs::Pose2D passedGoal) :
    goal_(passedGoal)
  {
    odomSub_ = myNodeH_.subscribe<nav_msgs::Odometry>("/odom", 5, &MyController::OdomCallback, this);
    twistPub_ = myNodeH_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  
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
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
  

};


class AvoidObst
{
public:
  AvoidObst(ros::NodeHandle nodeHandle)
  {
    laserSub_ = nodeHandle.subscribe<sensor_msgs::LaserScan>("/kobuki/laser/scan", 10, &AvoidObst::LaserCallback, this);
  };
  vector<string> robotRegions =
    {
      "leftA", "leftB", "leftC" ,
      "frontA" , "frontB" , "frontC" ,
      "rightA" , "rightB" , "rightC"
    };
  std::map<string, vector<double>> myLaserDistanceMap = 
    {
      {"leftA", {0}}, {"leftB", {0}}, {"leftC" , {0}},
      {"frontA" , {0}}, {"frontB" , {0}}, {"frontC" , {0}},
      {"rightA" , {0}}, {"rightB" , {0}}, {"rightC" , {0}}
    };
  std::map<string, int> myCostMap
    {
      {"leftA", 4}, {"leftB", 3}, {"leftC" , 2},
      {"frontA" , 1}, {"frontB" , 0}, {"frontC" , 1},
      {"rightA" , 2}, {"rightB" , 3}, {"rightC" , 4}
    };
private:
  ros::Subscriber laserSub_;
  double laserValue_;
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
  void MakePairDistances(const sensor_msgs::LaserScan::ConstPtr& laserMsg);

};


void AvoidObst::MakePairDistances(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
  int index=0;
  int indexOld=0;
  for(auto val : robotRegions)
  {
    std::vector<double> myVector;
    for(index=indexOld; index<=indexOld+80 && index<=720; index++)
    {
      if(laserMsg->ranges[index]<1.1)
        myVector.push_back(laserMsg->ranges[index]);
      else myVector.push_back(0);
    }
    if (myLaserDistanceMap.find(val) == myLaserDistanceMap.end())
    {
      myLaserDistanceMap.insert( std::make_pair(val, myVector) );
    }
    else
    {
        myLaserDistanceMap[val] = myVector;
    }
    indexOld = index;
  }
}


void AvoidObst::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
  laserValue_ = laserMsg->ranges[360];
  MakePairDistances(laserMsg);
  for (auto itr = myLaserDistanceMap.begin(); itr != myLaserDistanceMap.end(); ++itr)  
  {
      cout << "\t " << itr->first << "\t";
      for(auto itr2=itr->second.begin(); itr2!=itr->second.end(); ++itr2) std::cout << *itr2 << " ";
      std::cout << '\n';
  }
}


void MyController::OdomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  pose_.x = odomMsg->pose.pose.position.x;
  pose_.y = odomMsg->pose.pose.position.y;
  
  tf2::Quaternion q(odomMsg->pose.pose.orientation.x,
                    odomMsg->pose.pose.orientation.y,
                    odomMsg->pose.pose.orientation.z,
                    odomMsg->pose.pose.orientation.w);
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  diffX_ = goal_.x - pose_.x;
  diffY_ = goal_.y - pose_.y;

  cout << "Diff X = " << diffX_ << endl;
  cout << "Diff Y = " << diffY_ << endl;
  
  angleToGoal_ = atan2(diffY_, diffX_);
  
  cout << "angleToGoal_ = " << angleToGoal_ << endl;
  cout << "yaw_ = " << yaw_ << endl;

  cout << "angleToGoal_ - yaw = " << (angleToGoal_ - yaw_) << endl;
  
  if( (abs(angleToGoal_ - yaw_) > 0.07))
  {
    robotTwist_.linear.x = 0.0;
    robotTwist_.angular.z = 0.2;
  }
  else 
  {
    if( (abs(diffX_)>0.1) || (abs(diffY_)>0.1) )
    {
      robotTwist_.linear.x = 0.2;
      robotTwist_.angular.z = 0.0;
    }
    else
    {
      robotTwist_.linear.x = 0.0;
      robotTwist_.angular.z = 0.0;
    }
  }
  twistPub_.publish(robotTwist_);

}


int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_controller_node");
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