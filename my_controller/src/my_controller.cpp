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
#define STATE0 "[STATE MACHINE] State changed to Adjust Heading"
#define STATE1 "[STATE MACHINE] State changed to Straight To The Goal"

using namespace std;

class MyController {
public:
  MyController(const geometry_msgs::Pose2D passedGoal) 
    : goal_(passedGoal)
  {
    ControlStateMachine(0);
    odomSub_ = myNodeH_.subscribe<nav_msgs::Odometry>(
      "/odom", 5, &MyController::OdomCallback, this);
    laserSub_ = myNodeH_.subscribe<sensor_msgs::LaserScan>(
      "/kobuki/laser/scan", 10, &MyController::LaserCallback, this);
    twistPub_ = myNodeH_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

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
  float diffX_, diffY_, angleToGoal_, yawError_;
  int state_;
  bool obstacleAlert_;
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void MakePairDistances(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void ManageYawToGoal();
  float NormalizeAngleToGoal(float angleToNormalize);
  void ControlStateMachine(int newState);
  void DoAvoidance();
};


void MyController::ControlStateMachine(int newState)
{
  state_ = newState;
  if( newState == 0) cout << STATE0 << endl;
  else if ( newState == 1) cout << STATE1 << endl;

}


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


void MyController::DoAvoidance()
{
    float cheapestCost = 10;
    string destinationPath;
    float warningDistance=0;
    float angValue;
    
    float runtimeCost = 0;
    for (auto val : myLaserDistanceMap)
    {
      // calc the cost of the side compared with the straight path cost 0
      runtimeCost = abs(myCostMap[val.first] - myCostMap[STRAIGHTPATH]); //4
      if (DEBUGP) // ok just ray < X are saved
        cout << val.first << "  " << val.second.size()  << endl;
      if (val.second.size() < 1)
      {
        if (runtimeCost < cheapestCost) //4 < 10 
        {
          cheapestCost = runtimeCost; // 10----> 4
          warningDistance = 1.0f;
          destinationPath = val.first; // rightA
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

// maps the full range of angles to the range zero to one
float MyController::NormalizeAngleToGoal(float angleToNormalize)
{
  float returnAngle;
  if(angleToNormalize > M_PI)
    angleToNormalize = angleToNormalize - (2 * M_PI * angleToNormalize) / (abs(angleToNormalize));
  return angleToNormalize;
}


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
  {
    if(yawError_> 0) robotTwist_.angular.z = -0.5;
    else if (yawError_ < 0) robotTwist_.angular.z = 0.5;      
  }
  else
  {
    robotTwist_.angular.z = 0.0;
    ControlStateMachine(1);
  }
  robotTwist_.linear.x = 0.0;  
  twistPub_.publish(robotTwist_);
}


void MyController::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
  pose_.x = odomMsg->pose.pose.position.x;
  pose_.y = odomMsg->pose.pose.position.y;

  tf2::Quaternion q(
      odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
      odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);
  
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);


  ManageYawToGoal();

  // cout << "angleToGoal_ = " << angleToGoal_ << endl;
  // cout << "yaw_ = " << yaw_ << endl;

  // cout << "angleToGoal_ - yaw = " << (angleToGoal_ - yaw_) << endl;
 /* if(state_==0)ManageYawToGoal();
  else if(state_==1)DoAvoidance();
  else
  {
    robotTwist_.linear.x = 0.0;
    robotTwist_.angular.z = 0.0;
  }
  twistPub_.publish(robotTwist_);

  
  else 
  {
    if ((abs(diffX_) > 0.1) || (abs(diffY_) > 0.1))
    {
      DoAvoidance();
    } else 
    {
      robotTwist_.linear.x = 0.0;
      robotTwist_.angular.z = 0.0;
    }
  }
  twistPub_.publish(robotTwist_);
*/
}

int main(int argc, char **argv) {
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
