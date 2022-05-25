#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

#define ANGLEMIN -1.57079994678
#define ANGLEINC 0.00436940183863

class OccupancyGridTest
{
private:
  ros::NodeHandle nodeH_;
  ros::Subscriber laserSub_, odomSub_;
  ros::Publisher gridPub_;
  nav_msgs::OccupancyGrid myGrid_;
  geometry_msgs::Pose2D robotPose_;
  int gridRate_, gridHeigth_, gridWidth_, robotCellSize_;
  float gridResolution_, robotFootprint_;
  tf2::Quaternion robotQuat_;
  double roll_, pitch_, yaw_;
  //std::vector<int8_t> a_;
  //std::vector<int8_t> b_;
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void SetRobotFreeCells();
  void InitGrid();
  void TransformPoint(float rayXMin, float rayYMin, float rayXMax, float rayYMax);


public:
  OccupancyGridTest(int passedRate=0, float passedResolution=0.0, 
    int passedHeigth=0, int passedWidth=0, float passedFootprint=0.0)
    : gridRate_(passedRate), gridResolution_(passedResolution), 
    gridHeigth_(passedHeigth), gridWidth_(passedWidth), robotFootprint_(passedFootprint)
    {
      robotCellSize_ = static_cast<int>(robotFootprint_ / gridResolution_);
      /*myGrid_.header.frame_id="/odom";
      myGrid_.info.resolution = gridResolution_;
      myGrid_.info.width = gridWidth_;
      myGrid_.info.height = gridHeigth_;
      a_.resize(gridWidth_*gridHeigth_, 1);
      myGrid_.data = a_;
      myGrid_.header.frame_id = "/odom";
      myGrid_.info.width = gridWidth_;
      myGrid_.info.height = gridHeigth_;
      myGrid_.info.origin.position.z = 0;
      myGrid_.info.origin.orientation.w = 1;
      myGrid_.info.origin.orientation.x = 0;
      myGrid_.info.origin.orientation.y = 0;
      myGrid_.info.origin.orientation.z = 0;
      //myGrid_.data.assign(gridWidth_*gridHeigth_, 1);
      a_.resize(gridWidth_*gridHeigth_, 1);
      myGrid_.data = a_;*/
      InitGrid();
      gridPub_ = nodeH_.advertise<nav_msgs::OccupancyGrid>("/my_grid", 10);
      laserSub_ = nodeH_.subscribe<sensor_msgs::LaserScan>(
        "/kobuki/laser/scan", 10, &OccupancyGridTest::LaserCallback, this);
      odomSub_ = nodeH_.subscribe<nav_msgs::Odometry>(
        "/odom", 5, &OccupancyGridTest::OdomCallback, this);
    }

};


void OccupancyGridTest::InitGrid()
{
  myGrid_.header.seq = 1;
  myGrid_.header.frame_id = "/odom";
  myGrid_.info.resolution = gridResolution_;
  myGrid_.info.width = gridWidth_;
  myGrid_.info.height = gridHeigth_;
  myGrid_.info.origin.position.x = 0;
  myGrid_.info.origin.position.y = 0;
  myGrid_.info.origin.position.z = 0;
  myGrid_.info.origin.orientation.w = 1;
  myGrid_.info.origin.orientation.x = 0;
  myGrid_.info.origin.orientation.y = 0;
  myGrid_.info.origin.orientation.z = 0;
  myGrid_.data.resize(gridWidth_*gridHeigth_,-1);
  //myGrid_.data.assign(gridWidth_*gridHeigth_, 1);
  //a_.resize(gridWidth_*gridHeigth_, 1);
  //myGrid_.data = a_;
}


void OccupancyGridTest::SetRobotFreeCells()
{
  // the pose of the robot based on the grid resolution
  int robotPoseGridX = static_cast<int>(robotPose_.x / gridResolution_);
	int robotPoseGridY = static_cast<int>(robotPose_.y / gridResolution_);
  int index;

  fill(myGrid_.data.begin(), myGrid_.data.end(), 0);

  // highlightin the robot grid space
  if( (robotPoseGridX>=((robotFootprint_ / gridResolution_) / 2)) &&
        (robotPoseGridY>=((robotFootprint_ / gridResolution_) / 2)) )
  {
    for(int x = - ((robotFootprint_ / gridResolution_) / 2); x < ((robotFootprint_ / gridResolution_) / 2); ++x)
    {
      for(int y = - ((robotFootprint_ / gridResolution_) / 2); y < ((robotFootprint_ / gridResolution_) / 2); ++y)
      {        
        index = (x + robotPoseGridX) + (gridWidth_ * (y + robotPoseGridY));
        if(index>=0) myGrid_.data[index] = -1;
      }
    }
  }
  else if( (robotPoseGridX<((robotFootprint_ / gridResolution_) / 2)) &&
        (robotPoseGridY<((robotFootprint_ / gridResolution_) / 2)) )
  {
    for(int x = -robotPoseGridX; x < ((robotFootprint_ / gridResolution_) / 2); ++x)
    {
      for(int y = -robotPoseGridY; y < ((robotFootprint_ / gridResolution_) / 2); ++y)
      {        
        index = (x + robotPoseGridX) + (gridWidth_ * (y + robotPoseGridY));
        if(index>=0) myGrid_.data[index] = -1;
      }
    }
  }
  else if (robotPoseGridX<((robotFootprint_ / gridResolution_) / 2))
  {
    for(int x = -robotPoseGridX; x < ((robotFootprint_ / gridResolution_) / 2); ++x)
    {
      for(int y = - ((robotFootprint_ / gridResolution_) / 2); y < ((robotFootprint_ / gridResolution_) / 2); ++y)
      {        
        index = (x + robotPoseGridX) + (gridWidth_ * (y + robotPoseGridY));
        if(index>=0) myGrid_.data[index] = -1;
      }
    }
  }
  else if (robotPoseGridY<((robotFootprint_ / gridResolution_) / 2))
  {
    for(int x = -((robotFootprint_ / gridResolution_) / 2); x < ((robotFootprint_ / gridResolution_) / 2); ++x)
    {
      for(int y = - robotPoseGridY; y < ((robotFootprint_ / gridResolution_) / 2); ++y)
      {        
        index = (x + robotPoseGridX) + (gridWidth_ * (y + robotPoseGridY));
        if(index>=0) myGrid_.data[index] = -1;
      }
    }
  }
  myGrid_.header.seq++;  
  gridPub_.publish(myGrid_);
}

void OccupancyGridTest::TransformPoint(float rayXMin, float rayYMin, float rayXMax, float rayYMax)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::PointStamped laserPointMin, laserPointMax;

  laserPointMin.header.frame_id = "laser_sensor_link";
  laserPointMin.header.stamp = ros::Time();
  laserPointMin.point.x = rayXMin;
  laserPointMin.point.y = rayYMin;
  laserPointMin.point.z = 0.0;

  laserPointMax.header.frame_id = "laser_sensor_link";
  laserPointMax.header.stamp = ros::Time();
  laserPointMax.point.x = rayXMax;
  laserPointMax.point.y = rayYMax;
  laserPointMax.point.z = 0.0;


  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PointStamped  transformedMinPt, transformedMaxPt;
  try
  {
    transformStamped = tfBuffer.lookupTransform("laser_sensor_link", "odom", ros::Time(0),
      ros::Duration(0.1));
    
    tfBuffer.transform(laserPointMin, transformedMinPt, "odom");
    tfBuffer.transform(laserPointMax, transformedMaxPt, "odom");

  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  std::cout << "ODOM Ray X min = " << transformedMinPt.point.x << 
    " | ODOM Ray Y min =" << transformedMinPt.point.y << std::endl;
  std::cout << "ODOM Ray X max = " << transformedMaxPt.point.x << 
    " | ODOM Ray Y max =" << transformedMaxPt.point.y << std::endl;
}

// only scan of obstacles
/*void OccupancyGridTest::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  float rayBeam, rayAngle, rayX, rayY, rayAngleOdom;
  float endPointX, endPointY;
  float laserRange;
  int indexGrid;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PointStamped  transformedPt;
  //fill(myGrid_.data.begin(), myGrid_.data.end(), -1);
  for(int index=0; index<720; index++)
  {
    if(!std::isinf(laserMsg->ranges[index]))
    {
      laserRange = laserMsg->ranges[index];
      //std::cout << laserRange << std::endl;
      rayAngle = ANGLEMIN + (index * ANGLEINC);
      rayX = laserRange * cos (rayAngle);
      rayY = laserRange * sin (rayAngle);
      geometry_msgs::PointStamped laserPoint;
      laserPoint.header.frame_id = "laser_sensor_link";
      laserPoint.header.stamp = ros::Time();
      laserPoint.point.x = rayX;
      laserPoint.point.y = rayY;
      laserPoint.point.z = 0.0;
      try
      {
        transformStamped = tfBuffer.lookupTransform("laser_sensor_link", "odom", ros::Time(0),
          ros::Duration(0.5));
        
        tfBuffer.transform(laserPoint, transformedPt, "odom");
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      //std::cout << "ODOM Ray X= " << transformedPt.point.x << 
      //  " | ODOM Ray Y=" << transformedPt.point.y << std::endl;
      endPointX = transformedPt.point.x/gridResolution_;
      endPointY = transformedPt.point.y/gridResolution_;
      indexGrid = static_cast<int>(endPointX) + (gridWidth_ * static_cast<int>(endPointY));
                //std::cout << indexGrid << std::endl;
      if(indexGrid>=0 && indexGrid<250000) myGrid_.data[indexGrid] = 100;
    }
  }
  gridPub_.publish(myGrid_);
}
*/

// full scans
void OccupancyGridTest::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  float rayBeam, rayAngle, rayX, rayY, rayAngleOdom;
  float endPointX, endPointY;
  float laserRange;
  int indexGrid;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PointStamped  transformedPt;
  //fill(myGrid_.data.begin(), myGrid_.data.end(), -1);
  for(int index=0; index<720; index++)
  {
    if(std::isinf(laserMsg->ranges[index])) laserRange=5.0;
    else laserRange = laserMsg->ranges[index];
    //std::cout << laserRange << std::endl;
    rayAngle = ANGLEMIN + (index * ANGLEINC);
    rayX = laserRange * cos (rayAngle);
    rayY = laserRange * sin (rayAngle);
    geometry_msgs::PointStamped laserPoint;
    laserPoint.header.frame_id = "laser_sensor_link";
    laserPoint.header.stamp = ros::Time();
    laserPoint.point.x = rayX;
    laserPoint.point.y = rayY;
    laserPoint.point.z = 0.0;
    try
    {
      transformStamped = tfBuffer.lookupTransform("laser_sensor_link", "odom", ros::Time(0),
        ros::Duration(0.5));
      
      tfBuffer.transform(laserPoint, transformedPt, "odom");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    //std::cout << "ODOM Ray X= " << transformedPt.point.x << 
    //  " | ODOM Ray Y=" << transformedPt.point.y << std::endl;
    endPointX = transformedPt.point.x;
    endPointY = transformedPt.point.y;
    int startX = static_cast<int>(robotPose_.x/gridResolution_);
    int startY = static_cast<int>(robotPose_.y/gridResolution_);
    int endPointXint = static_cast<int>(endPointX/gridResolution_);
    int endPointYint = static_cast<int>(endPointY/gridResolution_);
    //std::cout << "endPointXint= " << endPointXint << 
    //  " | startX=" << startX << std::endl;
    m*x + n = y 
    m =  tan(angle)
    n = startPoint_Y - (tan ( angle) * startPoint_X )
    

    for (int x=startX; x<=endPointXint; x++) 
    { 
      for (int y=startY; y<=endPointYint; y++) 
      { 
          //std::cout << "(" << x << "," << y << ")" << std::endl;
          //(y2-y1)x+(x2-x1)y+(x1y2-x2y1) = 0
          float testValue;
          testValue = ((endPointYint-startY)*x + (endPointXint-startX)*y +
              (startX*endPointYint - endPointXint*startY));
        // std::cout << "Test value= " << testValue << std::endl;
          if (abs(testValue)<1000000)
          { 
              //std::cout << "(" << x << "," << y << ")" << std::endl;
              indexGrid = x + (gridWidth_ * y);
              //std::cout << indexGrid << std::endl;
              if(indexGrid>=0 && indexGrid<250000) myGrid_.data[indexGrid] = 0; 
              //break; 
          } 
      } 
    }
  }
  gridPub_.publish(myGrid_);
}

/*void OccupancyGridTest::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  int indexOld = 0;
  int indexGrid;
  int rayCounter = 0;
  float rayAngleMin, rayAngleMax, rayAngle;
  float rayXMin, rayXMax, rayYMin, rayYMax, rayX, rayY;
  float endPointX, endPointY;
  std::vector<float> myBeamsVector;
  std::vector<int> myBeamsIndex;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  //fill(myGrid_.data.begin(), myGrid_.data.end(), 0);
  for (int index = 0; index < 2; index++)
  {
    //if (laserMsg->ranges[index] < 8)
    //{
      //myBeamsVector.push_back(laserMsg->ranges[index]);
      //myBeamsIndex.push_back(index);
      rayCounter++;
      rayAngle = ANGLEMIN + (index * ANGLEINC);
      //std::cout << "Ray n." << index;
      //std::cout << " Ray Angle = " << rayAngle;
      //std::cout << " Ray Distance = " << laserMsg->ranges[index] << std::endl;
      rayX = laserMsg->ranges[index] * cos (rayAngle);
      rayY = laserMsg->ranges[index] * sin (rayAngle);
      geometry_msgs::PointStamped laserPoint;

      laserPoint.header.frame_id = "laser_sensor_link";
      laserPoint.header.stamp = ros::Time();
      laserPoint.point.x = rayX;
      laserPoint.point.y = rayY;
      laserPoint.point.z = 0.0;
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::PointStamped  transformedPt;
      try
      {
        transformStamped = tfBuffer.lookupTransform("laser_sensor_link", "odom", ros::Time(0),
          ros::Duration(0.5));
        
        tfBuffer.transform(laserPoint, transformedPt, "odom");

      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      //std::cout << "ODOM Ray X= " << transformedPt.point.x << 
      //  " | ODOM Ray Y=" << transformedPt.point.y << std::endl;
      /*endPointX = transformedPt.point.x + 0.5 * cos(rayAngle);
      endPointY = transformedPt.point.y + 0.5 * sin(rayAngle);
      //(x1 + l * cos(ang), y1 + l * sin(ang))

      for(int x = static_cast<int>(transformedPt.point.x / gridResolution_); 
        x < static_cast<int>(endPointX / gridResolution_); ++x)
      {
        for(int y = static_cast<int>(transformedPt.point.y / gridResolution_); 
          y < static_cast<int>(endPointY / gridResolution_); ++y)
        {        
          index = x + (gridWidth_ * y);
          if(index>=0) myGrid_.data[index] = 100;
        }
      }
      
      indexGrid = static_cast<int>(transformedPt.point.x / gridResolution_) + 
                  (gridWidth_ * static_cast<int>(transformedPt.point.y / gridResolution_));
      
      if (indexGrid>0)
        myGrid_.data[indexGrid] = 100;
      
          //std::cout << "Ray X = " << rayX_ << " | Ray Y =" << rayY_ << std::endl;
          
    //}
    else
    {
      rayAngle = ANGLEMIN + (index * ANGLEINC);
      rayX = robotPose_.x + 8 * cos(rayAngle);
      rayY = robotPose_.y + 8 * sin(rayAngle);
      int dx = static_cast<int>(rayX - robotPose_.x);
      int dy = static_cast<int>(rayY - robotPose_.y);
      std::cout <<"robotPose_.x=" << robotPose_.x << " robotPose_.y"  << robotPose_.y << std::endl;
      std::cout << "rayx=" << rayX << " rayy=" << rayY
        << " dx=" << dx << " dy=" << dy << std::endl;

      for(int x = static_cast<int>(robotPose_.x / gridResolution_); 
        x < static_cast<int>(rayX / gridResolution_); ++x)
      {
        int y = static_cast<int>((robotPose_.y / gridResolution_) + (dy / gridResolution_) * (x - (robotPose_.x / gridResolution_)) / (dx / gridResolution_));   
        index = x + (gridWidth_ * y);
        //std::cout << y << std::endl;
        //if(index>=0) myGrid_.data[index] = 0;
      }
    }
  }
  //gridPub_.publish(myGrid_);
  if(rayCounter>0)
  {
    rayAngleMin = ANGLEMIN + (myBeamsIndex[0] * ANGLEINC);
    rayXMin = myBeamsVector[0] * cos (rayAngleMin);
    rayYMin = myBeamsVector[0] * sin (rayAngleMin);
    std::cout << "Ray x min = " <<  rayXMin << std::endl;

    rayAngleMax = ANGLEMIN + (myBeamsIndex[myBeamsIndex.size()-1] * ANGLEINC);
    rayXMax = myBeamsVector[myBeamsVector.size()-1] * cos (rayAngleMax);
    rayYMax = myBeamsVector[myBeamsVector.size()-1] * sin (rayAngleMax);
    std::cout << "Ray x max = " <<  rayXMax << std::endl;

  }

  TransformPoint(rayXMin, rayYMin, rayXMax, rayYMax);
  //rayAngle = angleMin + (320 * angleIncrement);
  //std::cout << "Ray n." << index;
  //std::cout << " Ray Angle = " << rayAngle;
  //std::cout << " Ray Distance = " << laserMsg->ranges[index] << std::endl;
  //rayX_ = laserMsg->ranges[320] * cos (rayAngle);
  //rayY_ = laserMsg->ranges[320] * sin (rayAngle);
  //if(rayCounter>0) 
  //  std::cout << "Min Distance = " << 
  //    *(min_element(std::begin(myVector), std::end(myVector))) << std::endl;
  
}*/


void OccupancyGridTest::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
  robotPose_.x = odomMsg->pose.pose.position.x;
  robotPose_.y = odomMsg->pose.pose.position.y;
  robotQuat_ = {
    odomMsg->pose.pose.orientation.x,
    odomMsg->pose.pose.orientation.y,
    odomMsg->pose.pose.orientation.z,
    odomMsg->pose.pose.orientation.w};
  //SetRobotFreeCells();
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_test");
  OccupancyGridTest myObj(5, 0.01, 500, 500, 0.5);
  ros::spin();
  return 0;
}