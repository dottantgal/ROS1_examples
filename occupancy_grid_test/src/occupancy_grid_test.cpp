/**
 * @file occupancy_grid_test.cpp
 *
 * @brief A simple node that is able to create an Occupancy Grid map based on 
 *        odometry and laser scan data. The node has been developed and test
 *        with a TurtleBot equipped with a kobuki laser
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
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
  int gridRate_, gridHeigth_, gridWidth_;
  float gridResolution_, robotFootprint_;
  float slopM_;
  int endPointXint_, endPointYint_;

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void SetRobotFreeCells();
  void InitGrid();
  void FillGridFree(const int x, const int y);
  void FillGridOccupied();
  // void TransformPoint(float rayXMin, float rayYMin, float rayXMax, float rayYMax);


public:
  OccupancyGridTest(int passedRate=0, float passedResolution=0.0, 
    int passedHeigth=0, int passedWidth=0, float passedFootprint=0.0)
    : gridRate_(passedRate), gridResolution_(passedResolution), 
    gridHeigth_(passedHeigth), gridWidth_(passedWidth), robotFootprint_(passedFootprint)
  {
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
}


void OccupancyGridTest::SetRobotFreeCells()
{
  // the pose of the robot based on the grid resolution
  int robotPoseGridX = static_cast<int>(robotPose_.x / gridResolution_);
	int robotPoseGridY = static_cast<int>(robotPose_.y / gridResolution_);
  int index;

  //fill(myGrid_.data.begin(), myGrid_.data.end(), 0);

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


/*
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
*/

void OccupancyGridTest::FillGridFree(const int x, const int y)
{
  int indexGrid;
  float testEquation = (y-endPointYint_)-(slopM_ * (x - endPointXint_));
  if( abs(testEquation) < 0.3)
  {
    indexGrid = x + (myGrid_.info.height * y);
    if( indexGrid>=0 && indexGrid<(myGrid_.info.width*myGrid_.info.height) ) 
      myGrid_.data[indexGrid] = 0; 
  } 
}


void OccupancyGridTest::FillGridOccupied()
{
  int indexGrid;
  indexGrid = endPointXint_ + (myGrid_.info.height *  endPointYint_);
  if( indexGrid>=0 && indexGrid<(myGrid_.info.width*myGrid_.info.height) ) 
      myGrid_.data[indexGrid] = 100;
}


void OccupancyGridTest::LaserCallback(
    const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
  float rayAngle, rayX, rayY;
  float laserRange;
  float slopM;
  int indexGrid;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PointStamped  transformedPt;
  
  //fill(myGrid_.data.begin(), myGrid_.data.end(), -1);
  for(int index=0; index<720; index++)
  {
    if( std::isinf(laserMsg->ranges[index]) || (laserMsg->ranges[index]>=2.0) ) laserRange=2.0;
    else if(laserMsg->ranges[index]<2.0) laserRange = laserMsg->ranges[index];

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

    int startX = static_cast<int>(robotPose_.x/gridResolution_);
    int startY = static_cast<int>(robotPose_.y/gridResolution_);
    endPointXint_ = static_cast<int>(transformedPt.point.x/gridResolution_);
    endPointYint_ = static_cast<int>(transformedPt.point.y/gridResolution_);
    
    try
    {
      slopM_ = (static_cast<float>(endPointYint_ - startY) / static_cast<float>(endPointXint_ - startX));
    }
    catch(...)
    {
      std::cout << "!!slopM calculation error!!" << std::endl;
    }

    if ( (endPointXint_>startX) && (endPointYint_>startY))
    {
      for (int x=startX; x<=endPointXint_ && x<myGrid_.info.width; x++) 
      { 
        for (int y=startY; y<=endPointYint_ && y<myGrid_.info.height; y++) 
        { 
          FillGridFree(x, y);
        } 
      }
    }
    else if ((endPointXint_<startX) && (endPointYint_>startY))
    {
      for (int x=startX; x>endPointXint_ && x<myGrid_.info.width; x--) 
      { 
        for (int y=startY; y<endPointYint_ && y<myGrid_.info.height; y++) 
        { 
          FillGridFree(x, y);
        } 
      }   
    }
    else if ((endPointXint_>startX) && (endPointYint_<startY))
    {
      for (int x=startX; x<endPointXint_ && x<myGrid_.info.width; x++) 
      { 
        for (int y=startY; y>endPointYint_ && y<myGrid_.info.height; y--) 
        { 
          FillGridFree(x, y);
        } 
      }
    }
    else if( (endPointXint_<startX) && (endPointYint_<startY) )
    {
      for (int x=startX; x>endPointXint_ && x<myGrid_.info.width; x--) 
      { 
        for (int y=startY; y>endPointYint_ && y<myGrid_.info.height; y--) 
        { 
          FillGridFree(x, y); 
        } 
      }
    }
    if(laserRange<2.0)
    {
      FillGridOccupied();
    }
  }
  myGrid_.header.seq++;
  gridPub_.publish(myGrid_);
  //SetRobotFreeCells();
}


void OccupancyGridTest::OdomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
  robotPose_.x = odomMsg->pose.pose.position.x;
  robotPose_.y = odomMsg->pose.pose.position.y;
  //SetRobotFreeCells();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_test");
  OccupancyGridTest myObj(5, 0.01, 500, 500, 0.5);
  ros::spin();
  return 0;
}