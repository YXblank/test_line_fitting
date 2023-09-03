
#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <obstacle_detector/Obstacles.h>

namespace obstacle_detector
{

class ObstacleVisualizer
{
public:
  ObstacleVisualizer();

private:
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& obstacles);

  // ROS handlers
  ros::NodeHandle nh_;
  ros::Subscriber obstacles_sub_;
  ros::Publisher  markers_pub_;
};

} // namespace obstacle_detector
