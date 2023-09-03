
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace obstacle_detector
{

class ScansMerger
{
public:
  ScansMerger();

private:
  void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void publishPCL();
  void updateParams();

  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber front_scan_sub_;
  ros::Subscriber rear_scan_sub_;
  ros::Publisher  pcl_pub_;

  sensor_msgs::PointCloud pcl_msg_;

  bool first_scan_received_;
  bool second_scan_received_;
  int unreceived_front_scans_;
  int unreceived_rear_scans_;

  // Parameters
  std::string p_pcl_frame_;          // TF frame name for the pcl message
  bool p_omit_overlapping_scans_;   // Omit the points which project onto area of the other scanner
  double p_scanners_separation_;    // Distance between scanner centers
  int p_max_unreceived_scans_;      // Maximum allowable unreceived scans to start publishing one scan
};

} // namespace obstacle_detector
