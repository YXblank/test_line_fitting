
#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <obstacle_detector/Obstacles.h>

#include "../include/point.h"
#include "../include/segment.h"
#include "../include/circle.h"
#include "../include/figure_fitting.h"

namespace obstacle_detector      //指定命名空间
{

class ObstacleDetector
{
public:
  ObstacleDetector();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl);
  void updateParams();

  void processPoints();
  void groupPointsAndDetectSegments();
  void detectSegments(std::list<Point>& point_set);
  void mergeSegments();
  bool compareAndMergeSegments(Segment& s1, Segment& s2);

  void detectCircles();
  void mergeCircles();
  bool compareAndMergeCircles(Circle& c1, Circle& c2);

  void publishObstacles();
  void transformToWorld();

  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber scan_sub_;
  ros::Subscriber pcl_sub_;
  ros::Publisher  obstacles_pub_;

  tf::TransformListener tf_listener_;

  // Detector variables  探测器变量
  std::vector<Point> initial_points_;    //存储所有距离范围之内的数据点
  std::list<Segment> segments_;          //存储符合直线的激光数据  list 将元素按顺序存储在链表中。与vector相比，它容许快速的插入和删除，但是随机访问很慢
  std::list<Circle>  circles_;           //存储符合圆状障碍物的激光数据

  // Parameters
  std::string p_world_frame_;     // Name of the world coordinate frame
  std::string p_scanner_frame_;   // Name of the scanner coordinate frame

  bool p_use_scan_;               // Use data from scans
  bool p_use_pcl_;                // Use data from point clouds
  bool p_use_split_and_merge_;    // If false, iterative closest point is used instead of split and merge
  bool p_transform_to_world;      // Transform obstacles to world coordinate frame

  int    p_min_group_points_;     // Miminal number of points in a set to process it further  一组点的最少个数
  double p_distance_proportion_;  // Proportion of allowable distances to the range of a point (based on scan angle increment)
  double p_max_group_distance_;   // Maximal allowable distance between two points in a set   集合中两点之间的最大允许距离

  double p_max_split_distance_;   // Maximal allowable distance between a point and a leading line in the splitting process
  double p_max_merge_separation_; // Maximal allowable distance between two segments when merging
  double p_max_merge_spread_;     // Maximal allowable spread of initial segments around merged segment
  double p_max_circle_radius_;    // Maximal allowable radius of a detected circle
  double p_radius_enlargement_;   // Additional boundary for the obstacle

  double p_max_scanner_range_;    // Restrictions on laser scanner   限制激光距离
  double p_max_x_range_;          // Restrictions on world coordinates  限制全局坐标
  double p_min_x_range_;
  double p_max_y_range_;
  double p_min_y_range_;

  ros::Time begin;
};

} // namespace obstacle_detector
