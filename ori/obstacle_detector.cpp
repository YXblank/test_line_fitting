#include "../include/obstacle_detector.h"
#include <vector>
#include <ros/time.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <vector>

using namespace std;
using namespace cv;
using namespace obstacle_detector;
ros::Publisher center_point;

//vector<cv::Point> Feasible_region;          // 存储白点
/*
void Read_Image(cv::Mat& src)
{
	cv::Point pt;
	Mat dst;
	if(dst.data!=src.data)
	{
		src.copyTo(dst);
	}
	int width, height;
	width = src.cols-2;
	height = src.rows-2;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (dst.at<uchar>(i,j)==255)  
			{
				pt.x= i;
				pt.y= j;
				Feasible_region.push_back(pt);
			}
		}
	}
}
*/
ObstacleDetector::ObstacleDetector() : nh_(""), nh_local_("~") {
  updateParams();

  begin = ros::Time::now();

  if (p_use_scan_)
    scan_sub_ = nh_.subscribe("scan", 10, &ObstacleDetector::scanCallback, this);  //对于二维激光雷达，这里接收的是“scan”话题
  else if (p_use_pcl_)
    //pcl_sub_ = nh_.subscribe("pcl", 10, &ObstacleDetector::pclCallback, this);
        pcl_sub_ = nh_.subscribe("PointCloud2", 10, &ObstacleDetector::pclCallback, this);
  obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("obstacles", 5);

  ROS_INFO("Obstacle Detector [OK]");
  ros::spin();
}

//参数更新
void ObstacleDetector::updateParams() 
{
  nh_local_.param<std::string>("world_frame", p_world_frame_, "world");
  nh_local_.param<std::string>("scanner_frame", p_scanner_frame_, "scanner");

  nh_local_.param<bool>("use_scan", p_use_scan_, true);
  nh_local_.param<bool>("use_pcl", p_use_pcl_, false);
  nh_local_.param<bool>("transform_to_world", p_transform_to_world, true);
  nh_local_.param<bool>("use_split_and_merge", p_use_split_and_merge_, true);

  nh_local_.param<int>("min_group_points", p_min_group_points_, 3);

  nh_local_.param<double>("max_group_distance", p_max_group_distance_, 0.100);
  nh_local_.param<double>("distance_proportion", p_distance_proportion_, 0.006136);
  nh_local_.param<double>("max_split_distance", p_max_split_distance_, 0.100);
  nh_local_.param<double>("max_merge_separation", p_max_merge_separation_, 0.200);
  nh_local_.param<double>("max_merge_spread", p_max_merge_spread_, 0.070);
  nh_local_.param<double>("max_circle_radius", p_max_circle_radius_, 2.000);   //0.3
  nh_local_.param<double>("radius_enlargement", p_radius_enlargement_, 0.050);

  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 5.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_, 2.0);
  nh_local_.param<double>("min_x_range", p_min_x_range_, -2.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_, 2.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -2.0);
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {   //方法1：取出满足条件的激光数据
  initial_points_.clear();        //删除所有的原始激光数据，initial_points_是定义的用于存放激光数据的一个 vector容器

  double phi = scan->angle_min - scan->angle_increment;   //起始角度 - 角距离

  for (const float r : scan->ranges) {    //激光雷达的有效范围，遍历所有激光数据
    phi += scan->angle_increment;

    if (r >= scan->range_min && r <= scan->range_max && r <= p_max_scanner_range_)   //激光雷达的判断范围，要扩大其检测范围，需要在launch文件中修改这三个参数
      initial_points_.push_back(Point::fromPoolarCoords(r, phi));                    //把所有符合条件的激光数据放入到点容器中
  }

  processPoints();
}

void ObstacleDetector::pclCallback(const sensor_msgs::PointCloud::ConstPtr& pcl) {    //plc 方法2：取出满足条件的激光数据
  initial_points_.clear();

  for (const geometry_msgs::Point32& p : pcl->points)
    if (Point(p.x, p.y).lengthSquared() <= pow(p_max_scanner_range_, 2.0))
      initial_points_.push_back(Point(p.x, p.y));

  processPoints();
}

void ObstacleDetector::processPoints() {           //处理数据点
  segments_.clear();    // segment 存线  circles 存圆
  //circles_.clear();    //清空数据

  groupPointsAndDetectSegments();      //对于激光点组进行分组 并 检测直线
  mergeSegments();                     //合并 线段

  //detectCircles();  //检测圆
  //mergeCircles();   //合并圆

  if (p_transform_to_world)    //true,执行transformtoworld
    transformToWorld();

  publishObstacles();

  cout<<"------------------------------------------"<<endl;
}

void ObstacleDetector::groupPointsAndDetectSegments() 
{
  list<Point> point_set;

  for (const Point& point : initial_points_) 
  {   //此时已经满足条件的激光数据传入到initial_points_点容器中 （x，y）
    if (point_set.size() != 0) 
    {
      double r = point.length();    // 反求距离r
                                    //当前点和上一个点进行比较，计算距离的平方
      if ((point - point_set.back()).lengthSquared() > pow(p_max_group_distance_ + r * p_distance_proportion_, 2.0)) 
      {
        detectSegments(point_set);   //若两个点之间的距离大于规定距离，判断一条直线的依据               //r越大，对一组点之间的距离就可以越大
        point_set.clear();
      }
    }

    point_set.push_back(point);  //一个个点往里面推
  }

  detectSegments(point_set); // Check the last point set too!
}

void ObstacleDetector::detectSegments(list<Point>& point_set) {     //检查直线  point_set中存储的是一组数据点
  if (point_set.size() < p_min_group_points_)   //点的个数小于最低要求，不做处理
    return;

  Segment segment(Point(0.0, 0.0), Point(1.0, 0.0));
  if (p_use_split_and_merge_)                                //使用fifsegment函数
    segment = fitSegment(point_set);                         //也是返回两个点

  else                                                       //使用迭代端点拟合
    segment = Segment(point_set.front(), point_set.back());  //使用一组点计算出来的直线的 前后端点 来拟合直线  

  list<Point>::iterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  // Seek the point of division; omit first and last point of the set   寻求分裂点； 省略集合的第一个和最后一个点  第一个点和最后一个点已经被使用
  for (auto point_itr = ++point_set.begin(); point_itr != --point_set.end(); ++point_itr) 
  {
    if ((distance = segment.distanceTo(*point_itr)) >= max_distance) {    //此时distance等于 直线上拟合点和原始点的距离
      double r = (*point_itr).length();        //激光点云到机器人的距离

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {   // 如果距离偏差值大于容许误差，继续进行处理
        max_distance = distance;
        set_divider = point_itr;        //传入分割点 
      }
    }
  }

  if (max_distance > 0.0) { // Split the set    拆分集
  
    point_set.insert(set_divider, *set_divider);  // Clone the dividing point for each group  克隆每个组的分割点   在分割点处克隆插入一个点

    list<Point> subset1, subset2;
    subset1.splice(subset1.begin(), point_set, point_set.begin(), set_divider);    //list.splice 合并两个链表
    subset2.splice(subset2.begin(), point_set, set_divider, point_set.end());

    detectSegments(subset1);
    detectSegments(subset2);
  } 
  else 
  {  // Add the segment  添加线段       max_distance只可能 >=0,这里else的情况应该是 =0
    if (!p_use_split_and_merge_)       
      segment = fitSegment(point_set);

    if (segment.length() > 0.0) 
    {
      segments_.push_back(segment);
      segments_.back().point_set().assign(point_set.begin(), point_set.end());   //清除之前的内容，将第一个元素和最后一个元素赋值进去
    }
  }
}

void ObstacleDetector::mergeSegments() {          //合并两个直线
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    auto j = i;
    for (++j; j != segments_.end(); ++j) {       //相邻的两条直线进行比较
      if (compareAndMergeSegments(*i, *j)) {  // If merged - a new segment appeared at the end of the list   如果合并-列表末尾会出现一个新直线
        auto temp_ptr = i;
        i = segments_.insert(i, segments_.back()); // Copy new segment in place; i now points to new segment  //将后一个直线合并到前一个直线中
        segments_.pop_back();       // Remove the new segment from the back of the list   返回到最后一个元素中
        segments_.erase(temp_ptr);  // Remove the first merged segment    删除第一个合并线段
        segments_.erase(j);         // Remove the second merged segment   删除第二个合并线段
        if (i != segments_.begin())
          i--;                      // The for loop will increment i so let it point to new segment in next iteration for循环将递增i，因此让它在下一次迭代中指向新的段
        break;
      }
    }
  }
}

// 用于比较两个直线
bool ObstacleDetector::compareAndMergeSegments(Segment& s1, Segment& s2) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise   段必须逆时针提供
  if (s1.first_point().cross(s2.first_point()) < 0.0)
    return compareAndMergeSegments(s2, s1);

  if ((s1.last_point() - s2.first_point()).length() < p_max_merge_separation_) { //这里再次判断能否合并
    list<Point> merged_points;
    merged_points.insert(merged_points.begin(), s1.point_set().begin(), s1.point_set().end());
    merged_points.insert(merged_points.end(), s2.point_set().begin(), s2.point_set().end());      //将segment s1  S2 插入到merged_points 中

    Segment s = fitSegment(merged_points);  //拟合直线

    if (s.distanceTo(s1.first_point()) < p_max_merge_spread_ &&
        s.distanceTo(s1.last_point())  < p_max_merge_spread_ &&
        s.distanceTo(s2.first_point()) < p_max_merge_spread_ &&
        s.distanceTo(s2.last_point())  < p_max_merge_spread_) {   //如果两直线的起始点到合并之后直线的距离小于域值 则用合并之后的直线替代
      segments_.push_back(s);
      segments_.back().point_set().assign(merged_points.begin(), merged_points.end());
      return true;
    }
  }

  return false;
}
/*
void ObstacleDetector::detectCircles() {    //找出符合的圆
  for (const Segment& s : segments_) {
    Circle c(s);
    c.setRadius(c.radius() + p_radius_enlargement_);    //相当于障碍物膨胀，为了包裹整个障碍物

    if (c.radius() < p_max_circle_radius_)      // 判断障碍物的半径是否在规定之内
      circles_.push_back(c);
  }
}
*/
/*
void ObstacleDetector::mergeCircles() {     //合并圆圈
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    auto j = i;
    for (++j; j != circles_.end(); ++j) {
      if (compareAndMergeCircles(*i, *j)) {      // If merged - a new circle appeared at the end of the list  如果合并-列表末尾出现一个新的圆圈
        auto temp_ptr = i;                       //比较相邻的两个圈
        i = circles_.insert(i, circles_.back()); // i now points to new circle
        circles_.pop_back();
        circles_.erase(temp_ptr);
        circles_.erase(j);
        if (i != circles_.begin())
          --i;
        break;
      }
    }
  }
}
*/
/*
bool ObstacleDetector::compareAndMergeCircles(Circle& c1, Circle& c2) {
  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c1.radius() + (c2.center() - c1.center()).length() <= c2.radius()) {
    circles_.push_back(c2);
    cout<<"111"<<endl;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c2.radius() + (c2.center() - c1.center()).length() <= c1.radius()) {
    circles_.push_back(c1);
    cout<<"222"<<endl;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius() + c2.radius() >= (c2.center() - c1.center()).length()) {
    Segment s(c1.center(), c2.center());
    Circle c(s);
    c.setRadius(c.radius() + max(c1.radius(), c2.radius()));    
    cout<<"合并圆"<<c.radius()<<endl;
    
        //if (c.radius() < p_max_circle_radius_) {
      circles_.push_back(c);
      return true;
    //}
  }



  
  return false;
}
*/
void ObstacleDetector::transformToWorld() {         //转化到全局坐标
  geometry_msgs::PointStamped point_l;  // Point in local (scan) coordinate frame
  geometry_msgs::PointStamped point_w;  // Point in global (world) coordinate frame

  point_l.header.stamp = ros::Time::now();
  point_l.header.frame_id = p_scanner_frame_;

  point_w.header.stamp = ros::Time::now();
  point_w.header.frame_id = p_world_frame_;

  try {
    tf_listener_.waitForTransform(p_world_frame_, p_scanner_frame_, ros::Time::now(), ros::Duration(3.0));

    for (auto it = circles_.begin(); it != circles_.end(); ++it) {
      if (it->center().x < p_max_x_range_ && it->center().x > p_min_x_range_ &&
          it->center().y < p_max_y_range_ && it->center().y > p_min_y_range_)
      {
        point_l.point.x = it->center().x;
        point_l.point.y = it->center().y;
        tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
        it->setCenter(point_w.point.x, point_w.point.y);
      }
      /*else {
        it = circles_.erase(it);
        --it;
      }*/
    }

    for (Segment& s : segments_) {
      point_l.point.x = s.first_point().x;
      point_l.point.y = s.first_point().y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      s.setFirstPoint(point_w.point.x, point_w.point.y);

      point_l.point.x = s.last_point().x;
      point_l.point.y = s.last_point().y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      s.setLastPoint(point_w.point.x, point_w.point.y);
    }
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void ObstacleDetector::publishObstacles() {
  Obstacles obstacles;
  obstacles.header.stamp = ros::Time::now();   //得到ros::Time实例化的当前时间

  if (p_transform_to_world)
    obstacles.header.frame_id = p_world_frame_;
  else
    obstacles.header.frame_id = p_scanner_frame_;

  for (const Segment& s : segments_) {
    obstacle_detector::SegmentObstacle segment;

    segment.first_point.x = s.first_point().x;
    segment.first_point.y = s.first_point().y;
    segment.last_point.x = s.last_point().x;
    segment.last_point.y = s.last_point().y;

    obstacles.segments.push_back(segment);  // 发布存储的直线
    obstacles_pub_.publish(obstacles);
  }

  struct pointandtime  // 定义的一个结构体，用于传输动态障碍物的坐标及时间信息
  {
    double x;
    double y;
    double t;
  };
  vector<pointandtime> ct;


//按顺序遍历所有的圆

  for (const Circle& c : circles_) {
    obstacle_detector::CircleObstacle circle;

    circle.center.x = c.center().x;
    circle.center.y = c.center().y;
    circle.radius = c.radius(); 
  
    // int center_x_image = 300-20*c.center().y;  // 构建地图的分辨率是384*384 栅格的分辨率是0.05
    // int center_y_image = 276+20*c.center().x;

    //int center_x_image = 332-20*c.center().y;  //空旷的环境
    //int center_y_image = 468+20*c.center().x;  //空旷的环境

    //int center_x_image = 300-20*c.center().y;    //20220228
    //int center_y_image = 308+20*c.center().x;    //20220228

    //int center_x_image = 300-20*c.center().y;    //dongtaishiyan
    //int center_y_image = 276+20*c.center().x;    //

    int center_x_image = 332-20*c.center().y;    //
    int center_y_image = 308+20*c.center().x;    //
    

    cv::Point pt_center;
    pt_center.x= center_x_image;
    pt_center.y= center_y_image;
  
//要排除非原始障碍物区域
    /*
    if((-0.44<c.center().x && c.center().x<1.77 && -1.29<c.center().y && c.center().y<2.44) ||
        ( 2.62<c.center().x && c.center().x<5.14 && -1.29<c.center().y && c.center().y<2.44) ||
        (-0.44<c.center().x && c.center().x<5.14 && -1.29<c.center().y && c.center().y<0.73) 
      )
    */
    /*
    if(find(Feasible_region.begin(), Feasible_region.end(), pt_center)!= Feasible_region.end())
    {
      //std::cout<<"-----找到动态障碍物-----"<<std::endl;
      obstacles.circles.push_back(circle);
      cout<<c.center().x<<","<<c.center().y<<endl;
      cout<<"半径为："<<c.radius()<<endl;
      
      center_point = nh_.advertise<std_msgs::Float32MultiArray>("center", 1000);    //初始化一个发布者，用于发布动态障碍物的中心点、以及此刻时间
      ros::Rate loop_rate(10);

      ros::Time end = ros::Time::now();
      double secs =(end-begin).toSec();
      //float secs =ros::Time::now().toSec();                        //获取此刻时间

      float ct_point[4]= {c.center().x, c.center().y, secs,c.radius()};       //将中心点、时间存入数组
      vector<float> array1(ct_point, ct_point+4);
      cout<<"此时时刻为:"<<secs<<endl;
      std_msgs::Float32MultiArray msg;                              //msg是一个容器类型的数据，需要通过容器进行转换
      msg.data = array1;
      center_point.publish(msg);




    } 
  }

  obstacles_pub_.publish(obstacles);
}
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");     //ros程序调用的第一个函数，对ros程序进行初始化

  //Mat binImage = cv::imread("/home/lj/complex_dynamic_environment/src/mbot_navigation/maps/kongkuang.pgm",0);
  //Mat binImage = cv::imread("/home/lj/complex_dynamic_environment/src/mbot_navigation/maps/20220113dongtaixiao.pgm",0);
  //Mat binImage = cv::imread("/home/lj/complex_dynamic_environment/src/mbot_navigation/maps/20220228.png",0);
  
  //Mat binImage = cv::imread("/home/lj/complex_dynamic_environment/src/mbot_navigation/maps/20220113dongtai.pgm",0);	
 // Mat binImage = cv::imread("/home/lj/complex_dynamic_environment/src/mbot_navigation/maps/20200310xiaoxiu.pgm",0);	
 // threshold(binImage, binImage, 205, 255, CV_THRESH_BINARY) ;   // 二值化 binImage变成二值图

  //Mat element_1 = getStructuringElement(MORPH_RECT, Size(8, 8));  // 腐蚀大小可以自己根据实际情况定义8-10
 // erode(binImage, binImage, element_1); //腐蚀操作
	//Read_Image(binImage);

  ObstacleDetector od;
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/PointCloud2", 1, cloud_cb);
 // updateParams();
  ros::spin();

  return 0;
}
