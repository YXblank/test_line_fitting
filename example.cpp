#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>     
#include<cmath>
#include <time.h>

using namespace std;

class fitLineRansac
{
  
public:
  vector<vector<float> > ransac_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist, int iterate)
  {
    /*** 
     *		dist: 点到直线距离小于dist 即当前点在直线上
     *      iterate: 迭代次数
    ***/
    int allPts = cloud->points.size();
    vector<int> cur_ptsIdx(allPts);			//提取当前直线后，不在当前直线上的其他点索引
    for(int i=0; i<allPts; i++)
      cur_ptsIdx[i] = i;

    int r=0;
    vector<int> line_ptsIdx, all_ptsIdx(allPts);
    vector<vector<float> > lines;			//所有直线参数 [0]: k, [1]: b
    vector<float> cur_line(2);
    Eigen::Vector3f line_model, best_lineModel;
    while (1)
    { 
      int line_pts=0, tmp_pts;
      if(r >= 2) iterate = iterate / 4;
      if(cur_ptsIdx.size()<10 && cur_ptsIdx.size()>3) iterate = 3;
      for(int i=0; i<iterate; i++){
        line_model = leastSquare(cloud, cur_ptsIdx, dist);
        tmp_pts = line_model[2] / 1;
        if(tmp_pts > line_pts){
          line_pts = tmp_pts;
          best_lineModel = line_model;
          line_ptsIdx = tmp_ptsIdx;
        }
        tmp_ptsIdx.clear();
      }
      
      cur_line[0] = best_lineModel[0]; cur_line[1] = best_lineModel[1];
      lines.push_back(cur_line);
      cout<<"第　"<<r++<<"　次循环,直线参数:  "<<best_lineModel<<endl;
    //   cout<<"所有点的个数:  "<<cur_ptsIdx.size()<<endl;
    //   cout<<"当前直线上的点数："<<line_ptsIdx.size()<<endl;
	
	//得到剩余点的索引
    for(int i=0; i<line_ptsIdx.size(); i++)
      all_ptsIdx[line_ptsIdx[i]] = 1;
    cur_ptsIdx.clear();
    for(int j=0; j<allPts; j++)
      if(!all_ptsIdx[j]) cur_ptsIdx.push_back(j);
      
      if(cur_ptsIdx.size() < 5){
        cout<<"点数剩余过少......."<<endl;
        break;  
      }
    } 
    view(cloud, lines);
    return lines;
  }

private:
  vector<int> tmp_ptsIdx;			//当前直线上的点数
  Eigen::Vector3f leastSquare(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> pIdx, float dist)
  {
    //求解给定若干点的直线方程
    float a=0, B=0, c=0, d=0;          //a: x之和　b: y之和  c: x平方和  d: x*y之和  e: 样本数量
    int s = pIdx.size();
    vector<int> cur_ptsIdx = rangedRand(0, s, 4);					//４：每次选４点用最小二乘拟合直线
    int e = cur_ptsIdx.size();
    for(int i=0; i<e; i++){
      a += cloud->points[pIdx[cur_ptsIdx[i]]].x;
      B += cloud->points[pIdx[cur_ptsIdx[i]]].y;
      c += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].x;
      d += cloud->points[pIdx[cur_ptsIdx[i]]].x * cloud->points[pIdx[cur_ptsIdx[i]]].y;
    }
    float k, b;
    float tmp = e * c - a * a;
    if(abs(tmp) > 0.0005){
      b = (c*B - a*d) / tmp;
      k = (e*d - a*B) / tmp;
    }
    else{
      k=1;b=0;
    }
    
    //求每一个点到直线的距离，小于dist, 即在直线上
    int line_pnum=0;
    for(int i=0; i<s; i++){
      float d, numerator, denominator;             //分子分母        点到直线的距离　d = |kx - y + b| / sqrt(k^2 + 1)
      numerator = abs(k*cloud->points[pIdx[i]].x - cloud->points[pIdx[i]].y + b);
      denominator = sqrt(k*k + 1);
      d = numerator / denominator;
      if(d < dist){
        line_pnum++;
        tmp_ptsIdx.push_back(pIdx[i]);
      }
    }
    Eigen::Vector3f line_model;              
    line_model[0] = k;line_model[1] = b; line_model[2] = line_pnum;
    return line_model;
  }

  vector<int> rangedRand( int range_begin, int range_size, int n )
  {
    int i;vector<int> indices;
    // srand((unsigned)time(NULL));           //生成随机数种子
    for ( i = 0; i < n; i++ )
    {
      int u = rand()%range_size + range_begin; //生成[range_begin, range_begin+range_size]内的随机数
      // cout<<i<<": "<<u<<endl;
      indices.push_back(u);
    }
    return indices;
  }

  void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<float> > lines)
  { 
    cout<<"共找到   "<<lines.size()<<"  条直线!!!!!!!!"<<endl;
    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.setBackgroundColor(0.5, 0.5, 0.5, 0);
    viewer.addPointCloud(cloud, "pc");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pc"); 
    for(int i=0; i<lines.size(); i++){
        pcl::PointXYZ p1(200, 200*lines[i][0]+lines[i][1], 0);
        pcl::PointXYZ p2(-500, -500*lines[i][0]+lines[i][1], 0);
        cout<<"直线     "<<i+1<<"   上两点坐标： "<<p1<<",  "<<p2<<endl;
        viewer.addLine(p1, p2, 240-40*i, 0, 0, "line"+boost::to_string(i), 0);
    }
    while (!viewer.wasStopped()) {
    	viewer.spinOnce();
    }
  }

};
void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{ 
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i = 0;i < cloud_msg -> width*cloud_msg -> height;i++)
  {
    pcl::PointXYZ p;
    std::memcpy(&p.x,&cloud_msg -> data[32*i],4);
    std::memcpy(&p.y,&cloud_msg -> data[32*i+4],4);
    std::memcpy(&p.z,&cloud_msg -> data[32*i+8],4);
    cloud->points.push_back(p);
  }
  fitLineRansac ransac;
  ransac.ransac_line(cloud, 7, 50);
  cout<<"done!"<<endl;
}
int main(int argc, char**argv) {
  // 加载点云模型
 
 ros::init(argc, argv, "my_pcl");
 ros::NodeHandle nh;
 ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/output", 1, callback);
 

 
 ros::spin();
 return 0;
}


