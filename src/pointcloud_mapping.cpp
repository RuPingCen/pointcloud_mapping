/**
 * @function 接受RGB图像 Depth深度图像  相机位姿Tcw
 * 
 * 
 * @param topicColor
 * @param topicDepth
 * @param topicTcw
 * @param cameraParamFile  
 * 修改于高翔发布的点云构建线程
 * cenruping@vip.qq.com
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <boost/concept_check.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>



  
 #include "PointCloudMapper.h"

using namespace std;
 
 
 
int main(int argc, char **argv)
{
  
    std::string cameraParamFile;
    
    ros::init(argc, argv, "pointcloud_mapping", ros::init_options::AnonymousName);

    if(!ros::ok())
    {
	     cout<<"ros init error..."<<endl;
	    return 0;
    }
   ros::start();
   
	Mapping::PointCloudMapper mapper;
	mapper.viewer();
	
	cout<<"ros shutdown ..."<<endl;
	ros::waitForShutdown();
	ros::shutdown();

    return 0;
}