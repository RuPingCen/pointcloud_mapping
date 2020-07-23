/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H
 
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
#include <condition_variable>
   
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
 
 #include <opencv2/opencv.hpp>
 #include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//#include "orbslam2_ros/RGBDPose.h" //自定义消息类型


 using namespace std;
namespace Mapping
{

class PointCloudMapper
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
	 bool mbuseExact, mbuseCompressed;
	 size_t queueSize;
	 
	 

	PointCloudMapper();
	~PointCloudMapper();

	// 插入一个keyframe，会更新一次地图
	void insertKeyFrame(cv::Mat& color, cv::Mat& depth,Eigen::Isometry3d& T);
	void viewer();
	void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap);
	void reset();
	void shutdown();
	void callback(const sensor_msgs::Image::ConstPtr msgRGB,
				  const sensor_msgs::Image::ConstPtr msgD,  const geometry_msgs::PoseStamped::ConstPtr tcw);
	void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr msgPointCloud,
								const geometry_msgs::PoseStamped::ConstPtr tcw );
protected:
    unsigned int index=0;
    float mresolution = 0.04;
	float mDepthMapFactor=1000.0;
	float mcx=0,mcy=0,mfx=0,mfy=0;
    pcl::VoxelGrid<PointT>  voxel; //点云显示精度
    //float mDepthMapFactor =1; //深度图尺度因子
     size_t  lastKeyframeSize =0; // 
     size_t mGlobalPointCloudID=0; //点云ID
     size_t  mLastGlobalPointCloudID=0;
	 
	  // data to generate point clouds
    vector<cv::Mat>         colorImgs,depthImgs;   //image buffer
    cv::Mat   depthImg,colorImg,mpose;
	 vector<PointCloud>   mvGlobalPointClouds; //存储关键帧对应的点云序列
	 vector<Eigen::Isometry3d>   mvGlobalPointCloudsPose;
	 
	 PointCloud::Ptr globalMap,localMap; 
	 
	// vector<cv::Mat>      mvLocalColorImgs,mvLocalDepthImgs; //关键帧之间的图像帧 局部点云的图像帧
     //vector<PointCloud::Ptr>   mvLocalPointClouds; //两个关键帧之间对应的点云
    //vector<cv::Mat>   mvLocalPointCloudsPose; //这些普通帧的位姿
	//unsigned long int  mReFerenceGLPID=0;   //记忆更新关键帧的ID
	 
    
    bool    shutDownFlag    =false; // 程序退出标志位
    mutex shutDownMutex;  
	 
    bool    mbKeyFrameUpdate    =false;        //有新的关键帧插入
    mutex   keyframeMutex;
    mutex keyFrameUpdateMutex;
	mutex deletekeyframeMutex;
 
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> ExactSyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> ApproximateSyncPolicy;
	//typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy_2;
	//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproximateSyncPolicy_2;


	std::string topic_sub;  //ROS varible
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	ros::Subscriber sub ;;
	ros::Publisher pub_global_pointcloud,pub_local_pointcloud;
	image_transport::ImageTransport it;
	image_transport::SubscriberFilter *subImageColor, *subImageDepth;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *tcw_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_sub;

	message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
	message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
	//message_filters::Synchronizer<ExactSyncPolicy_2> *syncExact_2 ;
	//message_filters::Synchronizer<ApproximateSyncPolicy_2> *syncApproximate_2 ;
	
	
	
	
   PointCloud::Ptr generatePointCloud( cv::Mat& color, cv::Mat& depth,Eigen::Isometry3d &T);
   
   void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,std::stringstream& compressedData);
   void depressPointCloud(std::stringstream& compressedData,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudOut);
   Eigen::Matrix4f cvMat2Eigen(const cv::Mat &cvT);
   void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
   
};
}
#endif // POINTCLOUDMAPPING_H
