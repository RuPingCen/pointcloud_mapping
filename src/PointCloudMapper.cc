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

#include <chrono>
#include <ctime>
#include <climits>
#include <Eigen/Core>
#include <Eigen/Geometry>  // Eigen 几何模块
#include <opencv2/highgui/highgui.hpp>
#include "PointCloudMapper.h"


#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h> 

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */  
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */ 
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
 ////    	cout<<RED<<"compute the intensity of features ..."<<WHITE<<endl;
using namespace std;

namespace Mapping
{
	
/*
 * 
 * @ 设置点云分辨率
 */
PointCloudMapper::PointCloudMapper()
									:nh("~"), spinner(0), it(nh)
{
	float fx_,  fy_,  cx_,  cy_,  resolution_,depthfactor_;
	int queueSize_;
	bool mbuseExact_;
	
	mbuseCompressed = false;
	lastKeyframeSize=0;
	mGlobalPointCloudID=0; //点云ID
	mLastGlobalPointCloudID=0;
	queueSize =10;
 
	std::string topicColor ,topicDepth,topicTcw,topicPointCloud;
 
	
	if(ros::param::get("~topicColor" ,topicColor))
	;
	else
	topicColor = "/RGBD/RGB/Image";
	if(ros::param::get("~topicDepth" ,topicDepth))
	;
	else
	topicDepth = "/RGBD/Depth/Image";
	if(ros::param::get("~topicTcw" ,topicTcw))
	;
	else
	topicTcw = "/RGBD/CameraPose";
 

	nh.param<float>("fx", fx_, 515.2888);
	nh.param<float>("fy", fy_, 517.6610);
	nh.param<float>("cx", cx_, 317.9098);
	nh.param<float>("cy", cy_, 241.5734);
	nh.param<float>("resolution", resolution_, 0.05);
	nh.param<float>("depthfactor", depthfactor_, 1000.0);
 
	nh.param<int>("queueSize", queueSize_, 10);
	nh.param<bool>("buseExact", mbuseExact_, true);
 
	mbuseExact = mbuseExact_;  //
	queueSize=queueSize_;
	mcx = cx_;
	mcy = cy_;
	mfx = fx_;
	mfy = fy_;
	mresolution = resolution_;
	mDepthMapFactor = depthfactor_;
	
	image_transport::TransportHints hints(mbuseCompressed ? "compressed" : "raw");
	subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
	subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
	tcw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, topicTcw, queueSize);
 
	cout<<"topicColor: "<<topicColor<<endl;
	cout<<"topicDepth: "<<topicDepth<<endl;
	cout<<"topicTcw: "<<topicTcw<<endl;
 

	cout<<"fx: "<<mfx<<endl;
	cout<<"fy: "<<mfy<<endl;
	cout<<"cx: "<<mcx<<endl;
	cout<<"cy: "<<mcy<<endl;
	cout<<"resolution: "<<mresolution<<endl;
	cout<<"DepthMapFactor: "<<mDepthMapFactor<<endl;
 	cout<<"queueSize: "<<queueSize<<endl;
	cout<<"mbuseExact: "<<mbuseExact<<endl;
	
    //接受RGB DepTh 位姿数据
	if(mbuseExact)
	{
		syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *tcw_sub);
		syncExact->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3));
	}
	else
	{
		syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *tcw_sub);
		syncApproximate->registerCallback(boost::bind(&PointCloudMapper::callback, this, _1, _2, _3));
	}
  
	voxel.setLeafSize( mresolution, mresolution, mresolution);
	globalMap = boost::make_shared< PointCloud >( );
	localMap = boost::make_shared< PointCloud >( );

	pub_global_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("Global/PointCloudOutput", 1); 
	pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("Local/PointCloudOutput", 10); 
     
 //viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}
PointCloudMapper::~PointCloudMapper()
{
	//viewerThread->join();
	shutdown();
}


// 由外部函数调用，每生成一个关键帧调用一次该函数
void PointCloudMapper::insertKeyFrame( cv::Mat& color, cv::Mat& depth,Eigen::Isometry3d& T)
{
	unique_lock<mutex> lck(keyframeMutex);

	mvGlobalPointCloudsPose.push_back(T);    // 每个点云的pose
	colorImgs.push_back( color.clone() );
	depthImgs.push_back( depth.clone() );

	// mvLocalPointCloudsPose.clear();     
	//mvLocalColorImgs.clear();     
	//mvLocalDepthImgs.clear();     

	//mLastGlobalPointCloudID=mGlobalPointCloudID;
	mGlobalPointCloudID++;
	mbKeyFrameUpdate =true;
 
	cout<<GREEN<<"receive a keyframe, id = "<<mGlobalPointCloudID<<WHITE<<endl;
}
 
/**
 * @function 更加关键帧生成点云、并对点云进行滤波处理
 * 备注：点云生成函数在　台式机上调用时间在0.1ｓ 左右
 */
pcl::PointCloud< PointCloudMapper::PointT >::Ptr PointCloudMapper::generatePointCloud( cv::Mat& color, cv::Mat& depth,Eigen::Isometry3d &T)
{
     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    PointCloud::Ptr tmp( new PointCloud() );// point cloud is null ptr
    
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n]/mDepthMapFactor;
            if (d < 0.01 || d>10)
			{
// 				if(d < 0.01)
// 				cout<<".";
// 				else if(d>10)
// 				cout<<"+";
				continue;
			}
            PointT p;
            p.z = d;
            p.x = ( n - mcx) * p.z / mfx;
            p.y = ( m -mcy) * p.z / mfy;  
            
            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
         PointCloud::Ptr cloud_voxel_tem(new PointCloud);
		tmp->is_dense = false;
		voxel.setInputCloud( tmp );
		voxel.setLeafSize( mresolution, mresolution, mresolution);
		voxel.filter( *cloud_voxel_tem );
	  
	    PointCloud::Ptr cloud1(new PointCloud);
		pcl::transformPointCloud( *cloud_voxel_tem, *cloud1, T.matrix());
 
   	  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	   
	  cout<<GREEN<<"generate point cloud from  kf-ID:"<<mLastGlobalPointCloudID<<", size="<<cloud1->points.size()<<" cost time: "<<time_used.count()*1000<<" ms ."<<WHITE<<endl;
	  mLastGlobalPointCloudID++;
	 return cloud1;
}
pcl::PointCloud< PointCloudMapper::PointT >::Ptr PointCloudMapper::generatePointCloud( cv::Mat& color, cv::Mat& depth)
{
     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    PointCloud::Ptr tmp( new PointCloud() );// point cloud is null ptr
    
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n]/1000;
            if (d < 0.01 || d>10)
				continue;
                
            PointT p;
            p.z = d;
            p.x = ( n - mcx) * p.z / mfx;
            p.y = ( m -mcy) * p.z / mfy;  
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
     
     cout<<"generate point cloud from  kf-ID:"<<mGlobalPointCloudID<<", size="<<tmp->points.size();
	 
     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
     cout<<"generate point cloud  cost time: "<<time_used.count()*1000<<" ms ."<<endl;
     
    return tmp;
}

/*
 * 
 * 原来版本的显示函数
 * 由于随着尺寸的增加以后,显示函数会异常退出
 */
  void PointCloudMapper::viewer()
{
	pcl::visualization::CloudViewer pcl_viewer("viewer");
	size_t N=0,i=0;
	bool FUpdate=false;
	bool KFUpdate=false;
	bool LoopCloserUpdate=false;

 
	ros::AsyncSpinner spinner(2); // Use 1threads
	spinner.start();
 
	while(ros::ok())
    {
//         {
//             unique_lock<mutex> lck_shutdown( shutDownMutex );
//             if (shutDownFlag)
//             {
//                 break;
//             }
//         }
    
        // keyframe is updated 
		KFUpdate=false;
	   {
			unique_lock<mutex> lck( keyframeMutex );
			N =mvGlobalPointCloudsPose.size();
			KFUpdate=mbKeyFrameUpdate;
			mbKeyFrameUpdate=false;
	   }
	  if(KFUpdate)
	  {
		  for (  i=lastKeyframeSize; i<N&& i<(lastKeyframeSize+5); i++ )  
		  {
			  if((mvGlobalPointCloudsPose.size() != colorImgs.size() )|| (mvGlobalPointCloudsPose.size()!= depthImgs.size() ) || (depthImgs.size() != colorImgs.size() ))
			  {
			   cout<<" depthImgs.size != colorImgs.size()  "<<endl;
			   //cout<<"depthImgs.size(): "<<depthImgs.size()<<endl;
			   //cout<<"colorImgs.size(): "<<colorImgs.size()<<endl;
			   continue;
			  }
			  PointCloud::Ptr tem_cloud1( new PointCloud() );
			  cout<<"i: "<<i<<"  mvPosePointClouds.size(): "<<mvGlobalPointCloudsPose.size()<<endl;
			  //生成一幅点云大约在０．１s左右 点云数据 
			  tem_cloud1= generatePointCloud(colorImgs[i], depthImgs[i],mvGlobalPointCloudsPose[i]); 
 
			  if(tem_cloud1->empty())
			  continue;

			  *globalMap += *tem_cloud1;

			  sensor_msgs::PointCloud2 local;  
			  pcl::toROSMsg(*tem_cloud1,local);// 转换成ROS下的数据类型 最终通过topic发布
			  local.header.stamp=ros::Time::now();
			  local.header.frame_id  ="world";
			  pub_local_pointcloud.publish(local);
		  }
		   {
			int buff_length=150;
			if(i > (buff_length+5))
			{
				   	unique_lock<mutex> lck( deletekeyframeMutex );
					mvGlobalPointCloudsPose.erase(mvGlobalPointCloudsPose.begin(),mvGlobalPointCloudsPose.begin()+buff_length/2);
					depthImgs.erase(depthImgs.begin(),depthImgs.begin()+buff_length);
					colorImgs.erase(colorImgs.begin(),colorImgs.begin()+buff_length);
					i=i-buff_length;
					cout<<RED<<"delete keyframe ...."<<WHITE<<endl;
			}
		}
		
		 lastKeyframeSize = i;
		 sensor_msgs::PointCloud2 output;  
		 pcl::toROSMsg(*globalMap,output);// 转换成ROS下的数据类型 最终通过topic发布
		 output.header.stamp=ros::Time::now();
		 output.header.frame_id  ="world";
		 pub_global_pointcloud.publish(output);
		 	 
         pcl_viewer.showCloud( globalMap );
		 cout<<"show global map, size="<<globalMap->points.size()<<endl;
		 //if((lastKeyframeSize%3 == 0 )&& (globalMap->points.size()>0))
		 if((N==i )&& (globalMap->points.size()>0))
			shutdown();
	   }	
	  
		 //ros::spinOnce(); 
	}
	//spinner.stop();

}
 
   
Eigen::Matrix4f  PointCloudMapper::cvMat2Eigen(const cv::Mat &cvT)
{
    Eigen::Matrix<float,4,4> T;
    T<< cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),cvT.at<float>(0,3),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),cvT.at<float>(1,3),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2),cvT.at<float>(2,3),
         0,0,0,1;
 
    return  T;
}
// Eigen::Isometry3d toSE3Quat(const cv::Mat &cvT)
// {
//     Eigen::Matrix<double,3,3> R;
//     R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
//          cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
//          cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);
//     Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
// 
//   //  return g2o::SE3Quat(R,t);
// }
  /**
 * @ 将深度图像映射成彩色图
 * 
 */
void PointCloudMapper::dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
{
cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
const uint32_t maxInt = 255;

//  #pragma omp parallel for
for(int r = 0; r < in.rows; ++r)
{
	const uint16_t *itI = in.ptr<uint16_t>(r);
	uint8_t *itO = tmp.ptr<uint8_t>(r);

	for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
	{
	*itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
	}
}

cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
}

void PointCloudMapper::callback(const sensor_msgs::Image::ConstPtr msgRGB,
								const sensor_msgs::Image::ConstPtr msgD,	const geometry_msgs::PoseStamped::ConstPtr tcw)
 {
       
		cv::Mat color, depth,depthDisp;
		geometry_msgs::PoseStamped Tcw=*tcw; 
		cv_bridge::CvImageConstPtr pCvImage;

		pCvImage = cv_bridge::toCvShare(msgRGB, "rgb8");
		pCvImage->image.copyTo(color);
		pCvImage = cv_bridge::toCvShare(msgD, msgD->encoding); //imageDepth->encoding
		pCvImage->image.copyTo(depth);
		// IR image input
		if(color.type() == CV_16U)
		{
			cv::Mat tmp;
			color.convertTo(tmp, CV_8U, 0.02);
			cv::cvtColor(tmp, color, CV_GRAY2BGR);
		}
		// 	if(depth.type() != CV_16U)
		// 	{
		// 		// cv::Mat tmp;
		// 		depth.convertTo(depth, CV_16U);
		// 		// cv::cvtColor(tmp, color, CV_GRAY2BGR);
		// 	}
	 
      if(depth.type()!=CV_32F)
        depth.convertTo(depth,CV_32F);
		
		Eigen::Quaterniond q =Eigen::Quaterniond(Tcw.pose.orientation.w,Tcw.pose.orientation.x,Tcw.pose.orientation.y,Tcw.pose.orientation.z) ;
		Eigen::AngleAxisd V6(q);
		//  V6.fromRotationMatrix<double,3,3>(q.toRotationMatrix());
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
		T.rotate(V6);  // 旋转部分赋值
		T(0,3)= Tcw.pose.position.x;
		T(1,3)= Tcw.pose.position.y;
		T(2,3)= Tcw.pose.position.z;
		// 已测试接受到的数据没有问题
		//cout<< GREEN<<"T:"<<T.matrix()<<WHITE<<endl;
		insertKeyFrame( color, depth,T);

// 	dispDepth(depth,depthDisp,1000);
// 	cv::imshow("color",color);
// 	cv::imshow("depth",depthDisp);
// 	cv::waitKey(1);
		 
 }
void PointCloudMapper::callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr msgPointCloud,
								const geometry_msgs::PoseStamped::ConstPtr tcw )
 {
	geometry_msgs::PoseStamped Tcw=*tcw; 

	Eigen::Quaterniond q =Eigen::Quaterniond(Tcw.pose.orientation.w,Tcw.pose.orientation.x,Tcw.pose.orientation.y,Tcw.pose.orientation.z) ;
	Eigen::AngleAxisd V6(q);
	//  V6.fromRotationMatrix<double,3,3>(q.toRotationMatrix());
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
	T.rotate(V6);  // 旋转部分赋值
	T(0,3)= Tcw.pose.position.x;
	T(1,3)= Tcw.pose.position.y;
	T(2,3)= Tcw.pose.position.z;
	// 已测试接受到的数据没有问题
	//cout<< GREEN<<"T:"<<T.matrix()<<WHITE<<endl;
 }
void PointCloudMapper::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr &outputMap)
{
	   unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );   
	   outputMap= globalMap;
}
// 复位点云显示模块
void PointCloudMapper::reset()
{
      mvGlobalPointCloudsPose.clear();
      mvGlobalPointClouds.clear();
      mGlobalPointCloudID=0;
	  mLastGlobalPointCloudID=0;
}

void PointCloudMapper::shutdown()
{
	{
		unique_lock<mutex> lck(shutDownMutex);
		shutDownFlag = true;
	}
	string save_path = "/home/crp/resultPointCloudFile.pcd";
	pcl::io::savePCDFile(save_path,*globalMap);
	cout<<"save pcd files to :  "<<save_path<<endl;
}
// -----end of namespace
}
