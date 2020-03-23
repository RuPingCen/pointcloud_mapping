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
//   1.图像Vector 读取以后还没有删除，
//   2.由于需要频繁的删除和写入队列，因此到底是Vector更加适合频繁写入还是使用Vector更好
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
namespace Mapping
{
	
/*
 * 
 * @ 设置点云分辨率
 */
PointCloudMapper::PointCloudMapper(float fx_,float fy_,float cx_,float cy_,float resolution_)
									:nh("~")
{
   mresolution = resolution_;
 
	 mcx = fx_;
	 mcy = fy_;
	 mfx = cx_;
	 mfy = cy_;
	 
    voxel.setLeafSize( mresolution, mresolution, mresolution);
    globalMap = boost::make_shared< PointCloud >( );
    localMap = boost::make_shared< PointCloud >( );
	
    lastKeyframeSize=0;
    mGlobalPointCloudID=0; //点云ID
    mLastGlobalPointCloudID=0;
	
     topic_sub ="/orbslam2_ros/RGBDImageAndPose"; 
	 pub_global_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("Global/PointCloudOutput", 1); 
	 //pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("Local/PointCloudOutput", 10); 
      sub = nh.subscribe(topic_sub,50,&PointCloudMapper::callback,this);
	 // sub = nh.subscribe(topic_sub,50,&PointCloudMapper::callback);
	//ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
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
 
	mLastGlobalPointCloudID=mGlobalPointCloudID;
	mGlobalPointCloudID++;
	mbKeyFrameUpdate =true;
	 
    cout<<"receive a keyframe, id = "<<mGlobalPointCloudID<<endl;
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
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
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
   
// rotation the pointcloud and stiching 
    PointCloud::Ptr cloud1(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud1, T.inverse().matrix());
	
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
     cout<<"generate point cloud  cost time: "<<time_used.count()*1000<<" ms ."<<endl;
	 
    cloud1->is_dense = false;
    voxel.setInputCloud( cloud1 );
    voxel.filter( *tmp );
    cloud1->swap( *tmp );
   //mvGlobalPointClouds.push_back(*cloud1) ;    //点云数据 
   
// 	 sensor_msgs::PointCloud2 output;  
// 	 pcl::toROSMsg(*cloud1,output);// 转换成ROS下的数据类型 最终通过topic发布
// 	 output.header.stamp=ros::Time::now();
// 	 output.header.frame_id  ="camera";
// 	 pub_local_pointcloud.publish(output);
	 
     cout<<"generate point cloud from  kf-ID:"<<mGlobalPointCloudID<<", size="<<cloud1->points.size();

 
     
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
	

    ros::AsyncSpinner spinner(1); // Use 1threads
    spinner.start();
 while(ros::ok());
  while(ros::ok())
    {
         {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
    
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
		  for (  i=lastKeyframeSize; i<N; i++ )  
		  {
			  if((mvGlobalPointCloudsPose.size() != colorImgs.size() )|| (mvGlobalPointCloudsPose.size()!= depthImgs.size() ) || (depthImgs.size() != colorImgs.size() ))
			  {
						cout<<" depthImgs.size != colorImgs.size()  "<<endl;
						//cout<<"depthImgs.size(): "<<depthImgs.size()<<endl;
						//cout<<"colorImgs.size(): "<<colorImgs.size()<<endl;
					   continue;
			  }
			  PointCloud::Ptr tem_cloud1( new PointCloud() );
		     PointCloud::Ptr tem_cloud2(new PointCloud);
			  cout<<"i: "<<i<<"  mvPosePointClouds.size(): "<<mvGlobalPointCloudsPose.size()<<endl;
			 tem_cloud1= generatePointCloud(colorImgs[i], depthImgs[i],mvGlobalPointCloudsPose[i]); //生成一幅点云大约在０．１s左右        点云数据 
			// tem_cloud1= generatePointCloud(colorImgs[i], depthImgs[i]);
			// Eigen::Isometry3d T_cw = mvGlobalPointCloudsPose[i];
			// pcl::transformPointCloud( *tem_cloud1, *tem_cloud2,T_cw.inverse().matrix());
			  if(tem_cloud1->empty())
					  continue;
			
			  *globalMap += *tem_cloud1;
			  
		  }
		 lastKeyframeSize = i;
		 sensor_msgs::PointCloud2 output;  
		 pcl::toROSMsg(*globalMap,output);// 转换成ROS下的数据类型 最终通过topic发布
		 output.header.stamp=ros::Time::now();
		 output.header.frame_id  ="world";
		 pub_global_pointcloud.publish(output);
        pcl_viewer.showCloud( globalMap );
		 cout<<"show global map, size="<<globalMap->points.size()<<endl;	
 
	   }	
	}
	spinner.stop();
	 ros::shutdown();
	ros::waitForShutdown();
}
// 压缩点云需要４０-50ms　解压缩需要２０-30ｍｓ
// 参考：　http://www.voidcn.com/article/p-zbhwxhfh-bpr.html
void PointCloudMapper::compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,std::stringstream& compressedData)
{
  // 压缩选项详见 /io/include/pcl/compression/compression_profiles.h
//  - LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方厘米，压缩完之后无颜色，快速在线编码 
// - LOW_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率1立方厘米，压缩完之后有颜色，快速在线编码 
// - MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率5立方毫米，压缩完之后无颜色，快速在线编码 
// - MED_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率5立方毫米，压缩完之后有颜色，快速在线编码 
// - HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方毫米，压缩完之后无颜色，快速在线编码 
// - HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR：分辨率1立方毫米，压缩完之后有颜色，快速在线编 
// - LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方厘米，压缩完之后无颜色，高效离线编码 
// - LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率1立方厘米，压缩完之后有颜色，高效离线编码 
// - MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率5立方毫米，压缩完之后无颜色，高效离线编码 
// - MED_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率5立方毫米，压缩完之后有颜色，高效离线编码 
// - HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR：分辨率1立方毫米，压缩完之后无颜色，高效离线编码 
// - HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR：分辨率1立方毫米，压缩完之后有颜色，高效离线编码
    chrono::steady_clock::time_point t1= chrono::steady_clock::now();
    bool showStatistics=true;                       //设置在标准设备上输出打印出压缩结果信息
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> * PointCloudEncoder;
    PointCloudEncoder=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
    //std::stringstream compressedData;// 存储压缩点云的字节流对象
    PointCloudEncoder->encodePointCloud(cloud, compressedData);// 压缩点云
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"\t encode point cloud  cost time: "<<time_used.count()*1000<<"ms  seconds."<<endl;
}
void PointCloudMapper::depressPointCloud(std::stringstream& compressedData,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudOut)
{
   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
  PointCloudDecoder=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> ();
  PointCloudDecoder->decodePointCloud (compressedData, cloudOut);// 解压缩点云
  //pcl::visualization::CloudViewer viewer("viewer");
  //viewer.showCloud (cloudOut);//可视化解压缩点云
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
  cout<<"\t decode point cloud  cost time: "<<time_used.count()*1000<<"ms  seconds."<<endl;
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

void PointCloudMapper::callback(const  orbslam2_ros::RGBDPose::ConstPtr rgbdpose)
 {
       
	   cv::Mat color, depth,depthDisp;
	   geometry_msgs::PoseStamped Tcw =rgbdpose->Tcw; 
	   cv_bridge::CvImageConstPtr  cv_ptr_rgb = cv_bridge::toCvCopy(rgbdpose->rgb, "rgb8");
	   cv_bridge::CvImageConstPtr cv_ptr_depth = cv_bridge::toCvCopy(rgbdpose->depth, "mono16"); // 16bits depth
	   color = cv_ptr_rgb->image;
	   depth = cv_ptr_depth->image;
	  
	    
	   Eigen::Quaterniond q =Eigen::Quaterniond(Tcw.pose.orientation.w,Tcw.pose.orientation.x,Tcw.pose.orientation.y,Tcw.pose.orientation.z) ;
	   Eigen::AngleAxisd V6(q);
	 //  V6.fromRotationMatrix<double,3,3>(q.toRotationMatrix());
	   Eigen::Isometry3d T = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
	   T.rotate(V6);  // 旋转部分赋值
	   T(0,3)= Tcw.pose.position.x;
	   T(1,3)= Tcw.pose.position.y;
	   T(2,3)= Tcw.pose.position.z;

       //  insertKeyFrame( color, depth,T);
		 
		PointCloud::Ptr tem_cloud1( new PointCloud() );
		tem_cloud1= generatePointCloud(color, depth,T); //生成一幅点云大约在０．１s左右        点云数据 
		if(tem_cloud1->empty())
			  return;
		*globalMap += *tem_cloud1;

		sensor_msgs::PointCloud2 output;  
		pcl::toROSMsg(*globalMap,output);// 转换成ROS下的数据类型 最终通过topic发布
		output.header.stamp=ros::Time::now();
		output.header.frame_id  ="world";
		pub_global_pointcloud.publish(output);
		cout<<"show global map, size="<<globalMap->points.size()<<endl;	
		
		
		
		 // 已测试接受到的数据没有问题
		//cout<< GREEN<<"T:"<<T.matrix()<<WHITE<<endl;
 
		  
        // IR image input
//         if(color.type() != CV_8U)
//         {
//               cv::Mat tmp;
//               color.convertTo(color, CV_8U);
//         }
//       if(depth.type() != CV_16U)
//         {
//              // cv::Mat tmp;
//               depth.convertTo(depth, CV_16U);
//              // cv::cvtColor(tmp, color, CV_GRAY2BGR);
//         }

//         dispDepth(depth,depthDisp,1);
//        cv::imshow("color",color);
// 	   cv::imshow("depth",depthDisp);
// 	    cv::waitKey(1);
		
 
         
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
