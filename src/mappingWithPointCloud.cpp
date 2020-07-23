/**
 * @function 接受点云和 相机位姿Tcw 拼接生成全局点云
 * 
 * 
 * @param topicColor
 * @param topicDepth
 * @param topicTcw
 * @param cameraParamFile  
 * 
 * 2019-12-1 修改于高翔发布的点云构建线程
 * 2020-7-5 通过创建一个opencv的窗口来获取按键的值，用于在建图结束的时候保存地图。
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
#include <condition_variable>
#include <ctime>
#include <climits>
 

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
#include<pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include<pcl/filters/passthrough.h>
 #include<pcl/filters/voxel_grid.h>
 
 
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

 

#include <Eigen/Core>
#include <Eigen/Geometry>  // Eigen 几何模块
#include <opencv2/highgui/highgui.hpp>
 

 
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
 
 
 
 
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



using namespace std;
 
class Receiver
{
	typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
                  enum Mode
                  {
		      NOTHING = 0,
		      IMAGE ,
		      DEPTH,
		      BOTH,
		      CLOUD
                  };
   
    private:
	 bool mbuseExact, mbuseCompressed;
	 shared_ptr<thread>  viewerThread;   
		 
	 size_t queueSize;
	unsigned int index=0;
    float mresolution = 0.04;
	float mDepthMapFactor;
	float mcx=0,mcy=0,mfx=0,mfy=0;
    pcl::VoxelGrid<PointT>  voxel; //点云显示精度
     pcl::PassThrough<PointT> passthrough;
	  
     size_t  lastKeyframeSize =0; // 占存变量
     size_t mGlobalPointCloudID=0; //点云ID  指示接受到的点云数量
     size_t  mLastGlobalPointCloudID=0;   //用于指示建图处理到某一个位置
	 
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
     mutex   deletekeyframeMutex;
 
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy_2;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproximateSyncPolicy_2;
	message_filters::Synchronizer<ExactSyncPolicy_2> *syncExact_2 ;
	message_filters::Synchronizer<ApproximateSyncPolicy_2> *syncApproximate_2 ;
	//message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
	//message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;
	
	
	 //ROS varible
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner;
	ros::Subscriber sub ;;
	ros::Publisher pub_global_pointcloud,pub_local_pointcloud;
	image_transport::ImageTransport it;
	//image_transport::SubscriberFilter *subImageColor, *subImageDepth;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *tcw_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *pointcloud_sub;
 

public:
 Receiver()
  :nh("~"), spinner(2), it(nh)
{
 
	lastKeyframeSize=0;
	mGlobalPointCloudID=0; //点云ID
	mLastGlobalPointCloudID=0;

	pub_global_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("Global/PointCloudOutput", 1); 
	pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("Local/PointCloudOutput", 10); 

	std::string topicColor ,topicDepth,topicTcw,topicPointCloud;
	float resolution_,mdepthfactor_=1;
	int queueSize_;
	bool mbuseExact_;
	if(ros::param::get("~topicTcw" ,topicTcw))
	;
	else
	topicTcw = "/STEREO/CameraPose";
	if(ros::param::get("~topicPointCloud" ,topicPointCloud))
	;
	else
	topicPointCloud = "/elas/point_cloud";
 
	nh.param<float>("resolution", resolution_, 0.1);
	nh.param<int>("queueSize", queueSize_, 10);
	nh.param<bool>("buseExact", mbuseExact_, true);
	
   mbuseExact = mbuseExact_;  //
   mresolution = resolution_;
   queueSize=queueSize_;
	   
	tcw_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, topicTcw, queueSize);
   pointcloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topicPointCloud, queueSize);

	cout<<"topicTcw: "<<topicTcw<<endl;
	cout<<"topicPointCloud: "<<topicPointCloud<<endl;
 	cout<<"resolution: "<<mresolution<<endl;
 	cout<<"queueSize: "<<queueSize<<endl;
	cout<<"mbuseExact: "<<mbuseExact<<endl;
 
	//接受点云和位姿数据
   if(mbuseExact)
	{
		syncExact_2 = new message_filters::Synchronizer<ExactSyncPolicy_2 >(ExactSyncPolicy_2 (queueSize), *pointcloud_sub, *tcw_sub);
		syncExact_2->registerCallback(boost::bind( &Receiver::callback_pointcloud, this, _1, _2));
	}
	else
	{
		syncApproximate_2  = new message_filters::Synchronizer<ApproximateSyncPolicy_2 >(ApproximateSyncPolicy_2 (queueSize), *pointcloud_sub, *tcw_sub);
		syncApproximate_2 ->registerCallback(boost::bind(&Receiver::callback_pointcloud, this, _1, _2));
	}
 

	voxel.setLeafSize( mresolution, mresolution, mresolution);
	globalMap = boost::make_shared< PointCloud >( );
	localMap = boost::make_shared< PointCloud >( );

	pub_global_pointcloud= nh.advertise<sensor_msgs::PointCloud2> ("Global/PointCloudOutput", 1); 
	pub_local_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("Local/PointCloudOutput", 10); 
     
   //viewerThread = make_shared<thread>( bind(&Receiver::viewer, this ) );
 
}

~Receiver()
{
 // viewerThread->join();
 shutdown();
}
 

pcl::PointCloud< PointT >::Ptr generatePointCloud( int index_i)
{
		PointCloud::Ptr tmp( new PointCloud() );
		*tmp = 	mvGlobalPointClouds[index_i];
		Eigen::Isometry3d T = 	mvGlobalPointCloudsPose[index_i];
			// 已测试接受到的数据没有问题
	  //cout<<"T:"<<T.matrix()<<endl;
		if(tmp->empty() )
		 return nullptr;
      	//cout<<"T:"<<T.matrix()<<endl;
		chrono::steady_clock::time_point t1 = chrono::steady_clock::now();   
		/*方法一：直通滤波器对点云进行处理。*/
		PointCloud::Ptr cloud_after_PassThrough(new PointCloud);//
		passthrough.setInputCloud(tmp);//输入点云
		passthrough.setFilterFieldName("z");//对z轴进行操作   
		passthrough.setFilterLimits(-1.0,15.0);//设置直通滤波器操作范围 KITTI 参数 -1.0,15.0
		passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
		passthrough.filter(*tmp);//执行滤波，过滤结果保存在 cloud_after_PassThrough

		passthrough.setInputCloud(tmp);//输入点云
		passthrough.setFilterFieldName("y");//对y轴进行操作
		passthrough.setFilterLimits(-6.0,6.0);//设置直通滤波器操作范围 KITTI 参数 -6.0,6.0
		passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
		passthrough.filter(*tmp);//执行滤波，过滤结果保存在 cloud_after_PassThrough
		
		passthrough.setInputCloud(tmp);//输入点云
		passthrough.setFilterFieldName("x");//对x轴进行操作
		passthrough.setFilterLimits(-6.0,6.0);//设置直通滤波器操作范围 KITTI 参数 -6.0,6.0
		passthrough.setFilterLimitsNegative(false);//false就是 删除此区间外的
		passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
         	//std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;

		PointCloud::Ptr cloud_voxel_tem(new PointCloud);
		cloud_after_PassThrough->is_dense = false;
		voxel.setInputCloud( cloud_after_PassThrough );
		//voxel.setLeafSize( mresolution, mresolution, mresolution);
		voxel.filter( *cloud_voxel_tem );
		//cloud_after_PassThrough->swap( *cloud_voxel_tem );
		
		
		mLastGlobalPointCloudID++; //用于指示处理到哪一个位置
		PointCloud::Ptr cloud1(new PointCloud);
		pcl::transformPointCloud( *cloud_voxel_tem, *cloud1, T.matrix());
		
		chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
		chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	
		cout<<GREEN<<"generate point cloud from  kf-ID:"<<mLastGlobalPointCloudID<<", size="<<cloud1->points.size()
		<<", cost time: "<<time_used.count()*1000<<" ms ."<<WHITE<<endl;

 
      return cloud1;
}
/** 
 * 原来版本的显示函数
 * 由于随着尺寸的增加以后,显示函数会异常退出
 */
  void viewer(void)
{
	pcl::visualization::CloudViewer pcl_viewer("viewer");
	//pcl::visualization::PCLVisualizer viewer; 
	//viewer.setBackgroundColor(100, 100, 100); // rgb
	
	size_t N=0,i=0;
	bool KFUpdate=false;
	cv::namedWindow("pointcloud");
	//pcl::visualization::KeyboardEvent event;
	//ros::AsyncSpinner spinner(2); // Use 1threads
	spinner.start();
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
	  if(KFUpdate || N>lastKeyframeSize)
	  {
		  for (  i=lastKeyframeSize; i<N && i<(lastKeyframeSize+5); i++ )  
		  {
				PointCloud::Ptr cloud1( new PointCloud() );
				cloud1 = generatePointCloud(i);				
				*globalMap += *cloud1;		
				//sensor_msgs::PointCloud2 local;  
				//pcl::toROSMsg(*cloud1,local);// 转换成ROS下的数据类型 最终通过topic发布
				//local.header.stamp=ros::Time::now();
				//local.header.frame_id  ="world";
				//pub_local_pointcloud.publish(local);
				//pcl_viewer.showCloud( cloud1 );
				
		  }

			int buff_length=150;
			if(i > (buff_length+5))
			{
			 		unique_lock<mutex> lck( deletekeyframeMutex );
					mvGlobalPointCloudsPose.erase(mvGlobalPointCloudsPose.begin(),mvGlobalPointCloudsPose.begin()+buff_length);
					mvGlobalPointClouds.erase(mvGlobalPointClouds.begin(),mvGlobalPointClouds.begin()+buff_length);
					i=i-buff_length;
					cout<<RED<<"delete keyframe ...."<<WHITE<<endl;
			}
			
		// publish global pointcloud map in ROS topic
// 		sensor_msgs::PointCloud2 output;  
// 		pcl::toROSMsg(*globalMap,output);// 转换成ROS下的数据类型 最终通过topic发布
// 		output.header.stamp=ros::Time::now();
// 		output.header.frame_id  ="world";
// 		pub_global_pointcloud.publish(output);
		 pcl_viewer.showCloud( globalMap );	 
		 cout<<"show global map, size="<<globalMap->points.size()<<endl;
 
		//  if(((N-lastKeyframeSize)>2 )&& (globalMap->points.size()>0))
		// if((N==i )&& (globalMap->points.size()>0))
		//		shutdown(); //处理到最后一帧的时候保存点云，写文件很耗时
		  
		   lastKeyframeSize = i;
		 //viewer.addPointCloud(globalMap, "globalMap"); 
		//  viewer.spin();
		 
	   }

// 	   	if (event.keyDown()) 
// 	{
// 		//打印出按下的按键信息
// 		cout << event.getKeySym() << endl;
// 	}
	  int key_value=cv::waitKey(1);
 
 	  switch (key_value) 
       {
 		case 's': 
							cout<<"key_value:"<<key_value<<endl; 
							shutdown(); 
							break;
//         case 'S': cout<<"key_value:"<<key_value<<endl; break;
//         case '\r':break;
//         case 0x18:break;//cancel
//         case 0x1B:break;//escape
           default:break;
         }
	}
	shutdown();
	spinner.stop();
}

void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr msgPointCloud,
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
	//cout<<"T:"<<T.matrix()<<endl;

	PointCloud::Ptr tem_cloud1( new PointCloud() );
	pcl::fromROSMsg(*msgPointCloud, *tem_cloud1);
	mvGlobalPointClouds.push_back(*tem_cloud1); //存储关键帧对应的点云序列
	mvGlobalPointCloudsPose.push_back(T);

	unique_lock<mutex> lck(keyframeMutex);
	{
	mGlobalPointCloudID++;
	mbKeyFrameUpdate =true;
	}
	cout<<"receive a keyframe, id = "<<mGlobalPointCloudID<<"     map size="<<tem_cloud1->points.size()<<endl;	;
		
 }

 void shutdown()
{
		   if( globalMap->points.size()>0)
		   {	
				string save_path = "/home/crp/resultPointCloudFile_KITTI.pcd";
				cout<<"save pcd files to :  "<<save_path<<endl;
				pcl::io::savePCDFile(save_path,*globalMap);
		   }
		   else
			   cout<<"globalMap is empty, Nothing to save ... "<<endl;
}
};

int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "mappingWithPointCloud", ros::init_options::AnonymousName);

    if(!ros::ok())
    {
	    return 0;
    }
     
    Receiver receiver_pointcloud;
    cout<<"starting receiver..."<<endl;
   receiver_pointcloud.viewer();
  ros::shutdown();
	ros::waitForShutdown();
     cout<<"exit  receiver..."<<endl;
    return 0;
}
