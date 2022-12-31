//标准库
#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
//eigen
#include <eigen3/Eigen/Dense>
//Ceres
#include <ceres/ceres.h>
//GTSAM
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
//aloam
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
//SC
#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 



std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;//用以存放mapping节点的位姿估计结果 精确
std::queue<nav_msgs::Odometry::ConstPtr> odometryHighBuf; //高频
std::queue<nav_msgs::Odometry::ConstPtr> VOKeyBuf;
std::queue<nav_msgs::Odometry::ConstPtr> VOBuf; 



std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;//
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;//位姿数据的时间
double timeLaserOdometryHigh = 0.0;
double timeLaser = 0.0;
double timeVOdometry=0.0;
double timeVOdometryKey=0.0;


pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; //存放关键帧点云
std::vector<Pose6D> keyframePoses; //存放关键帧位姿
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
std::vector<Pose6D> framePoses; //存放帧位姿
std::vector<Pose6D> framePosesUpdated;
std::vector<double> frameTimes;



int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;



double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;
int prev_key_node_idx;//记录上一个关键帧对应的普通帧向量所在的位置


std::string save_directory;
std::string pgKITTIformat, pgScansDirectory, pgSCDsDirectory;
std::string odomKITTIformat;
std::fstream  pgTimeSaveStream;



int sequencenum;


//*)转换位姿格式
//输入：
//  p：位姿，pose6D格式，在SC-ALOAM中自定义的一种格式
//  gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) )：Gtsam格式的位姿
gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3



Eigen::Matrix4f H_rot;

Eigen::Matrix4f H;
//*)
//输入：
//  _estimates：值
//  _filename：保存文件
void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;
    //1.生成读写对象
    std::fstream stream(_filename.c_str(), std::fstream::out);
    std::cout<<"begin write**"<<std::endl;
    std::cout<<_filename<<std::endl;

    H_rot<<	0,-1,0,0,
    0,0,-1,0,
    1,0,0,0,
    0,0,0,1;
    //2.遍历
    for(const auto& key_value: _estimates) {
        
        //2.1
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();


        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3
        
        H <<col1.x() , col2.x() , col3.x() , t.x(),
            col1.y() , col2.y() , col3.y() , t.y(),
            col1.z() , col2.z() , col3.z() , t.z(),
            0,0,0,1;

        H=H_rot*H;

        // stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
        //        << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
        //        << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                if(i==2 && j==3)
                {
                    stream <<H.row(i)[j]<< std::endl ;
                }
                else
                {
                    stream <<H.row(i)[j]<< " " ;
                }

            }
        }
    }
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} 

void laserOdometryHighHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometryHigh)
{
	mBuf.lock();
	odometryHighBuf.push(_laserOdometryHigh);
	mBuf.unlock();
} 

void VOdKeyHandler(const nav_msgs::Odometry::ConstPtr &_vOdometryKey)
{
	mBuf.lock();
	VOKeyBuf.push(_vOdometryKey);
	mBuf.unlock();
} 

void VOHandler(const nav_msgs::Odometry::ConstPtr &_vOdometry)
{
	mBuf.lock();
	VOBuf.push(_vOdometry);
	mBuf.unlock();
} 








void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

//*)
//输入：
//  _odom：接受到mapping的位姿，ROS格式
//输出：
//  Pose6D{tx, ty, tz, roll, pitch, yaw}：位姿，这里的Pose6D是SC-A-LOAM里定义的一种数据类型。
Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    //1.获取位置
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;
    //2.获取角度
        //2.1声明三个角度
    double roll, pitch, yaw;
        //2.2以四原数的形式获取pose的旋转部分
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
        //2.3获取旋转部分
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);
    //3.输出
    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom


void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}




//*)构建约束线程
void pg();
void isam_opt(void);
void gs_opt(void);

int main(int argc, char **argv)
{
//1)初始化ROS节点
	ros::init(argc, argv, "laserPGO");
	ros::NodeHandle nh;

//2)获取配置参数
	nh.param<std::string>("save_directory", save_directory, "/"); // pose assignment every k m move 

    pgKITTIformat = save_directory + "optimized_poses.txt";

   
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out); 
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    pgScansDirectory = save_directory + "Scans/";
    auto unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

    pgSCDsDirectory = save_directory + "SCDs/"; // SCD: scan context descriptor 
    unused = system((std::string("exec rm -r ") + pgSCDsDirectory).c_str());
    unused = system((std::string("mkdir -p ") + pgSCDsDirectory).c_str());

    // system params 
	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 2.0); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10.0); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);

	nh.param<double>("sc_dist_thres", scDistThres, 0.2);  
	nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor 

    nh.param<int>("seq", sequencenum,0);
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();//初始化噪声模型

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4; 
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
	nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames 
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);
//3)订阅与发布节点
	ros::Subscriber subVOdometryKey = nh.subscribe<nav_msgs::Odometry>("/orb_slam2_rgbd/Keyframe_odom", 100, VOdKeyHandler);//订阅mapping的位姿估计结果  低频里程计
    ros::Subscriber subVOdometry = nh.subscribe<nav_msgs::Odometry>("/orb_slam2_rgbd/pose_odom", 100, VOHandler);
    
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    ros::Subscriber subLaserOdometryHigh = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, laserOdometryHighHandler);

//4)创建线程

    std::thread posegraph {pg};
    std::thread optimize {isam_opt};
    //std::thread optimize {gs_opt};
//5)触发回调
 	ros::spin();

	return 0;
}


void runopt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}
//GS优化法 
void gs_opt(void){
    float hz = 1; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();

        if( gtSAMgraphMade && (odometryHighBuf.size()==0) ) {
            //2.2.1上锁
            mtxPosegraph.lock();
            initialEstimate.print("\nInitial Values:\n");
            gtSAMgraph.print("\nFactor Graph:\n");
            //2.2.2优化
            gtsam::GaussNewtonParams parameters;
            gtsam::GaussNewtonOptimizer optimizer(gtSAMgraph, initialEstimate, parameters);
            gtsam::Values results = optimizer.optimize();

            results.print("Final Result:\n");
            //isamCurrentEstimate.print("Final Result:\n");
            cout << "optimization over..." << endl;
            //2.2.3解锁
            mtxPosegraph.unlock();
            //pgKITTIformat="/home/y/evo/ev_ki/fusion/04/test04_.txt";
            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            std::cout<<"opt over"<<std::endl;
            std::chrono::milliseconds waitMs(20000);
            std::this_thread::sleep_for(waitMs);
        }

    }
}
//ISAM优化法
void isam_opt(void)
{
    float hz = 1; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        //2.1线程调频
        rate.sleep();
        //2.2当图初始化成功，进入优化
        if( gtSAMgraphMade && (odometryHighBuf.size()==0)&&(VOBuf.size())==0 ) {
            //2.2.1上锁
            mtxPosegraph.lock();
            initialEstimate.print("\nInitial Values:\n");
            //gtSAMgraph.print("\nFactor Graph:\n");
            //2.2.2优化
            runopt();
            //isamCurrentEstimate.print("Final Result:\n");
            cout << "optimization over..." << endl;
            //2.2.3解锁
            mtxPosegraph.unlock();
            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            std::cout<<"opt over"<<std::endl;
            std::chrono::milliseconds waitMs(20000);
            std::this_thread::sleep_for(waitMs);

        }
    }
}



bool systemInit=false;
//*)构建约束线程
void pg()
{
    int picnum;
    if(sequencenum==0){
        picnum=4540;
    }
    else if(sequencenum==1||sequencenum==6||sequencenum==7){
        picnum=1100;
    }
    else if (sequencenum==2){
        picnum=4660;
    }
    else if (sequencenum==3){
        picnum=800;
    }
    else if(sequencenum==4){
        picnum=270;
    }
    else if(sequencenum==5){
        picnum=2760;
    }
    else if(sequencenum==8){
        picnum=4070;
    }
    else if (sequencenum==9){
        picnum=1590;
    }
    
    while (odometryHighBuf.size()<picnum||VOBuf.size()<picnum)
    {
        std::cout<<"picnum"<<picnum<<std::endl;
        std::cout<<"HighBuf:"<<odometryHighBuf.size()<<std::endl;
        std::cout<<"keyBuf:"<<odometryBuf.size()<<std::endl;
        std::cout<<"vo:"<<VOBuf.size()<<std::endl;
        std::cout<<"vokey:"<<VOKeyBuf.size()<<std::endl;
        std::cout<<"-----------------------------------------"<<std::endl;
        std::chrono::milliseconds dura(400);
        std::this_thread::sleep_for(dura);
        continue;
       
    }
    std::cout<<"msg have collect"<<std::endl;
//------------------构建因子图
    while(1)
    {
        //loam部分
		while ( odometryHighBuf.size()>0)
        {
            while (!systemInit)
            {
                std::cout<<"init begin"<<std::endl;
                if(odometryBuf.size()>0){
                    systemInit=true;
                    std::cout<<"init over"<<std::endl;
                }
            }
            
        //1.1时间同步
            //1.1.1上锁
			mBuf.lock();
        //1.2从缓存中取出数据
            //1.2.1获取两个数据的时间
            timeLaserOdometryHigh = odometryHighBuf.front()->header.stamp.toSec();
            
            if(odometryBuf.size()>0){
                std::cout<<"mapBuf >0"<<std::endl;
                timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            }
            std::cout<<"odo:"<<odometryBuf.size()<<" time:"<<timeLaserOdometry<<std::endl;
            std::cout<<"odoHigh："<<odometryHighBuf.size()<<" time:"<<timeLaserOdometryHigh<<std::endl;

            //1.2.2获取数据
                //2.2获取位姿
            Pose6D pose_curr = getOdom(odometryHighBuf.front());//mapping坐标系 世界坐标
            odometryHighBuf.pop();
            mBuf.unlock(); 

        //1.3重置位姿变量
            odom_pose_prev = odom_pose_curr;//重置上一帧位姿
            odom_pose_curr = pose_curr;//重置当前位姿
        //1.4当积累移动或旋转超过阈值，积累量重新置0，选取当前帧为关键帧
            //1.4.1关键帧判断
            if(timeLaserOdometry==timeLaserOdometryHigh) {
                isNowKeyFrame = true;

                Pose6D keypose_curr=getOdom(odometryBuf.front());
                keyframePoses.push_back(keypose_curr);//位姿
                keyframePosesUpdated.push_back(keypose_curr); // init
                keyframeTimes.push_back(timeLaserOdometry);//位姿数据的时间

            } else {
                isNowKeyFrame = false;
            }
        //1.9关键帧数据存入向量
            //1.9.1上锁
            mKF.lock(); 
            //1.9.2存入数据
            framePoses.push_back(pose_curr);//位姿
            framePosesUpdated.push_back(pose_curr); // init
            frameTimes.push_back(timeLaserOdometryHigh);//位姿数据的时间
            //1.9.5解锁
            mKF.unlock(); 
        //1.10
            //1.10.1获取上一帧和当前帧在向量中的索引
            const int prev_node_idx = framePoses.size() - 2; 
            const int curr_node_idx = framePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
                        
            //1.10.2添加因子
                //2.1当没有添加先验节点时
            if( ! gtSAMgraphMade /* prior node */) {
                    //2.1.1创建先验因子
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(framePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
                    //2.1.2添加因子图
                        //2.1上锁
                mtxPosegraph.lock();
                        //2.2添加先验因子
                {
                    // prior factor 
                            //2.2.1添加因子
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                            //2.2.2添加变量
                    initialEstimate.insert(init_node_idx, poseOrigin);       
                }   
                        //2.3解锁
                mtxPosegraph.unlock();
                        //2.4状态置为True
                gtSAMgraphMade = true; 
                cout << "posegraph prior node " << init_node_idx << " added" << endl;
                
                Pose6D keypose_curr=getOdom(odometryBuf.front());
                std::cout<<"x"<<keypose_curr.x<<std::endl;
                std::cout<<"y"<<keypose_curr.y<<std::endl;
                std::cout<<"z"<<keypose_curr.z<<std::endl;
                keyframePoses.push_back(keypose_curr);//位姿
                keyframePosesUpdated.push_back(keypose_curr); // init
                keyframeTimes.push_back(timeLaserOdometry);//位姿数据的时间
                odometryBuf.pop();
                std::chrono::milliseconds dura(2000);
                std::this_thread::sleep_for(dura);

            }
                //2.2当添加过先验节点时 
            else if(isNowKeyFrame==false){ 
                    //2.2.1从向量中取出位姿
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(framePoses.at(prev_node_idx));//上一关键帧位姿
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(framePoses.at(curr_node_idx));//当前关键帧位姿
                    //2.2.2添加因子图
                        //2.1上锁
                mtxPosegraph.lock();
                        //2.2添加因子图的节点
                {
                            //2.2.1添加因子   odom factor
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));
                            //2.2.3添加变量
                    initialEstimate.insert(curr_node_idx, poseTo);                

                }
                        //2.3解锁
                mtxPosegraph.unlock();
        
                cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");
                //当是关键帧的时候
            else{
                std::cout<<"add key frame!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                const int prev_keynode_idx = keyframePoses.size() - 2; 
                const int curr_keynode_idx = keyframePoses.size() - 1;
                    //2.2.1从向量中取出位姿
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(framePoses.at(prev_node_idx));//上一关键帧位姿
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(framePoses.at(curr_node_idx));//当前关键帧位姿

                gtsam::Pose3 keyposeFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_keynode_idx));//上一关键帧位姿
                gtsam::Pose3 keyposeTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_keynode_idx));//当前关键帧位姿

                    //2.2.2添加因子图
                        //2.1上锁
                mtxPosegraph.lock();
                        //2.2添加因子图的节点
                {
                            //2.2.1添加odometry因子
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));
                            //2.2.2添加变量
                    initialEstimate.insert(curr_node_idx, poseTo);                
                            //2.2.3添加keyframe因子
                    
                    gtsam::Pose3 keyrelPose = keyposeFrom.between(keyposeTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_key_node_idx, curr_node_idx, keyrelPose, odomNoise));
                    prev_key_node_idx=curr_node_idx;
                
                }
                        //2.3解锁
                mtxPosegraph.unlock();
                odometryBuf.pop();
                std::chrono::milliseconds dura(20);
                std::this_thread::sleep_for(dura);
            }
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }
        
        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);
        framePoses.clear();//位姿
        framePosesUpdated.clear();
        frameTimes.clear();
        
        keyframePoses.clear();//位姿
        keyframePosesUpdated.clear(); // init
        keyframeTimes.clear();
        if(VOBuf.size()>0){
            gtSAMgraphMade=false;
            std::cout<<"VO begin----------"<<std::endl;
        }
       
        while ( VOBuf.size()>0)
        {
 
			mBuf.lock();
        //1.2从缓存中取出数据
            timeVOdometry = VOBuf.front()->header.stamp.toSec();
            
            if(VOKeyBuf.size()>0){
                std::cout<<"keyBuf >0"<<std::endl;
                timeVOdometryKey = VOKeyBuf.front()->header.stamp.toSec();
            }

            std::cout<<"voKey:"<<VOKeyBuf.size()<<" time:"<<timeVOdometryKey<<std::endl;
            std::cout<<"vo："<<VOBuf.size()<<" time:"<<timeVOdometry<<std::endl;

            Pose6D pose_curr = getOdom(VOBuf.front());//mapping坐标系 世界坐标
            VOBuf.pop();
            mBuf.unlock(); 

            odom_pose_prev = odom_pose_curr;//重置上一帧位姿
            odom_pose_curr = pose_curr;//重置当前位姿

            if(timeVOdometry==timeVOdometryKey) {
                isNowKeyFrame = true;

                Pose6D keypose_curr=getOdom(VOKeyBuf.front());
                keyframePoses.push_back(keypose_curr);//位姿
                keyframePosesUpdated.push_back(keypose_curr); // init
                keyframeTimes.push_back(timeVOdometryKey);//位姿数据的时间

            } else {
                isNowKeyFrame = false;
            }
        //1.9关键帧数据存入向量
            //1.9.1上锁
            mKF.lock(); 
            //1.9.2存入数据
            framePoses.push_back(pose_curr);//位姿
            framePosesUpdated.push_back(pose_curr); // init
            frameTimes.push_back(timeVOdometryKey);//位姿数据的时间
            //1.9.5解锁
            mKF.unlock(); 
        //1.10
            //1.10.1获取上一帧和当前帧在向量中的索引
            const int prev_node_idx = framePoses.size() - 2; 
            const int curr_node_idx = framePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
                        
            if( ! gtSAMgraphMade ) {
                gtSAMgraphMade = true; 
            }
                //2.2当添加过先验节点时 
            else if(isNowKeyFrame==false){ 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(framePoses.at(prev_node_idx));//上一关键帧位姿
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(framePoses.at(curr_node_idx));//当前关键帧位姿
                mtxPosegraph.lock();
                {   
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));
                    //initialEstimate.insert(curr_node_idx, poseTo);                
                }
                mtxPosegraph.unlock();
        
                cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
                //当是关键帧的时候
            else{
                std::cout<<"add key frame!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                const int prev_keynode_idx = keyframePoses.size() - 2; 
                const int curr_keynode_idx = keyframePoses.size() - 1;
                std::cout<<"kfsize:"<<keyframePoses.size()<<std::endl;
                    //2.2.1从向量中取出位姿
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(framePoses.at(prev_node_idx));//上一关键帧位姿
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(framePoses.at(curr_node_idx));//当前关键帧位姿

                gtsam::Pose3 keyposeFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_keynode_idx));//上一关键帧位姿
                gtsam::Pose3 keyposeTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_keynode_idx));//当前关键帧位姿

                    //2.2.2添加因子图
                mtxPosegraph.lock();
                        //2.2添加因子图的节点
                {
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));
                    std::cout<<"add graph"<<std::endl;
                    //initialEstimate.insert(curr_node_idx, poseTo);                

                    gtsam::Pose3 keyrelPose = keyposeFrom.between(keyposeTo);//获取两关键帧间相对运动
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_key_node_idx, curr_node_idx, keyrelPose, odomNoise));
                    std::cout<<"key graph"<<std::endl;
                    prev_key_node_idx=curr_node_idx;
                
                }
                mtxPosegraph.unlock();
                VOKeyBuf.pop();
                std::chrono::milliseconds dura(20);
                std::this_thread::sleep_for(dura);
            }
            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

    }
}