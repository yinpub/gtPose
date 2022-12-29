/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <sys/resource.h>
//从Node.h搬过来的
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <orb_slam2_ros/dynamic_reconfigureConfig.h>
#include "orb_slam2_ros/SaveMap.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
//

#include "Tracking.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
//为了避免重复引用，所以只能把Node的定义搬过来了

class Node;

namespace ORB_SLAM2
{
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

struct ORBParameters;

class System
{
public:
    // Input sensor
    //enum为一种枚举类
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //初始化系统
    System(const string strVocFile,//指定ORB字典文件路径
           const eSensor sensor, //指定传感器类型
           ORBParameters& parameters,//引用读取的相机参数
           const std::string & map_file = "",//地图序列位置和是否读取
           bool load_map = false,// map serialization addition
           Node* node = NULL);
    //以下针对不同类型传感器设计的运动跟踪

    // Process the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    void TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    void TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Process the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    void TrackMonocular(const cv::Mat &im, const double &timestamp);

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Returns true if Global Bundle Adjustment is running
    bool isRunningGBA();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    //Checks the current mode (mapping or localization) and changes the mode if requested
    void EnableLocalizationOnly (bool localize_only);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    void SetMinimumKeyFrames (int min_num_kf);

    bool SaveMap(const string &filename);

    cv::Mat GetCurrentPosition ();

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    cv::Mat DrawCurrentFrame ();

    std::vector<MapPoint*> GetAllMapPoints();

private:
    bool SetCallStackSize (const rlim_t kNewStackSize);

    rlim_t GetCurrentCallStackSize ();

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();

    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    bool LoadMap(const string &filename);

    bool currently_localizing_only_;

    bool load_map;

    std::string map_file;

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    FrameDrawer* mpFrameDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // Current position
    cv::Mat current_position_;

};

}// namespace ORB_SLAM




class Node
{
    friend class ORB_SLAM2::LocalMapping;
public:
    Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();

protected:
    void Update ();
    ORB_SLAM2::System* orb_slam_;
    ros::Time current_frame_time_;
    bool save_trajectoryTum_param_;
    std::string camera_info_topic_;

private:
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishGBAStatus (bool gba_status);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
    bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters);

    // initialization Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    tf2::Transform TransformFromMat (cv::Mat position_mat);
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher pose_odom_publisher_;//帧位姿用odom格式发出
    ros::Publisher keyframe_publisher_;//关键帧位姿发布
    ros::Publisher status_gba_publisher_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM2::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif // SYSTEM_H