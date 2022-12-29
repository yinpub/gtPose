#include "RGBDNode.h"
std::string RESULT_PATH;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    node_handle.getParam("RESULT_PATH", RESULT_PATH);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}

//构造RGBD节点对象的同时，传参数给Node，创造一个Node对象
RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node(sensor, node_handle, image_transport) {
  rgb_subscriber_ =  new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 3);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 3);
  camera_info_topic_ = "/camera/rgb/camera_info";
//做RGB和D的同步
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2));
  //注册回调函数
}


RGBDNode::~RGBDNode () {
    //orb_slam_->SaveTrajectoryKITTI("/home/host/zhongxiangrong/orb_slam_2_ros/SaveTrajectoryKITTI.txt");
    //orb_slam_->SaveTrajectoryTUM();
    //orb_slam_->SaveKeyFrameTrajectoryTUM();
    if(save_trajectoryTum_param_) {
        //orb_slam_->SaveKeyFrameTrajectoryTUM("/home/host/zhongxiangrong/orb_slam_2_ros/rgbdKeyFrameTrajectoryTum.txt");
        //orb_slam_->SaveTrajectoryTUM("/home/host/zhongxiangrong/orb_slam_2_ros/rgbdTrajectoryTum.txt");
        // orb_slam_->SaveTrajectoryKITTI(RESULT_PATH);
    }
  delete rgb_subscriber_;//需要释放堆区
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.，利用cv bridge接受图像，转换为cv::Mat类型
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;//这个有什么用？

  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

  Update ();
}
