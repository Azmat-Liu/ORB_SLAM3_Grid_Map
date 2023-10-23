#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Converter.h>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include "utility.hpp"
using namespace std;
using std::placeholders::_1;

int all_pts_pub_gap = 0;
bool pub_all_pts = false;
int pub_count = 0;
int frame_id;

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(ORB_SLAM3::System* pSLAM, const string &strDoEqual);

    ~MonoInertialNode();

private:

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg0);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();
    void GrabMap(cv::Mat Tcw);
    
    ORB_SLAM3::System* m_SLAM;
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImage0_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pts_and_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_all_kf_and_pts;

    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<ImageMsg::SharedPtr> image0Buf_;
    std::mutex bufMutex0_;

    bool doEqual_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM, const string &strDoEqual)
    :   Node("ORB_SLAM3_ROS2"),m_SLAM(pSLAM)
{
    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Equal: " << doEqual_ << std::endl;

    subImu_ = this->create_subscription<ImuMsg>("/imu/data_raw", 1000, std::bind(&MonoInertialNode::GrabImu, this, _1));
    subImage0_ = this->create_subscription<ImageMsg>("/image_raw",100,std::bind(&MonoInertialNode::GrabImage, this, _1));
    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);

    
    std::cout << "slam changed" << std::endl;
}

MonoInertialNode::~MonoInertialNode()
{
    syncThread_->join();
    delete syncThread_;
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg0)
{
    bufMutex0_.lock();
    if(!image0Buf_.empty())
        image0Buf_.pop();
    image0Buf_.push(msg0);
    bufMutex0_.unlock();
}

cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInertialNode::SyncWithImu()
{
    while(1)
    {
        cv::Mat im;
        double tIm = 0;
        if(!image0Buf_.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(image0Buf_.front()->header.stamp);
            if(tIm > Utility::StampToSec(imuBuf_.back()->header.stamp))
               continue;
            
            bufMutex0_.lock();
            im = GetImage(image0Buf_.front());
            image0Buf_.pop();
            bufMutex0_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if(!imuBuf_.empty())
            {
                vImuMeas.clear();
                while(!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();
            if(bClahe_)
            {
                clahe_->apply(im,im);
            }
            cv::Mat Tcw;
            Sophus::SE3f Tcw_SE3f = m_SLAM->TrackMonocular(im,tIm,vImuMeas);
            Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
            cv::eigen2cv(Tcw_Matrix,Tcw);
            std::cout<<Tcw<<std::endl;
            GrabMap(Tcw);
            ++frame_id;
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void MonoInertialNode::GrabMap(cv::Mat Tcw)
{
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pts_and_pose = this->create_publisher<geometry_msgs::msg::PoseArray>("pts_and_pose",1000);
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_all_kf_and_pts = this->create_publisher<geometry_msgs::msg::PoseArray>("all_kf_and_pts",1000);
    if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap){
        pub_all_pts = true;
        pub_count = 0;
    }
    if (pub_all_pts || m_SLAM->getLoopClosing()->loop_detected || m_SLAM->getTracker()->loop_detected) {
        pub_all_pts = m_SLAM->getLoopClosing()->loop_detected = m_SLAM->getTracker()->loop_detected = false;
        geometry_msgs::msg::PoseArray kf_pt_array;
        vector<ORB_SLAM3::KeyFrame*> key_frames = m_SLAM->getMap()->GetCurrentMap()->GetAllKeyFrames();
        kf_pt_array.poses.push_back(geometry_msgs::msg::Pose());
        sort(key_frames.begin(),key_frames.end(),ORB_SLAM3::KeyFrame::lId);
        unsigned int n_kf = 0;
        for (auto key_frame : key_frames) {
            if(key_frame->isBad())
               continue;
            cv::Mat R;
            Eigen::Matrix3f RE = key_frame->GetPose().matrix3x4().block<3,3>(0,0).transpose();
            cv::eigen2cv(RE,R);
            vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
            cv::Mat twc;
            Eigen::Vector3f twcE = key_frame->GetCameraCenter();
            cv::eigen2cv(twcE,twc);
            geometry_msgs::msg::Pose kf_pose;

            kf_pose.position.x = twc.at<float>(0);
            kf_pose.position.y = twc.at<float>(1);
            kf_pose.position.z = twc.at<float>(2);
            kf_pose.orientation.x = q[0];
            kf_pose.orientation.y = q[1];
            kf_pose.orientation.z = q[2];
            kf_pose.orientation.w = q[3];
            kf_pt_array.poses.push_back(kf_pose);

            unsigned int n_pts_id = kf_pt_array.poses.size();
            kf_pt_array.poses.push_back(geometry_msgs::msg::Pose());
            std::set<ORB_SLAM3::MapPoint*> map_points = key_frame->GetMapPoints();
            unsigned int n_pts = 0;
            for (auto map_pt : map_points) {
                if (!map_pt || map_pt->isBad()) {
                    continue;
                }
                cv::Mat pt_pose;
                Eigen::Vector3f pt_poseE = map_pt->GetWorldPos();
                cv::eigen2cv(pt_poseE,pt_pose);
                if (pt_pose.empty()) {
                    continue;
                }
                geometry_msgs::msg::Pose curr_pt;
                curr_pt.position.x = pt_pose.at<float>(0);
                curr_pt.position.y = pt_pose.at<float>(1);
                curr_pt.position.z = pt_pose.at<float>(2);
                kf_pt_array.poses.push_back(curr_pt);
                ++n_pts;
            }
            geometry_msgs::msg::Pose n_pts_msg;
            n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
            kf_pt_array.poses[n_pts_id] = n_pts_msg;
            ++n_kf;
        }
        geometry_msgs::msg::Pose n_kf_msg;
        n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
        kf_pt_array.poses[0] = n_kf_msg;
        kf_pt_array.header.frame_id = "1";
        printf("Publishing data for %u keyfranmes\n", n_kf);
        pub_all_kf_and_pts->publish(kf_pt_array);
    }
    else if (m_SLAM->getTracker()->mCurrentFrame.is_keyframe || !Tcw.empty()){
        ++pub_count;
        m_SLAM->getTracker()->mCurrentFrame.is_keyframe = false;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);
        std::vector<ORB_SLAM3::MapPoint*> map_points = m_SLAM->GetTrackedMapPoints();
        int n_map_pts = map_points.size();

        geometry_msgs::msg::PoseArray pt_array;
        geometry_msgs::msg::Pose camera_pose;
        camera_pose.position.x =twc.at<float>(0);
        camera_pose.position.y =twc.at<float>(1);
        camera_pose.position.z =twc.at<float>(2);

        camera_pose.orientation.x =q[0];
        camera_pose.orientation.y =q[1];
        camera_pose.orientation.z =q[2];
        camera_pose.orientation.w =q[3];

        pt_array.poses.push_back(camera_pose);
        for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
            if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
                continue;
            }
            cv::Mat wp;
            Eigen::Vector3f wpE = map_points[pt_id - 1]->GetWorldPos();
            cv::eigen2cv(wpE,wp);

            if (wp.empty()) {
                continue;
            }
            geometry_msgs::msg::Pose curr_pt;
            curr_pt.position.x = wp.at<float>(0);
            curr_pt.position.y = wp.at<float>(1);
            curr_pt.position.z = wp.at<float>(2);
            pt_array.poses.push_back(curr_pt);
        }
        pt_array.header.frame_id = "1";
        pub_pts_and_pose->publish(pt_array);
    }
}

int main(int argc, char **argv)
{

    bool bEqual = false;
    if(argc < 3 || argc > 4)
    {
      cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
      rclcpp::shutdown();
      return 1;
    }

    if(argc==4)
    {
      std::string sbEqual(argv[3]);
      if(sbEqual == "true")
      bEqual = true;
    }

    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    auto node = std::make_shared<MonoInertialNode>(&SLAM, argv[2]);
    
    std::cout << "============================ " << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
