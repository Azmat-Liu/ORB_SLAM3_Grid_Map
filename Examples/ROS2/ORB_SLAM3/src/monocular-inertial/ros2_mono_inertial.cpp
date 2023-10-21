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

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include "utility.hpp"
using namespace std;
using std::placeholders::_1;

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

    ORB_SLAM3::System* m_SLAM;
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImage0_;//m_image_subscriber;

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
    subImage0_ = this->create_subscription<ImageMsg>("/image_raw",100,std::bind(&MonoInertialNode::GrabImage, this, _1)); // d435i topic
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
            m_SLAM->TrackMonocular(im,tIm,vImuMeas);
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
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
