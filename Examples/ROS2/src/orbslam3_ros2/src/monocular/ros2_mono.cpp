#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"

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

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System& pSLAM);

    ~MonocularSlamNode();

//private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System& SLAM;
    
    int frame_id;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System& pSLAM)
    :   Node("ORB_SLAM3_ROS2"), SLAM(pSLAM),frame_id(0)
{
    
    m_image_subscriber = this->create_subscription<ImageMsg>("/image_raw",1,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1)); // d435i topic
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImagePtr m_cvImPtr;
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    SLAM.TrackMonocular(m_cvImPtr->image, m_cvImPtr->header.stamp.sec+(m_cvImPtr->header.stamp.nanosec * pow(10,-9)));
    
    ++frame_id;
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    auto node = std::make_shared<MonocularSlamNode>(SLAM);
    
    std::cout << "============================ " << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
