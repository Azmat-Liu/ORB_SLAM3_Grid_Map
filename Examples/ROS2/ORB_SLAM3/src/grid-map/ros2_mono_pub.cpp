#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
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

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System& pSLAM);

    ~MonocularSlamNode();

    //private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void GrabMap(cv::Mat Tcw);
    //Sophus::SE3f Tcw;
    ORB_SLAM3::System& SLAM;
    int frame_id;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pts_and_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_all_kf_and_pts;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System& pSLAM)
    :   Node("ORB_SLAM3_ROS2"), SLAM(pSLAM),frame_id(0)
{
    m_image_subscriber = this->create_subscription<ImageMsg>("/image_raw",1,std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

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
    cv::Mat Tcw;
    Sophus::SE3f Tcw_SE3f = SLAM.TrackMonocular(m_cvImPtr->image, m_cvImPtr->header.stamp.sec+(m_cvImPtr->header.stamp.nanosec * pow(10,-9)));
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);
    GrabMap(Tcw);
    ++frame_id;
}
int frame_id;
void MonocularSlamNode::GrabMap(cv::Mat Tcw)
{
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pts_and_pose = this->create_publisher<geometry_msgs::msg::PoseArray>("pts_and_pose", 1000);
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_all_kf_and_pts = this->create_publisher<geometry_msgs::msg::PoseArray>("all_kf_and_pts", 1000);


    if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
        pub_all_pts = true;
        pub_count = 0;
    }

    if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {

        pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
        geometry_msgs::msg::PoseArray kf_pt_array;
        vector<ORB_SLAM3::KeyFrame*> key_frames = SLAM.getMap()->GetCurrentMap()->GetAllKeyFrames();
        //! placeholder for number of keyframes
        kf_pt_array.poses.push_back(geometry_msgs::msg::Pose());
        sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);
        unsigned int n_kf = 0;
        for (auto key_frame : key_frames) {
            if (key_frame->isBad())
                continue;

            cv::Mat R;
            Eigen::Matrix3f RE = key_frame->GetPose().matrix3x4().block<3,3>(0,0).transpose();
            cv::eigen2cv(RE, R);

            vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
            cv::Mat twc;
            Eigen::Vector3f twcE= key_frame->GetCameraCenter();
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
            //! placeholder for number of points
            kf_pt_array.poses.push_back(geometry_msgs::msg::Pose());
            std::set<ORB_SLAM3::MapPoint*> map_points = key_frame->GetMapPoints();
            unsigned int n_pts = 0;
            for (auto map_pt : map_points) {
                if (!map_pt || map_pt->isBad()) {
                    //printf("Point %d is bad\n", pt_id);
                    continue;
                }
                cv::Mat pt_pose;
                Eigen::Vector3f pt_poseE = map_pt->GetWorldPos();
                cv::eigen2cv(pt_poseE,pt_pose);
                if (pt_pose.empty()) {
                    //printf("World position for point %d is empty\n", pt_id);
                    continue;
                }
                geometry_msgs::msg::Pose curr_pt;
                //printf("wp size: %d, %d\n", wp.rows, wp.cols);
                //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
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
        //kf_pt_array.header.seq = frame_id + 1;
        printf("Publishing data for %u keyfranmes\n", n_kf);
        pub_all_kf_and_pts->publish(kf_pt_array);
    }
    else if (SLAM.getTracker()->mCurrentFrame.is_keyframe || !Tcw.empty()) {

        ++pub_count;
        SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
        cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
        int n_map_pts = map_points.size();

        geometry_msgs::msg::PoseArray pt_array;
        //pt_array.poses.resize(n_map_pts + 1);

        geometry_msgs::msg::Pose camera_pose;

        camera_pose.position.x =twc.at<float>(0);
        camera_pose.position.y =twc.at<float>(1);
        camera_pose.position.z =twc.at<float>(2);

        camera_pose.orientation.x =q[0];
        camera_pose.orientation.y =q[1];
        camera_pose.orientation.z =q[2];
        camera_pose.orientation.w =q[3];

        pt_array.poses.push_back(camera_pose);

        //printf("Done getting camera pose\n");

        for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
            if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
                //printf("Point %d is bad\n", pt_id);
                continue;
            }
            cv::Mat wp;
            Eigen::Vector3f wpE = map_points[pt_id - 1]->GetWorldPos();
            cv::eigen2cv(wpE,wp);

            if (wp.empty()) {
                //printf("World position for point %d is empty\n", pt_id);
                continue;
            }
            geometry_msgs::msg::Pose curr_pt;
            //printf("wp size: %d, %d\n", wp.rows, wp.cols);
            //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
            curr_pt.position.x = wp.at<float>(0);
            curr_pt.position.y = wp.at<float>(1);
            curr_pt.position.z = wp.at<float>(2);
            pt_array.poses.push_back(curr_pt);
            //printf("Done getting map point %d\n", pt_id);
        }
        pt_array.header.frame_id = "1";
        pub_pts_and_pose->publish(pt_array);
        //pub_kf.publish(camera_pose);
    }

}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    //MonocularSlamNode cv;
    auto node = std::make_shared<MonocularSlamNode>(SLAM);

    std::cout << "============================ " << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
