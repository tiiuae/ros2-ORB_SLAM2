#include "rgbd-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM2::System* pSLAM)
:   Node("orbslam"),
    m_SLAM(pSLAM), publisher(std::make_shared<VIOPublisher>(this))
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, msgRGB->header.stamp.sec);

    if (!Tcw.empty()) {
        publisher->PublishVIO(Tcw, msgRGB->header.stamp);
    }
}
