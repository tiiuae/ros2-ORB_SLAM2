#include <opencv2/core/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Converter.h"
#include "vio_publisher.hpp"

VIOPublisher::VIOPublisher(rclcpp::Node *node) : tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(node))
{}

VIOPublisher::~VIOPublisher()
{
}

void VIOPublisher::PublishVIO(const cv::Mat &Tcw, const rclcpp::Time &timestamp) {
    // Publish transform
    cv::Mat R = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat t = -R*Tcw.rowRange(0,3).col(3);

    std::vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);

    float scale_factor=1.0;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = std::string("world");
    transform.header.stamp = timestamp;
    transform.child_frame_id = std::string("ORB_SLAM3");
    transform.transform.translation.x = t.at<float>(0, 0)*scale_factor;
    transform.transform.translation.y = t.at<float>(0, 1)*scale_factor;
    transform.transform.translation.z = t.at<float>(0, 2)*scale_factor;
    transform.transform.rotation.x = q[0];
    transform.transform.rotation.y = q[1];
    transform.transform.rotation.z = q[2];
    transform.transform.rotation.w = q[3];

    tf_broadcaster->sendTransform(transform);
}
