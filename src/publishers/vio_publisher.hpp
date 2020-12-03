#ifndef __VIO_PUBLISHER_HPP
#define __VIO_PUBLISHER_HPP

#include <tf2_ros/transform_broadcaster.h>

class VIOPublisher {
public:
    VIOPublisher(rclcpp::Node *node);
    virtual ~VIOPublisher();
    void PublishVIO(const cv::Mat &Tcw, const rclcpp::Time &timestamp);

private:
    VIOPublisher();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

#endif
