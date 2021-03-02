/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <iostream>

using std::placeholders::_1;
using TransformStamped = geometry_msgs::msg::TransformStamped;
/// Uncomment when running valgrind memtest
//#define VALGRIND_CHECK

TransformStamped cvMatToTransform(const cv::Mat& cv_mat)
{
    tf2::Quaternion tf2_quat;
    tf2::Matrix3x3 tf2_rot(cv_mat.at<float>(0, 0),
                           cv_mat.at<float>(0, 1),
                           cv_mat.at<float>(0, 2),
                           cv_mat.at<float>(1, 0),
                           cv_mat.at<float>(1, 1),
                           cv_mat.at<float>(1, 2),
                           cv_mat.at<float>(2, 0),
                           cv_mat.at<float>(2, 1),
                           cv_mat.at<float>(2, 2));

    tf2_rot.getRotation(tf2_quat);

    TransformStamped msg;
    msg.transform.translation.x = cv_mat.at<float>(0, 3);
    msg.transform.translation.y = cv_mat.at<float>(1, 3);
    msg.transform.translation.z = cv_mat.at<float>(2, 3);

    msg.transform.rotation.x = tf2_quat.x();
    msg.transform.rotation.y = tf2_quat.y();
    msg.transform.rotation.z = tf2_quat.z();
    msg.transform.rotation.w = tf2_quat.w();
    return msg;
}

bool IsTransformValid(const TransformStamped& transform)
{
    const bool is_translation_valid = (!isnan(transform.transform.translation.x)) &&
                                      (!isnan(transform.transform.translation.y)) &&
                                      (!isnan(transform.transform.translation.z));

    const bool is_rotation_valid = (!isnan(transform.transform.rotation.x)) &&
                                   (!isnan(transform.transform.rotation.y)) &&
                                   (!isnan(transform.transform.rotation.z)) && (!isnan(transform.transform.rotation.w));

    return is_translation_valid && is_rotation_valid;
}

class MonocularSlamNode : public rclcpp::Node
{
  public:
    using ImageMsg = sensor_msgs::msg::Image;

    MonocularSlamNode(ORB_SLAM3::System* pSLAM)
        : Node("orbslam"),
          slam_(pSLAM),
          clock_(RCL_SYSTEM_TIME),
          tf_buffer_(std::make_shared<rclcpp::Clock>(clock_)),
          tf_listener_(tf_buffer_, true)
    {
        std::cout << " ============== MonocularSlamNode ============" << std::endl;
        declare_parameter<string>("image_topic", "/camera/image");
        declare_parameter<string>("base_frame_id", "base_link");
        declare_parameter<string>("map_frame_id", "map");

        const std::string image_topic = get_parameter("image_topic").as_string();
        base_frame_ = get_parameter("base_frame_id").as_string();
        map_frame_ = get_parameter("map_frame_id").as_string();
        std::cout << "Listening to images from topic '" << image_topic << "'" << std::endl;

        rclcpp::QoS qos(5);
        qos = qos.best_effort();
        image_subscriber_ = create_subscription<ImageMsg>(
            image_topic, qos, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        base2map_transform_.header.stamp = now();
        base2map_transform_.header.frame_id = map_frame_;
        base2map_transform_.child_frame_id = base_frame_;
        base2map_transform_.transform.translation.x = 0.;
        base2map_transform_.transform.translation.y = 0.;
        base2map_transform_.transform.translation.z = 0.;
        base2map_transform_.transform.rotation.x = 0.0;
        base2map_transform_.transform.rotation.y = 0.0;
        base2map_transform_.transform.rotation.z = 0.0;
        base2map_transform_.transform.rotation.w = 1.0;
        // send temporary transform in order to create frame
        tf_broadcaster_->sendTransform(base2map_transform_);
    }

    ~MonocularSlamNode()
    {
        // Stop all threads
        slam_->Shutdown();

        // Save camera trajectory
        slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

  private:
    rclcpp::Clock clock_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ORB_SLAM3::System* slam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    geometry_msgs::msg::TransformStamped base2map_transform_;

    std::string base_frame_, map_frame_;

    void GrabImage(const ImageMsg::SharedPtr msg)
    {
        std::cout << "GrabImage from frame " << msg->header.frame_id << std::endl;
        cv_bridge::CvImagePtr m_cvImPtr;

        try
        {
            m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cam2map = slam_->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);

        if (cam2map.empty())
        {
            return;
        }
        std::cout << cam2map << std::endl;

        auto camera2map_transform = cvMatToTransform(cam2map);

        if (IsTransformValid(camera2map_transform))
        {
            try
            {
                //                geometry_msgs::msg::TransformStamped base2camera_transform =
                //                tf_buffer_.lookupTransform(
                //                    base_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(2.0));
                //
                //                tf2::doTransform(camera2map_transform, base2map_transform_, base2camera_transform);
                //                // transform was valid at time of frame capture
                //                base2map_transform_.header.stamp = msg->header.stamp;
                //                base2map_transform_.header.frame_id = map_frame_;
                //                base2map_transform_.child_frame_id = base_frame_;
                //                tf_broadcaster_->sendTransform(base2map_transform_);

                camera2map_transform.header.stamp = msg->header.stamp;
                camera2map_transform.header.frame_id = map_frame_;
                camera2map_transform.child_frame_id = base_frame_;
                tf_broadcaster_->sendTransform(camera2map_transform);
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR(get_logger(), "Transform error: %s, %s\n", ex.what(), "quitting callback");
                return;
            }
        }
    }
};

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: ros2 run orbslam_node ros_mono vocabulary_file settings_file" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    std::cout << "SLAM Initialization" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);
    std::cout << "Node Initialization" << std::endl;

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

#ifdef VALGRIND_CHECK
    for (std::size_t i = 0; i < 10000; i++)
    {
        rclcpp::spin_some(node);
    }
#else
    rclcpp::spin(node);
#endif

    rclcpp::shutdown();

    return 0;
}
