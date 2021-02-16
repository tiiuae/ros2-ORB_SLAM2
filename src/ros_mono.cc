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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <iostream>

using std::placeholders::_1;

/// Uncomment when running valgrind memtest
//#define VALGRIND_CHECK

class MonocularSlamNode : public rclcpp::Node
{
  public:
    using ImageMsg = sensor_msgs::msg::Image;

    MonocularSlamNode(ORB_SLAM3::System* pSLAM, std::string topic_name = "/camera/image")
        : Node("orbslam"), m_SLAM(pSLAM)
    {
        std::cout << " ============== MonocularSlamNode ============" << std::endl;
        rclcpp::QoS qos(5);
        qos = qos.best_effort();
        image_subscriber_ = this->create_subscription<ImageMsg>(
            topic_name, qos, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        transformStamped_.header.stamp = now();
        transformStamped_.header.frame_id = "map";
        transformStamped_.child_frame_id = "base_link";
        transformStamped_.transform.translation.x = 0.;
        transformStamped_.transform.translation.y = 0.;
        transformStamped_.transform.translation.z = 0.;
        transformStamped_.transform.rotation.x = 0.0;
        transformStamped_.transform.rotation.y = 0.0;
        transformStamped_.transform.rotation.z = 0.0;
        transformStamped_.transform.rotation.w = 1.0;
        // send temporary transform
        tf_broadcaster_->sendTransform(transformStamped_);
    }

    ~MonocularSlamNode()
    {
        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ORB_SLAM3::System* m_SLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    geometry_msgs::msg::TransformStamped transformStamped_;

    void GrabImage(const ImageMsg::SharedPtr msg)
    {
        std::cout << "GrabImage: " << msg->header.frame_id << std::endl;
        cv_bridge::CvImagePtr m_cvImPtr;

        try
        {
            m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);
        std::cout << Tcw << std::endl;

        transformStamped_.header.stamp = now();

        if (!Tcw.empty())
        {
            tf2::Matrix3x3 tf2_rot(Tcw.at<float>(0, 0),
                                   Tcw.at<float>(0, 1),
                                   Tcw.at<float>(0, 2),
                                   Tcw.at<float>(1, 0),
                                   Tcw.at<float>(1, 1),
                                   Tcw.at<float>(1, 2),
                                   Tcw.at<float>(2, 0),
                                   Tcw.at<float>(2, 1),
                                   Tcw.at<float>(2, 2));
            tf2::Quaternion tf2_quat;
            tf2_rot.getRotation(tf2_quat);

            transformStamped_.transform.translation.x = Tcw.at<float>(0, 3);
            transformStamped_.transform.translation.y = Tcw.at<float>(1, 3);
            transformStamped_.transform.translation.z = Tcw.at<float>(2, 3);

            transformStamped_.transform.rotation.x = tf2_quat.x();
            transformStamped_.transform.rotation.y = tf2_quat.y();
            transformStamped_.transform.rotation.z = tf2_quat.z();
            transformStamped_.transform.rotation.w = tf2_quat.w();
            tf_broadcaster_->sendTransform(transformStamped_);
        }
        else
        {
            // continue sending previous transform
            tf_broadcaster_->sendTransform(transformStamped_);
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

    std::string topic_name = "/camera/image";

    std::cout << "Listening to '" << topic_name << "' topic" << std::endl;

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    std::cout << "SLAM Initialization" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);
    std::cout << "Node Initialization" << std::endl;

    auto node = std::make_shared<MonocularSlamNode>(&SLAM, topic_name);

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
