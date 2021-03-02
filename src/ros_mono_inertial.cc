/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of
 * Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <ImuTypes.h>
#include <System.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

using namespace std;
/// Uncomment when running valgrind memtest
//#define VALGRIND_CHECK

class MonoInertialSlamNode : public rclcpp::Node
{
  public:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;

    MonoInertialSlamNode(ORB_SLAM3::System* slam) : Node("orbslam"), slam_(slam)
    {
        std::cout << " ============== MonoInertialSlamNode ============" << std::endl;
        rclcpp::QoS qosImg = rclcpp::QoS(10).best_effort();
        rclcpp::QoS qosImu = rclcpp::QoS(100).best_effort();

        declare_parameter<string>("image_topic", "/camera/image");
        declare_parameter<string>("imu_topic", "/imu");
        declare_parameter<string>("camera_frame_id", "base_link");
        declare_parameter<string>("map_frame_id", "map");

        const std::string image_topic = get_parameter("image_topic").as_string();
        const std::string imu_topic = get_parameter("imu_topic").as_string();
        const std::string camera_frame_id = get_parameter("camera_frame_id").as_string();
        const std::string map_frame_id = get_parameter("map_frame_id").as_string();

        std::cout << "Listening to images from topic '" << image_topic << "'" << std::endl;
        std::cout << "Listening to imu from topic '" << imu_topic << "'" << std::endl;

        // Maximum delay, 5 seconds
        image_subscriber_ = this->create_subscription<ImageMsg>(
            image_topic, qosImg, std::bind(&MonoInertialSlamNode::GrabImage, this, std::placeholders::_1));
        imu_subscriber_ = this->create_subscription<ImuMsg>(
            imu_topic, qosImu, std::bind(&MonoInertialSlamNode::GrabImu, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        transformStamped_.header.stamp = now();
        transformStamped_.header.frame_id = map_frame_id;
        transformStamped_.child_frame_id = camera_frame_id;
        transformStamped_.transform.translation.x = 0.;
        transformStamped_.transform.translation.y = 0.;
        transformStamped_.transform.translation.z = 0.;
        transformStamped_.transform.rotation.x = 0.0;
        transformStamped_.transform.rotation.y = 0.0;
        transformStamped_.transform.rotation.z = 0.0;
        transformStamped_.transform.rotation.w = 1.0;

        // send temporary transform
        // tf_broadcaster_->sendTransform(transformStamped_);

        sync_thread_running_ = true;
        sync_thread_ = std::make_unique<std::thread>(&MonoInertialSlamNode::SyncWithImu, this);
    }

    ~MonoInertialSlamNode()
    {
        sync_thread_running_ = false;
        sync_thread_->join();

        // Stop all threads
        slam_->Shutdown();

        // Save camera trajectory
        slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ORB_SLAM3::System* slam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    geometry_msgs::msg::TransformStamped transformStamped_;
    queue<ImuMsg::ConstSharedPtr> imu_buf_;
    queue<ImageMsg::ConstSharedPtr> img0_buf_;

    std::mutex img_mutex_, imu_mutex_;

    std::atomic_bool sync_thread_running_;
    std::unique_ptr<std::thread> sync_thread_;

    const bool clahe_flag_ = false;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    void GrabImu(ImuMsg::ConstSharedPtr imu_msg)
    {
        // std::cout << "MonoInertialSlamNode::GrabImu(" << imu_msg->header.frame_id << ")" << std::endl;
        imu_mutex_.lock();
        imu_buf_.push(imu_msg);
        imu_mutex_.unlock();
        return;
    }

    void GrabImage(ImageMsg::ConstSharedPtr img_msg)
    {
        std::cout << "MonoInertialSlamNode::GrabImage(" << img_msg->width << "x" << img_msg->height << ")" << std::endl;
        img_mutex_.lock();
        if (!img0_buf_.empty())
            img0_buf_.pop();
        img0_buf_.push(img_msg);
        img_mutex_.unlock();
    }

    cv::Mat GetImage(const ImageMsg::ConstSharedPtr& img_msg)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }

        if (cv_ptr->image.type() == 0)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }

    void SyncWithImu()
    {
        while (sync_thread_running_)
        {
            cv::Mat im;
            double tIm = 0;
            if (!img0_buf_.empty() && !imu_buf_.empty())
            {
                tIm = img0_buf_.front()->header.stamp.sec;
                if (tIm > imu_buf_.back()->header.stamp.sec)
                    continue;
                {
                    img_mutex_.lock();
                    im = GetImage(img0_buf_.front());
                    img0_buf_.pop();
                    img_mutex_.unlock();
                }

                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                imu_mutex_.lock();
                if (!imu_buf_.empty())
                {
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    while (!imu_buf_.empty() && imu_buf_.front()->header.stamp.sec <= tIm)
                    {
                        double t = imu_buf_.front()->header.stamp.sec;
                        cv::Point3f acc(imu_buf_.front()->linear_acceleration.x,
                                        imu_buf_.front()->linear_acceleration.y,
                                        imu_buf_.front()->linear_acceleration.z);
                        cv::Point3f gyr(imu_buf_.front()->angular_velocity.x,
                                        imu_buf_.front()->angular_velocity.y,
                                        imu_buf_.front()->angular_velocity.z);
                        vImuMeas.emplace_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                        imu_buf_.pop();
                    }
                }
                imu_mutex_.unlock();
                if (clahe_flag_)
                    clahe_->apply(im, im);

                cv::Mat Tcw = slam_->TrackMonocular(im, tIm, vImuMeas);
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
                    const bool is_translation_valid = (!isnan(transformStamped_.transform.translation.x)) &&
                                                      (!isnan(transformStamped_.transform.translation.y)) &&
                                                      (!isnan(transformStamped_.transform.translation.z));

                    const bool is_rotation_valid = (!isnan(transformStamped_.transform.rotation.x)) &&
                                                   (!isnan(transformStamped_.transform.rotation.y)) &&
                                                   (!isnan(transformStamped_.transform.rotation.z)) &&
                                                   (!isnan(transformStamped_.transform.rotation.w));

                    if (is_translation_valid && is_rotation_valid)
                    {
                        tf_broadcaster_->sendTransform(transformStamped_);
                    }
                }
                else
                {
                    // continue sending previous transform
                    // tf_broadcaster_->sendTransform(transformStamped_);
                }
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
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
    std::cout << "IMU SLAM Initialization" << std::endl;
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR);
    std::cout << "Node Initialization" << std::endl;

    auto node = std::make_shared<MonoInertialSlamNode>(&slam);

#ifdef VALGRIND_CHECK
    for (std::size_t i = 0; i < 100000; i++)
    {
        rclcpp::spin_some(node);
    }
#else
    rclcpp::spin(node);
#endif

    rclcpp::shutdown();

    return 0;
}
