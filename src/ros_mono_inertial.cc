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

    MonoInertialSlamNode(ORB_SLAM3::System* pSLAM,
                         std::string image_topic = "/camera/image",
                         std::string imu_topic = "/imu")
        : Node("orbslam"), m_SLAM(pSLAM)
    {
        std::cout << " ============== MonoInertialSlamNode ============" << std::endl;
        rclcpp::QoS qosImg = rclcpp::QoS(100).best_effort();
        rclcpp::QoS qosImu = rclcpp::QoS(1000).best_effort();

        // Maximum delay, 5 seconds
        image_subscriber_ = this->create_subscription<ImageMsg>(
            image_topic, qosImg, std::bind(&MonoInertialSlamNode::GrabImage, this, std::placeholders::_1));
        imu_subscriber_ = this->create_subscription<ImuMsg>(
            imu_topic, qosImu, std::bind(&MonoInertialSlamNode::GrabImu, this, std::placeholders::_1));

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

        sync_thread_running = true;
        sync_thread = std::make_unique<std::thread>(&MonoInertialSlamNode::SyncWithImu, this);
    }

    ~MonoInertialSlamNode()
    {
        sync_thread_running = false;
        sync_thread->join();

        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ORB_SLAM3::System* m_SLAM;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    geometry_msgs::msg::TransformStamped transformStamped_;
    queue<ImuMsg::ConstSharedPtr> imuBuf;
    std::mutex mImgMutex, mImuMutex;
    queue<ImageMsg::ConstSharedPtr> img0Buf;
    std::atomic_bool sync_thread_running;
    std::unique_ptr<std::thread> sync_thread;

    const bool mbClahe = false;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    void GrabImage(ImageMsg::ConstSharedPtr img_msg)
    {
        mImgMutex.lock();
        if (!img0Buf.empty())
            img0Buf.pop();
        img0Buf.push(img_msg);
        mImgMutex.unlock();
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
        while (sync_thread_running)
        {
            cv::Mat im;
            double tIm = 0;
            if (!img0Buf.empty() && !imuBuf.empty())
            {
                tIm = img0Buf.front()->header.stamp.sec;
                if (tIm > imuBuf.back()->header.stamp.sec)
                    continue;
                {
                    mImgMutex.lock();
                    im = GetImage(img0Buf.front());
                    img0Buf.pop();
                    mImgMutex.unlock();
                }

                vector<ORB_SLAM3::IMU::Point> vImuMeas;
                mImuMutex.lock();
                if (!imuBuf.empty())
                {
                    // Load imu measurements from buffer
                    vImuMeas.clear();
                    while (!imuBuf.empty() && imuBuf.front()->header.stamp.sec <= tIm)
                    {
                        double t = imuBuf.front()->header.stamp.sec;
                        cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
                                        imuBuf.front()->linear_acceleration.y,
                                        imuBuf.front()->linear_acceleration.z);
                        cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
                                        imuBuf.front()->angular_velocity.y,
                                        imuBuf.front()->angular_velocity.z);
                        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                        imuBuf.pop();
                    }
                }
                mImuMutex.unlock();
                if (mbClahe)
                    mClahe->apply(im, im);

                cv::Mat Tcw = m_SLAM->TrackMonocular(im, tIm, vImuMeas);
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

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }

    void GrabImu(ImuMsg::ConstSharedPtr imu_msg)
    {
        mImuMutex.lock();
        imuBuf.push(imu_msg);
        mImuMutex.unlock();
        return;
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
    std::cout << "IMU SLAM Initialization" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR);
    std::cout << "Node Initialization" << std::endl;

    auto node = std::make_shared<MonoInertialSlamNode>(&SLAM, topic_name);

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
