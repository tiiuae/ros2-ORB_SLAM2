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

#include <ORB_SLAM3/System.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

using namespace std;
using std::placeholders::_1;

class MonocularSlamNode : public rclcpp::Node
{
  public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM) : Node("orbslam"), m_SLAM(pSLAM)
    {
        std::cout << " ============== MonocularSlamNode ============" << std::endl;

        m_image_subscriber = this->create_subscription<ImageMsg>(
            "/camera/image", 10, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    }

    ~MonocularSlamNode()
    {
        // Stop all threads
        m_SLAM->Shutdown();

        // Save camera trajectory
        m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

  private:
    using ImageMsg = sensor_msgs::msg::Image;

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
    }

    ORB_SLAM3::System* m_SLAM;


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
};

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cerr << endl
             << "Usage: ros2 run orbslam3_node ros_mono path_to_vocabulary "
                "path_to_settings"
             << endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    std::cout << "SLAM Initialization" << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);
    std::cout << "Node Initialization" << std::endl;

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
