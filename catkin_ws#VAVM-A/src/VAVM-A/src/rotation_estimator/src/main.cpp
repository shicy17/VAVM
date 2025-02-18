/*
 * Copyright (C) 2025 Beihang University, Neuromorphic Vision Perception and Computing Group
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Copyright (C) Beihang University, Neuromorphic Vision Perception and Computing Group.
 * License: This code is licensed under the GNU General Public License v3.0.
 * You can redistribute it and/or modify it under the terms of the GPL-3.0 License.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

// opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// eigen
#include <eigen_conversions/eigen_msg.h>

// messages
#include <std_msgs/Float64.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>

#include "utils/System.h"

class EventGrabber
{
public:
    EventGrabber(System* sys) : system(sys) {}

    void GrabEvent(const sensor_msgs::Image::ConstPtr& imu_upd_msg, const dvs_msgs::EventArrayConstPtr& event_bundle_msg);

    System* system;
};

void EventGrabber::GrabEvent(const sensor_msgs::ImageConstPtr& imu_upd_msg, const dvs_msgs::EventArrayConstPtr& event_bundle_msg)
{
    // std::chrono::system_clock::time_point sstart_delta_t = std::chrono::system_clock::now();
    
    static int lastseq = event_bundle_msg->header.seq -1;
    lastseq = event_bundle_msg->header.seq;

    EventData eventData;
    eventData.time_stamp = event_bundle_msg->header.stamp.toSec();
    eventData.event = event_bundle_msg->events;

    double imu_pos_x = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(0,0);
    double imu_pos_y = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(1,0);
    double imu_pos_z = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(2,0);
    double imu_ort_x = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(3,0);
    double imu_ort_y = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(4,0);
    double imu_ort_z = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(5,0);
    double imu_ort_w = cv_bridge::toCvShare(imu_upd_msg)->image.at<double>(6,0);
    std::vector<double> imu_pos = {imu_pos_x, imu_pos_y, imu_pos_z};
    std::vector<double> imu_ort = {imu_ort_x, imu_ort_y, imu_ort_z, imu_ort_w};


    system->PushEventData(eventData, imu_pos, imu_ort);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotation_estimator");
    ros::start();

    ros::NodeHandle nodeHandler("~");

    std::string yaml;
    nodeHandler.param<std::string>("yaml", yaml, "");

    int queues_size;
    nodeHandler.param<int>("queues_size", queues_size, 1);

    System Sys(yaml);

    EventGrabber eventGrabber(&Sys);

    // Configuration of the subscriber and the publisher
    message_filters::Subscriber<sensor_msgs::Image> imu_sub(nodeHandler, "/dvs/imu", 10000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<dvs_msgs::EventArray> event_sub(nodeHandler, "/dvs/events", 10000, ros::TransportHints().tcpNoDelay());

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, dvs_msgs::EventArray> synPolicy;//
    message_filters::Synchronizer<synPolicy> sync(synPolicy(10000),  imu_sub, event_sub);// live = synPolicy(3)    replay = synPolicy(100)
    sync.registerCallback(boost::bind(&EventGrabber::GrabEvent, &eventGrabber, _1, _2));

    ros::spin();

    ros::shutdown();
    return 0;
}
