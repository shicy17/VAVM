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

//eigen
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// messages
#include <geometry_msgs/PoseStamped.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>

// file manager
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

std::deque<geometry_msgs::PoseStamped> imu_buffer;
ros::Publisher imu_pub;
ros::Subscriber time_sub;
std::deque<double> time_buffer0;
std::deque<double> time_buffer1;
std::deque<double> time_buffer2;
double sleep_dur;
ros::Time last_end_ts;

int cnt = 0;

int flag = 0;

int height, width;



void time_callback(const sensor_msgs::ImageConstPtr& time_msg, std::string& bag_path, std::string& imu_topic){
    flag = 0;
    time_buffer0.push_back(cv_bridge::toCvShare(time_msg)->image.at<double>(0,0));
    time_buffer1.push_back(cv_bridge::toCvShare(time_msg)->image.at<double>(1,0));
    time_buffer2.push_back(cv_bridge::toCvShare(time_msg)->image.at<double>(2,0));

    rosbag::Bag inbag;
    inbag.open(bag_path, rosbag::bagmode::Read);
    if(!inbag.isOpen()) {
        ROS_ERROR_STREAM("Could not open rosbag with path '" << bag_path << "', exiting!");
        exit(EXIT_FAILURE);
    }

    std::chrono::system_clock::time_point istart_delta_t = std::chrono::system_clock::now();

    for(rosbag::MessageInstance const m: rosbag::View(inbag)) {
        if(m.getTopic() == imu_topic) {
            geometry_msgs::PoseStampedPtr imu_msg = m.instantiate<geometry_msgs::PoseStamped>();
            if(cnt != 0 && imu_msg->header.stamp <= last_end_ts)
                continue;

            imu_buffer.push_back(*imu_msg);

            if(imu_buffer.back().header.stamp < imu_buffer.front().header.stamp) {
                ROS_WARN("Badly ordered imus timestamps, cleaning the queue");
                imu_buffer.clear();
            }
            else {

                while(!imu_buffer.empty() && imu_buffer.back().header.stamp >= ros::Time().fromSec(time_buffer1.front())){
                    // 找到时间间隔末尾处的imu数据
                    std::vector<geometry_msgs::PoseStamped> imus_to_process;
                    for(const geometry_msgs::PoseStamped& imu :  imu_buffer) {
                        imus_to_process.push_back(imu);
                        imu_buffer.pop_front();
                        if(imus_to_process.back().header.stamp >= ros::Time().fromSec(time_buffer1.front())){
                            sleep_dur = time_buffer2.front();
                            time_buffer0.pop_front();
                            time_buffer1.pop_front();
                            time_buffer2.pop_front();
                            break;
                        }
                        imus_to_process.pop_back();
                    }

                    cv::Mat imu_pos_upd = cv::Mat::zeros(7, 1,CV_64F);
                    imu_pos_upd.at<double>(0,0) = imus_to_process[0].pose.position.x;
                    imu_pos_upd.at<double>(1,0) = imus_to_process[0].pose.position.y;
                    imu_pos_upd.at<double>(2,0) = imus_to_process[0].pose.position.z;
                    imu_pos_upd.at<double>(3,0) = imus_to_process[0].pose.orientation.x;
                    imu_pos_upd.at<double>(4,0) = imus_to_process[0].pose.orientation.y;
                    imu_pos_upd.at<double>(5,0) = imus_to_process[0].pose.orientation.z;
                    imu_pos_upd.at<double>(6,0) = imus_to_process[0].pose.orientation.w;

                    cv_bridge::CvImage imu_upd_msg;
                    imu_upd_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1;//"32SC1";
                    imu_upd_msg.header.stamp = imus_to_process[0].header.stamp;
                    imu_upd_msg.image =  imu_pos_upd;

                    imu_pub.publish(imu_upd_msg.toImageMsg());

                    last_end_ts = imus_to_process[0].header.stamp;
                    flag =1;


                    std::chrono::nanoseconds ielapsed_time = std::chrono::system_clock::now() - istart_delta_t;
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * sleep_dur)) - ielapsed_time);
                    istart_delta_t = std::chrono::system_clock::now();
                }
            }     
               
        }
        if(flag == 1)
            break;
    }
}

void imu_bundle_fun(const std::string& bag_path, int accumulation_window){
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "event_publisher");

    ros::NodeHandle nh_private("~");

    // std::string yaml = "yaml";
    // nh_private.param<std::string>("yaml", yaml, "");
    
    std::string bagname = "bagname";
    nh_private.param<std::string>("bagname", bagname, "");
    if(bagname.empty()) {
        ROS_ERROR("No input topic selected, exiting!");
        exit(EXIT_FAILURE);
    }    
    
    nh_private.param<int>("height", height, -1);
    nh_private.param<int>("width", width, -1);
    if(height <= 0 || width <= 0) {
        ROS_ERROR("Invalid height/width parameters, exiting!");
        exit(EXIT_FAILURE);
    }
    
    std::string imu_topic = "imu_topic";
    nh_private.param<std::string>("imu_topic", imu_topic, "");
    if(imu_topic.empty()) {
        ROS_ERROR("No imu topic selected, exiting!");
        exit(EXIT_FAILURE);
    }
    
    last_end_ts = ros::Time().fromSec(0.0);
    imu_pub = nh_private.advertise<sensor_msgs::Image>("/imu", 1); // 向rotation_estimator发布旋转真值
    time_sub = nh_private.subscribe<sensor_msgs::Image>("/time_bundler", 10000, boost::bind(&time_callback, _1, bagname, imu_topic)); // 接收event_publisher发布过来的事件组时间间隔

    ros::spin();

    return 0;
}
