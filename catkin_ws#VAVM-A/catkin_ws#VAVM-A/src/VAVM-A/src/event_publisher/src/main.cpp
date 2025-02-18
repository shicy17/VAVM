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

// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//eigen
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

// general
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

std::string event_topic = "event_topic";
std::deque<dvs_msgs::Event> evt_buffer;
dvs_msgs::EventArrayPtr event_msg;

ros::Publisher event_array_pub;
ros::Publisher time_pub;

int height, width;

void event_bundle_fun(const std::string& bag_path, int accumulation_window){
    //打开rosbag文件
    rosbag::Bag inbag;
    inbag.open(bag_path, rosbag::bagmode::Read);
    if(!inbag.isOpen()) {
        ROS_ERROR_STREAM("Could not open rosbag with path '" << bag_path << "', exiting!");
        exit(EXIT_FAILURE);
        //如果无法打开rosbag 输出错误并退出程序
    }
    int cnte = 0;

    //记录处理开始的时间点 用于计算处理时间
    std::chrono::system_clock::time_point estart_delta_t = std::chrono::system_clock::now();

    //遍历rosbag文件中的所有消息
    for(rosbag::MessageInstance const m: rosbag::View(inbag)) {
        //检查是否来自事件主题
        if(m.getTopic() == event_topic) {
            //提取事件数组消息
            dvs_msgs::EventArrayPtr evt_msg = m.instantiate<dvs_msgs::EventArray>();
            //将事件数据添加到缓冲队列evt_bufffer中
            evt_buffer.insert(evt_buffer.end(), std::begin(evt_msg->events), std::end(evt_msg->events));

            //检查事件时间戳是否有序
            if(evt_buffer.back().ts < evt_buffer.front().ts) {
                ROS_WARN("Badly ordered events timestamps, cleaning the queue");
                evt_buffer.clear();
                //无序则清空队列
            } 
            else{
                //当队列中有足够多的序列开始处理
                while(!evt_buffer.empty() && evt_buffer.size() >= accumulation_window)//!vec_msg.empty() && vec_msg.back()->events.back().ts - vec_msg.front()->events.front().ts >=ros::Duration().fromSec(accumulation_window/1000.)
                {
                    //如果event_msg没有初始化 创建一个新的事件数组
                    if (!event_msg){
                        event_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
                    }

                    //创建一个3x1的矩阵 用于存储时间数据
                    cv::Mat time_msg_mat = cv::Mat::zeros(3, 1,CV_64F);
                    //设置时间数组的高度宽度
                    event_msg->height = height;
                    event_msg->width = width;
                    //记录队列中第一个事件的时间戳
                    time_msg_mat.at<double>(0,0) = evt_buffer.begin()->ts.toSec();
                    //遍历队列中的事件 直到满足累积窗口的大小
                    for(dvs_msgs::Event& e : evt_buffer) {
                        //更新矩阵中的当前时间戳
                        time_msg_mat.at<double>(1,0) = e.ts.toSec(); 
                        //将事件添加到事件数组中
                        event_msg->events.push_back(e);
                        //从队列中移除已处理的事件
                        evt_buffer.pop_front();

                        //如果事件数组达到累积窗口的大小 停止处理
                        if(event_msg->events.size() >= accumulation_window){
                            //设置事件数组的时间戳
                            event_msg->header.stamp =  e.ts; 
                         time_pub.publish(time_msg.toImageMsg());
                        event_array_pub.publish(event_msg);
                    }
                   
                    //重置事件数组指针 为下一次处理做准备
                    event_msg.reset();
                    
                    if(cnte == 0)   cnte++;

                    std::chrono::nanoseconds eelapsed_time = std::chrono::system_clock::now() - estart_delta_t;
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * time_msg_mat.at<double>(2,0))) - eelapsed_time);
                    estart_delta_t = std::chrono::system_clock::now();
                    break;
                    }
                }
            }
        }
    }             
                    
                    //计算时间间隔
                    time_msg_mat.at<double>(2,0) = time_msg_mat.at<double>(1,0) - time_msg_mat.at<double>(0,0);

                    if(cnte != 0){
                        cv_bridge::CvImage time_msg;
                        time_msg.encoding = sensor_msgs::image_encodings::TYPE_64FC1;//"32SC1";
                        time_msg.header.stamp = event_msg->header.stamp;
                        time_msg.image =  time_msg_mat;
                    }
}
              

int main(int argc, char **argv)
{
    ros::init(argc, argv, "event_publisher");

    ros::NodeHandle nh_private("~");
    
    std::string bagname = "bagname";
    nh_private.param<std::string>("bagname", bagname, "");
    if(bagname.empty()) {
        ROS_ERROR("No input topic selected, exiting!");
        exit(EXIT_FAILURE);
    }    

    int accumulation_window;
    nh_private.param<int>("accumulation_window", accumulation_window, -1);
    if(accumulation_window <= 0) {
        ROS_ERROR("No accumulation window given, exiting!");
        exit(EXIT_FAILURE);
    }
    
    nh_private.param<int>("height", height, -1);
    nh_private.param<int>("width", width, -1);
    if(height <= 0 || width <= 0) {
        ROS_ERROR("Invalid height/width parameters, exiting!");
        exit(EXIT_FAILURE);
    }
    
    nh_private.param<std::string>("event_topic", event_topic, "");
    if(event_topic.empty()) {
        ROS_ERROR("No event topic selected, exiting!");
        exit(EXIT_FAILURE);
    }

    event_array_pub = nh_private.advertise<dvs_msgs::EventArray>("/events", 1); //向rotation_estimator发布事件组
    time_pub = nh_private.advertise<sensor_msgs::Image>("/time", 1); // 向gt_publisher发布事件组对应时间间隔

    event_bundle_fun(bagname, accumulation_window); //按固定事件数分割事件组并发布
    
    return 0;
}

