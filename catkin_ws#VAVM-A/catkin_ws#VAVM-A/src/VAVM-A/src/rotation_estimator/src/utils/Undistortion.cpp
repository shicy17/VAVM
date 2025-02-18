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

#include "utils/System.h"

void System::UndistortEvent(){
    if(eventBundle.size == 0){
        return;
    }
    
    eventUndistorted.Copy(eventBundle);
    std::vector<cv::Point2f> raw_event_point(eventBundle.size),
                             undistorted_event_point(eventBundle.size);            
    for (size_t pts_iter = 0; pts_iter < eventBundle.size; pts_iter++){
        raw_event_point[pts_iter] = cv::Point2f(eventBundle.x[pts_iter], eventBundle.y[pts_iter]);
    }
    cv::undistortPoints(raw_event_point, undistorted_event_point, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix); 
    cv::Mat temp = cv::Mat(eventBundle.size, 2, CV_32FC1);
    temp.data = cv::Mat(undistorted_event_point).data;
    cv::cv2eigen(temp, eventUndistorted.coord);
    
    eventUndistorted.x.resize(eventUndistorted.coord.col(0).size());
    eventUndistorted.y.resize(eventUndistorted.coord.col(1).size());

    Eigen::VectorXf::Map(&eventUndistorted.x[0], eventUndistorted.coord.col(0).size()) = Eigen::VectorXf(eventUndistorted.coord.col(0));
    Eigen::VectorXf::Map(&eventUndistorted.y[0], eventUndistorted.coord.col(1).size()) = Eigen::VectorXf(eventUndistorted.coord.col(1));  
    
    eventUndistorted.DiscriminateInner(width, height);
    eventUndistorted.SetCoord();
}

void System::UndistortEventOrg(){
    if(eventBundleOrg.size == 0){
        return;
    }
    
    eventUndistortedOrg.Copy(eventBundleOrg);
    std::vector<cv::Point2f> raw_event_pointOrg(eventBundleOrg.size),
                             undistorted_event_pointOrg(eventBundleOrg.size);            
    for (size_t pts_iter = 0; pts_iter < eventBundleOrg.size; pts_iter++){
        raw_event_pointOrg[pts_iter] = cv::Point2f(eventBundleOrg.x[pts_iter], eventBundleOrg.y[pts_iter]);
    }
    cv::undistortPoints(raw_event_pointOrg, undistorted_event_pointOrg, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix); 
    cv::Mat temp = cv::Mat(eventBundleOrg.size, 2, CV_32FC1);
    temp.data = cv::Mat(undistorted_event_pointOrg).data;
    cv::cv2eigen(temp, eventUndistortedOrg.coord);
    
    eventUndistortedOrg.x.resize(eventUndistortedOrg.coord.col(0).size());
    eventUndistortedOrg.y.resize(eventUndistortedOrg.coord.col(1).size());

    Eigen::VectorXf::Map(&eventUndistortedOrg.x[0], eventUndistortedOrg.coord.col(0).size()) = Eigen::VectorXf(eventUndistortedOrg.coord.col(0));
    Eigen::VectorXf::Map(&eventUndistortedOrg.y[0], eventUndistortedOrg.coord.col(1).size()) = Eigen::VectorXf(eventUndistortedOrg.coord.col(1));  
    
    eventUndistortedOrg.DiscriminateInner(width, height);
    eventUndistortedOrg.SetCoord();
}

// Generate mesh for undistortion
void System::GenerateMesh(){
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat::eye(3, 3, CV_32FC1), 
                                cameraMatrix, cv::Size(width, height), CV_32FC1, 
                                undist_mesh_x, undist_mesh_y); 
}
// Undistort image
void System::UndistortImage(){
    undistorted_image.time_stamp = current_image.time_stamp;
    undistorted_image.seq = current_image.seq;

    cv::remap(current_image.image, undistorted_image.image, undist_mesh_x, undist_mesh_y, CV_INTER_LINEAR);
}
