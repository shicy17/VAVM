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

void System::EstimateMotion(){
    UndistortEvent();
    UndistortEventOrg();
    eventUndistorted.InverseProjection(K);
    eventUndistortedOrg.InverseProjection(K);
    eventWarpedBack.Copy(eventUndistorted);
    eventWarpedBackOrg.Copy(eventUndistortedOrg);
    eventWarpedFor.Copy(eventUndistorted);
    eventOrg.Copy(eventUndistortedOrg);

    float event_num_weight = std::min(eventUndistorted.size * 1e-3 + 1e-4, 1.0);
    
    
    estimation_time_interval = eventUndistorted.time_stamp.front() - last_event_time;
    last_event_time = eventUndistorted.time_stamp.front();
    event_delta_time = eventUndistorted.time_stamp.back() - eventUndistorted.time_stamp.front();

    GenerateFrame();
    ros::Time t1 = ros::Time::now();

    ComputeAngularPosition();
    BindMap();

    nu_event0 = 1.0f;
    nu_event1 = 1.0f;
    nu_event2 = 1.0f;
    nu_map = 1.0f;
    float map_norm = 0.0, map_norm_reciprocal = 0.0;

    // angular_velocity = Eigen::Vector3f::Zero();
    angular_acc1 = Eigen::Vector3f::Zero();
    angular_acc2 = Eigen::Vector3f::Zero();
    angular_position_compensator = Eigen::Vector3f::Zero();
    
    if(rotation_estimation){
        GetWarpedEventMap(angular_position_init);
        cv::threshold(warped_map_image, truncated_map, event_image_threshold, 255, 2); 
        map_norm = cv::norm(truncated_map);
        map_norm_reciprocal = 1 / sqrt(map_norm);
        cv::GaussianBlur(truncated_map * map_norm_reciprocal, truncated_map, cv::Size(21, 21), 3);
        cv::Sobel(truncated_map, Ix_map, CV_32FC1, 1, 0);
        cv::Sobel(truncated_map, Iy_map, CV_32FC1, 0, 1);
    }
    for (int i = 0; i < optimizer_max_iter; i++){
        // Derive Error Analytic
        is_polarity_on = i < optimizer_max_iter / 2;
        // backward warping
        DeriveErrAnalyticBack(angular_velocity, angular_acc1, angular_acc2, angular_position_compensator);     
        // forward warping   
        DeriveErrAnalyticFor(angular_velocity, angular_acc1, angular_acc2);
        Gradient_vel = Jacobian_acc.transpose() * Jacobian_dt_vel;
        Gradient_acc1 = Jacobian_acc.transpose() * Jacobian_dt_acc1;
        Gradient_acc2 = Jacobian_acc.transpose() * Jacobian_dt_acc2;
        Gradient_vel -= Jacobian_acc_b.transpose() * Jacobian_dt_vel_b;
        Gradient_acc1 -= Jacobian_acc_b.transpose() * Jacobian_dt_acc1_b;
        Gradient_acc2 -= Jacobian_acc_b.transpose() * Jacobian_dt_acc2_b;

        // RMS-prop optimizer
        // optimize angular velocity
        nu_event0 = rho_event0 * nu_event0
                + (1.0f - rho_event0) * (float)(Gradient_vel.transpose() * Gradient_vel);
        update_angular_velocity = - event_num_weight * mu_event0 / std::sqrt(nu_event0 + 1e-8) * Gradient_vel;
        // angular_velocity = SO3add(update_angular_velocity, angular_velocity, true); 
        angular_velocity = update_angular_velocity + angular_velocity;  
        // optimize 1st angular acceleration
        nu_event1 = rho_event1 * nu_event1
                + (1.0f - rho_event1) * (float)(Gradient_acc1.transpose() * Gradient_acc1);
        update_angular_acc1 = - event_num_weight * mu_event1 / std::sqrt(nu_event1 + 1e-8) * Gradient_acc1;
        // angular_acc1 = SO3add(update_angular_acc1, angular_acc1, true);
        angular_acc1 = update_angular_acc1 + angular_acc1;
        // optimize 2nd angular acceleration
        nu_event2 = rho_event2 * nu_event2
                + (1.0f - rho_event2) * (float)(Gradient_acc2.transpose() * Gradient_acc2);
        update_angular_acc2 = - event_num_weight * mu_event2 / std::sqrt(nu_event2 + 1e-8) * Gradient_acc2;
        // angular_acc2 = SO3add(update_angular_acc2, angular_acc2, true); 
        angular_acc2 = update_angular_acc2 + angular_acc2;     

        // optimize angular position compensator
        if(map_norm != 0 && rotation_estimation){
            Gradient_map = Jacobian_map.transpose() * Jacobian_dt_map;
            nu_map = rho_map * nu_map
                    + (1.0f - rho_map) * (float)(Gradient_map.transpose() * Gradient_map);
            update_angular_position_compensator = - event_num_weight * mu_map / std::sqrt(nu_map + 1e-8) * Gradient_map;
            // angular_position_compensator = SO3add(update_angular_position_compensator, angular_position_compensator);
            angular_position_compensator = update_angular_position_compensator + angular_position_compensator; 
        }
    }

    cv::Mat back_warped;
    // cv::Mat for_warped, global_warped;
    GetWarpedEventPointBack(eventUndistortedOrg, eventWarpedBackOrg, angular_velocity, angular_acc1, angular_acc2, false, angular_position_compensator);
    eventWarpedBackOrg.Projection(K);
    eventWarpedBackOrg.SetXY();
    eventWarpedBackOrg.DiscriminateInner(width - 1, height - 1);
    GetEventImage(eventWarpedBackOrg, DVS_RENDERER).convertTo(back_warped, CV_8UC3);
    // GetEventImage(eventWarpedFor, DVS_RENDERER).convertTo(for_warped, CV_8UC3);
    // GetEventImage(eventWarpedBackMap, DVS_RENDERER).convertTo(global_warped, CV_8UC3);
    
    // cv::Mat back_warped_th, for_warped_th, global_warped_th;
    // cv::normalize(back_warped, back_warped_th, 0, 255,cv::NORM_MINMAX); 
    // cv::normalize(for_warped, for_warped_th, 0, 255,cv::NORM_MINMAX); 
    // cv::normalize(global_warped, global_warped_th, 0, 255,cv::NORM_MINMAX); 
    // back_warped.convertTo(back_warped_th, CV_8UC3, 255.0);  // 缩放到0-255范围
    // for(int ix = 0; ix < 346; ix++){
    //     for(int iy = 0; iy < 260; iy++){
    //         if(back_warped_th.at<unsigned short>(cv::Point(ix, iy)) == (0,0,0)){
    //             back_warped_th.at<unsigned short>(cv::Point(ix, iy)) = (230,230,230);
    //         }
    //     }
    // }
    // for_warped.convertTo(for_warped_th, CV_8UC3, 255.0);  // 缩放到0-255范围
    // global_warped.convertTo(global_warped_th, CV_8UC3, 255.0);  // 缩放到0-255范围
    imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/back_warp(renderer)/" + std::to_string(cnt_iter)  +"_b_warped"+ ".jpg", back_warped);
    // imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/for_warp(renderer)/" + std::to_string(cnt_iter)  +"_f_warped"+ ".jpg", for_warped);
    // imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/global_warp(renderer)/" + std::to_string(cnt_iter)  +"_g_warped"+ ".jpg", global_warped);

    // imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/back_ef/" + std::to_string(cnt_iter)  +"_b_warped"+ ".jpg", back_warped);
    // imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/for_ef/" + std::to_string(cnt_iter)  +"_f_warped"+ ".jpg", for_warped);
    // imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/global_ef/" + std::to_string(cnt_iter)  +"_g_warped"+ ".jpg", global_warped);


    angular_position = SO3add(angular_position_init, angular_position_compensator);
    
    ros::Time t2 = ros::Time::now();
    double total_evaluate_time = (t2-t1).toSec(); 
    total_total_evaluation_time += total_evaluate_time / (eventWarpedBack.size * 3);
    mean_average_evaluation_time = total_total_evaluation_time / (cnt_iter + 1); 

    std::ofstream mean_time("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/process_time.txt",std::ios::app);
    mean_time << std::fixed << eventWarpedBack.time_stamp.back() << " " << mean_average_evaluation_time * 1e6 << "  " << total_total_evaluation_time << "  " << total_data_size << " " << total_evaluate_time << "\n";      
    mean_time.close();

    //----------------------------------------------------
    //results
    //----------------------------------------------------
    // For dataset "EFRD -- cabinets"
    gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    gt_orientation_correction.x() = 0.251164; gt_orientation_correction.y() = 0.058256; gt_orientation_correction.z() = -0.079589; gt_orientation_correction.w() = 0.962906;
    gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EFRD -- blocks"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = 0.003256; gt_orientation_correction.y() = -0.171034; gt_orientation_correction.z() = -0.448182; gt_orientation_correction.w() = -0.877422;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EFRD -- miscellany"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = -0.111026; gt_orientation_correction.y() = 0.156941; gt_orientation_correction.z() = 0.109298; gt_orientation_correction.w() = 0.975242;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EFRD -- bicycles"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = 0.087531; gt_orientation_correction.y() = -0.020820; gt_orientation_correction.z() = 0.871301; gt_orientation_correction.w() = 0.482431;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3]; 

    // For dataset "EFRD -- building"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = 0.037612; gt_orientation_correction.y() = 0.011262; gt_orientation_correction.z() = 0.915428; gt_orientation_correction.w() = 0.400563;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EFRD -- staircase"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = -0.169240; gt_orientation_correction.y() = -0.043554; gt_orientation_correction.z() = 0.969101; gt_orientation_correction.w() = -0.174079;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.z() = -gt_orientation_new[0]; gt_orientation.x() = gt_orientation_new[1]; gt_orientation.y() = -gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];
    
    // For dataset "EDS -- shapes_rotation" 
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = -0.544357; gt_orientation_correction.y() = 0.463404; gt_orientation_correction.z() = -0.469128; gt_orientation_correction.w() = -0.518509;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.x() = gt_orientation_new[0]; gt_orientation.y() = gt_orientation_new[1]; gt_orientation.z() = gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EDS -- poster_rotation"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = 0.504; gt_orientation_correction.y() = -0.501; gt_orientation_correction.z() = 0.505; gt_orientation_correction.w() = 0.489;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.x() = gt_orientation_new[0]; gt_orientation.y() = gt_orientation_new[1]; gt_orientation.z() = gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EDS -- boxes_rotation"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = -0.038059; gt_orientation_correction.y() = 0.894778; gt_orientation_correction.z() = -0.444359; gt_orientation_correction.w() = -0.021669;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.x() = gt_orientation_new[0]; gt_orientation.y() = gt_orientation_new[1]; gt_orientation.z() = gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];

    // For dataset "EDS -- dynamic_rotation"
    // gt_orientation_new << gt_orientation.x(), gt_orientation.y(), gt_orientation.z(), gt_orientation.w();
    // gt_orientation_correction.x() = -0.569807; gt_orientation_correction.y() = 0.638680; gt_orientation_correction.z() = -0.387436; gt_orientation_correction.w() = -0.342492;
    // gt_orientation_new = toEulerMatrix(gt_orientation_correction) * gt_orientation_new;
    // gt_orientation.x() = gt_orientation_new[0]; gt_orientation.y() = gt_orientation_new[1]; gt_orientation.z() = gt_orientation_new[2]; gt_orientation.w() = gt_orientation_new[3];
   
    std::ofstream gt_tum("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/gt_tum.txt",std::ios::app);
    gt_tum << std::fixed << eventWarpedBack.time_stamp.back() << " " << gt_position.transpose()  << " "
                     <<  gt_orientation.x() << " " <<  gt_orientation.y() << " " <<  gt_orientation.z() << " " <<  gt_orientation.w() << "\n";      

    q = Eigen::Quaternionf(SO3(angular_position));    
    
    std::ofstream res_tum("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/res_tum.txt",std::ios::app);
    res_tum << std::fixed << eventWarpedBack.time_stamp.back() << " " << gt_position.transpose()  << " " <<  q.x() << " " <<  q.y() << " " <<  q.z() << " " <<  q.w() << "\n";      
    res_tum.close();

    euler = toEulerAngles(q);
    gt_euler = toEulerAngles(gt_orientation);

    float  mean_error_euler = (abs(euler[0] - gt_euler[0]) + abs(euler[1] - gt_euler[1]) +abs (euler[2] - gt_euler[2])) / 3.0;
    er_euler_total += mean_error_euler;
    mean_er_euler_total = er_euler_total / (cnt_iter + 1); 

    float square_error_x = (euler[0] - gt_euler[0]) * (euler[0] - gt_euler[0]);
    total_square_error_x += square_error_x;
    mean_square_error_x = total_square_error_x / (cnt_iter + 1); 
    root_mean_square_error_x = sqrt(mean_square_error_x);

    float square_error_y = (euler[1] - gt_euler[1]) * (euler[1] - gt_euler[1]);
    total_square_error_y += square_error_y;
    mean_square_error_y = total_square_error_y / (cnt_iter + 1); 
    root_mean_square_error_y = sqrt(mean_square_error_y);

    float square_error_z = (euler[2] - gt_euler[2]) * (euler[2] - gt_euler[2]);
    total_square_error_z += square_error_z;
    mean_square_error_z = total_square_error_z / (cnt_iter + 1); 
    root_mean_square_error_z = sqrt(mean_square_error_z);

    std::ofstream er_euler("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/er_euler.txt",std::ios::app);
    er_euler << std::fixed << mean_er_euler_total << " " << mean_error_euler << " " << euler[0] - gt_euler[0]  << " "
                     <<  euler[1] - gt_euler[1] << " " <<  euler[2] - gt_euler[2] << "\n";      
    er_euler.close();

    std::ofstream rmse_euler("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/rmse.txt",std::ios::app);
    rmse_euler << std::fixed << root_mean_square_error_x << "  " << root_mean_square_error_y << "  " << root_mean_square_error_z << "\n";      
    rmse_euler.close();

    cv::normalize(warped_event_image, warped_event_image_th, 0, 255,cv::NORM_MINMAX); 
    imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/curr_warp/" + std::to_string(cnt_iter)  +"_warped"+ ".jpg", warped_event_image_th);

    cv::normalize(warped_event_image_b, warped_event_image_b_th, 0, 255,cv::NORM_MINMAX); 
    imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/curr_warp_b/" + std::to_string(cnt_iter)  +"_warped"+ ".jpg", warped_event_image_b_th);

    cnt_iter ++;
}

void System::GenerateFrame(){
    eventOrg.coord_3d = eventUndistortedOrg.coord_3d;
    eventOrg.Projection(K);
    eventOrg.SetXY();
    eventOrg.DiscriminateInner(width - 1, height - 1);
    // if(is_polarity_on){
    //     GetEventImage(eventOrg, SIGNED_EVENT_IMAGE).convertTo(event_frame, CV_32FC1);
    // }
    // else{
        GetEventImage(eventOrg, DVS_RENDERER).convertTo(event_frame, CV_8UC3);
    // }
    // cv::normalize(event_frame, event_frame_th, 0, 255,cv::NORM_MINMAX); 
    imwrite("/home/julia/catkin_ws/src/VAVM-A/src/event_publisher/building/event_frame/" + std::to_string(cnt_iter)  +"_frame"+ ".jpg", event_frame);
}

void System::ComputeAngularPosition(){
    // Global events alignment
    eventAlignedPoints.Copy(eventUndistortedPrev);
    GetWarpedEventPointBack(eventUndistortedPrev, eventAlignedPoints, angular_velocity, angular_acc1, angular_acc2, false, angular_position);
    eventAlignedPoints.angular_velocity = angular_velocity_prev;
    eventAlignedPoints.angular_position = angular_position;
    eventMapPoints.push_back(eventAlignedPoints);

    if(cnt_iter == 0){
        // angular_position_init = Eigen::Vector3f::Zero();
        estimation_time_interval = 0.0;
    }

    angular_position_init = SO3add(angular_position, -angular_velocity * 0.5 * estimation_time_interval);
    angular_position_init = SO3add(angular_position_init, - 0.5 * angular_acc1 * pow(0.5 * estimation_time_interval, 2));
    angular_position_init = SO3add(angular_position_init, - (angular_velocity + angular_acc1 *  0.5 * estimation_time_interval) 
                                                                                                * 0.5 * estimation_time_interval);
    angular_position_init = SO3add(angular_position_init, - 0.5 * angular_acc2 * pow(0.5 * estimation_time_interval, 2));       

    eventUndistortedPrev = eventUndistorted;
    estimation_time_interval_prev = estimation_time_interval;
    angular_velocity_prev = angular_velocity;

    angular_position_increment = SO3(angular_position).inverse() * SO3(angular_position_init);
    angular_velocity = InvSO3(angular_position_increment.inverse() 
                            * SO3(angular_velocity + angular_acc1 * 0.5 * estimation_time_interval
                            + angular_acc2 * 0.5 * estimation_time_interval) 
                            * angular_position_increment);      

    angular_position_pprev = angular_position_prev;    
    angular_position_prev = angular_position;
}


// Compute Jacobian matrix
void System::DeriveErrAnalyticBack(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const Eigen::Vector3f &temp_ang_pos)
{
    GetWarpedEventImageBack(temp_ang_vel, temp_ang_acc1, temp_ang_acc2, temp_ang_pos);
    cv::threshold(warped_event_image, truncated_event, event_image_threshold, 255, 2); 
    cv::GaussianBlur(truncated_event, blur_image, cv::Size(5, 5), 1);
    
    cv::Sobel(blur_image, Ix, CV_32FC1, 1, 0);
    cv::Sobel(blur_image, Iy, CV_32FC1, 0, 1);

    if(rotation_estimation){
        float event_norm = 2.0f / cv::norm(truncated_event);
        truncated_sum = truncated_map + truncated_event * event_norm;
        Ix_sum = Ix_map + Ix * event_norm;
        Iy_sum = Iy_map + Iy * event_norm;
    }
    std::vector<uint16_t> e_valid = GetValidIndexFromEvent(eventWarpedBack);
    uint16_t valid_size = e_valid.size();
    
    xp.resize(valid_size);
    yp.resize(valid_size);
    zp.resize(valid_size);
    Ix_interp.resize(valid_size);
    Iy_interp.resize(valid_size);
    Ix_interp_map.resize(valid_size);
    Iy_interp_map.resize(valid_size);
    Jacobian_dt_vel.resize(valid_size);
    Jacobian_dt_acc1.resize(valid_size);
    Jacobian_dt_acc2.resize(valid_size);
    Jacobian_dt_map.resize(valid_size);
    Jacobian_acc.resize(valid_size, 3);
    Jacobian_map.resize(valid_size, 3);
    
    int16_t coord_x;
    int16_t coord_y;
    uint16_t e_valid_v_iter;
    
    for (uint16_t v_iter = 0; v_iter < valid_size; v_iter++)
    {
        
        e_valid_v_iter = e_valid[v_iter];

        coord_x = std::round(eventWarpedBack.coord(e_valid_v_iter, 0));
        coord_y = std::round(eventWarpedBack.coord(e_valid_v_iter, 1)); 

        xp(v_iter) = eventWarpedBack.coord_3d(e_valid_v_iter, 0);
        yp(v_iter) = eventWarpedBack.coord_3d(e_valid_v_iter, 1);
        zp(v_iter) = eventWarpedBack.coord_3d(e_valid_v_iter, 2);

        Ix_interp(v_iter) = Ix.at<float>(coord_y, coord_x);
        Iy_interp(v_iter) = Iy.at<float>(coord_y, coord_x);
        
        Jacobian_dt_vel(v_iter) =   ( eventWarpedBack.dt1(e_valid_v_iter) + eventWarpedBack.dt2(e_valid_v_iter))
                                                            * warped_event_image.at<float>(coord_y, coord_x);
        Jacobian_dt_acc1(v_iter) =   ( 0.5f * eventWarpedBack.dt1(e_valid_v_iter) * eventWarpedBack.dt1(e_valid_v_iter)
                                    + eventWarpedBack.dt1(e_valid_v_iter) * eventWarpedBack.dt2(e_valid_v_iter))
                                    * warped_event_image.at<float>(coord_y, coord_x);
        Jacobian_dt_acc2(v_iter) =   ( 0.5f * eventWarpedBack.dt2(e_valid_v_iter) * eventWarpedBack.dt2(e_valid_v_iter) )
                                    * warped_event_image.at<float>(coord_y, coord_x);

        if(rotation_estimation){
            Ix_interp_map(v_iter) = Ix_sum.at<float>(coord_y, coord_x);
            Iy_interp_map(v_iter) = Iy_sum.at<float>(coord_y, coord_x);

            Jacobian_dt_map(v_iter) = - truncated_sum.at<float>(coord_y, coord_x);
        }
    }

    Ix_interp *= camParam.fx;
    Iy_interp *= camParam.fy;
    

    xp_zp = xp.array() / zp.array();
    yp_zp = yp.array() / zp.array();

    Eigen::ArrayXf xx_zz = xp_zp.array() * xp_zp.array();
    Eigen::ArrayXf yy_zz = yp_zp.array() * yp_zp.array();
    Eigen::ArrayXf xy_zz = xp_zp.array() * yp_zp.array();

    Jacobian_acc.col(0) = -(Ix_interp.array() * xy_zz) - (Iy_interp.array() * (1 + yy_zz));
    Jacobian_acc.col(1) = (Ix_interp.array() * (1 + xx_zz)) + (Iy_interp.array() * xy_zz);
    Jacobian_acc.col(2) = -Ix_interp.array() * yp_zp.array() + Iy_interp.array() * xp_zp.array();
    
    if(rotation_estimation){
        Ix_interp_map *= camParam.fx;
        Iy_interp_map *= camParam.fy;

        Jacobian_map.col(0) = -(Ix_interp_map.array() * xy_zz) - (Iy_interp_map.array() * (1 + yy_zz));
        Jacobian_map.col(1) = (Ix_interp_map.array() * (1 + xx_zz)) + (Iy_interp_map.array() * xy_zz);
        Jacobian_map.col(2) = -Ix_interp_map.array() * yp_zp.array() + Iy_interp_map.array() * xp_zp.array();
    }
}

void System::DeriveErrAnalyticFor(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2)
{
    GetWarpedEventImageFor(temp_ang_vel, temp_ang_acc1, temp_ang_acc2);
    cv::threshold(warped_event_image_b, truncated_event_b, event_image_threshold, 255, 2); 
    cv::GaussianBlur(truncated_event_b, blur_image_b, cv::Size(5, 5), 1);
    
    cv::Sobel(blur_image_b, Ix_b, CV_32FC1, 1, 0);
    cv::Sobel(blur_image_b, Iy_b, CV_32FC1, 0, 1);

    std::vector<uint16_t> e_valid = GetValidIndexFromEvent(eventWarpedFor);
    uint16_t valid_size = e_valid.size();
    // std::cout << "eventWarpedBack valid size:  " << valid_size << std::endl;
    // std::cout << "eventWarpedBackPrev valid_size:  " << eprev_valid.size() << std::endl;
    // std::cout << "event_norm:  " << cv::norm(truncated_event) << "  event_norm_reciprocal:  " << event_norm << std::endl;
    // std::cout << "map_norm:  " << map_norm << "  map_norm_reciprocal:  " << map_norm_reciprocal << std::endl;

    xp.resize(valid_size);
    yp.resize(valid_size);
    zp.resize(valid_size);
    Ix_interp.resize(valid_size);
    Iy_interp.resize(valid_size);
    Jacobian_dt_vel_b.resize(valid_size);
    Jacobian_dt_acc1_b.resize(valid_size);
    Jacobian_dt_acc2_b.resize(valid_size);
    Jacobian_acc_b.resize(valid_size, 3);
    
    int16_t coord_x;
    int16_t coord_y;
    uint16_t e_valid_v_iter;
    
    for (uint16_t v_iter = 0; v_iter < valid_size; v_iter++)
    {
        
        e_valid_v_iter = e_valid[v_iter];

        coord_x = std::round(eventWarpedFor.coord(e_valid_v_iter, 0));
        coord_y = std::round(eventWarpedFor.coord(e_valid_v_iter, 1)); 

        xp(v_iter) = eventWarpedFor.coord_3d(e_valid_v_iter, 0);
        yp(v_iter) = eventWarpedFor.coord_3d(e_valid_v_iter, 1);
        zp(v_iter) = eventWarpedFor.coord_3d(e_valid_v_iter, 2);

        Ix_interp(v_iter) = Ix_b.at<float>(coord_y, coord_x);
        Iy_interp(v_iter) = Iy_b.at<float>(coord_y, coord_x);
        
        Jacobian_dt_vel_b(v_iter) =   ( eventWarpedFor.dt1p(e_valid_v_iter) + eventWarpedFor.dt2p(e_valid_v_iter))
                                                            * warped_event_image_b.at<float>(coord_y, coord_x);
        Jacobian_dt_acc1_b(v_iter) =   (eventWarpedFor.dt1(e_valid_v_iter) * eventWarpedFor.dt1p(e_valid_v_iter)
                                    + 0.5f * eventWarpedFor.dt1p(e_valid_v_iter) * eventWarpedFor.dt1p(e_valid_v_iter)
                                    + eventWarpedFor.dt2p(e_valid_v_iter) * 0.5f * eventWarpedFor.duration)
                                    * warped_event_image_b.at<float>(coord_y, coord_x);
        Jacobian_dt_acc2_b(v_iter) =   (eventWarpedFor.dt2(e_valid_v_iter) * eventWarpedFor.dt2p(e_valid_v_iter)
                                    + 0.5f * eventWarpedFor.dt2p(e_valid_v_iter) * eventWarpedFor.dt2p(e_valid_v_iter) )
                                    * warped_event_image_b.at<float>(coord_y, coord_x);
    }

    Ix_interp *= camParam.fx;
    Iy_interp *= camParam.fy;
    

    xp_zp = xp.array() / zp.array();
    yp_zp = yp.array() / zp.array();

    Eigen::ArrayXf xx_zz = xp_zp.array() * xp_zp.array();
    Eigen::ArrayXf yy_zz = yp_zp.array() * yp_zp.array();
    Eigen::ArrayXf xy_zz = xp_zp.array() * yp_zp.array();

    Jacobian_acc_b.col(0) = -(Ix_interp.array() * xy_zz) - (Iy_interp.array() * (1 + yy_zz));
    Jacobian_acc_b.col(1) = (Ix_interp.array() * (1 + xx_zz)) + (Iy_interp.array() * xy_zz);
    Jacobian_acc_b.col(2) = -Ix_interp.array() * yp_zp.array() + Iy_interp.array() * xp_zp.array();
}


// warping function for constant-velocity-model(恒速度模型)
void System::GetWarpedEventPoint(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const bool &is_map_warping /*= false*/, const Eigen::Vector3f &temp_ang_pos /*= Eigen::Vector3f::Zero()*/){
    if(eventIn.size == 0){
        std::cout << "EventIn size is zero" << std::endl;
        return;
    }
    float ang_vel_norm = temp_ang_vel.norm();
    float ang_pos_norm = temp_ang_pos.norm();
    Eigen::MatrixXf x_ang_vel_hat, x_ang_vel_hat_square;
    x_ang_vel_hat.resize(eventIn.size, 3);
    x_ang_vel_hat_square.resize(eventIn.size, 3);
    
    x_ang_vel_hat.col(0) = - temp_ang_vel(2) * eventIn.coord_3d.col(1) + temp_ang_vel(1) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(1) = + temp_ang_vel(2) * eventIn.coord_3d.col(0) - temp_ang_vel(0) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(2) = - temp_ang_vel(1) * eventIn.coord_3d.col(0) + temp_ang_vel(0) * eventIn.coord_3d.col(1);

    x_ang_vel_hat_square.col(0) = - temp_ang_vel(2) * x_ang_vel_hat.col(1) + temp_ang_vel(1) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(1) = + temp_ang_vel(2) * x_ang_vel_hat.col(0) - temp_ang_vel(0) * x_ang_vel_hat.col(2);
    x_ang_vel_hat_square.col(2) = - temp_ang_vel(1) * x_ang_vel_hat.col(0) + temp_ang_vel(0) * x_ang_vel_hat.col(1);

    // Element-wise
    // points warping via second order approximation with Rodrigue's formular
    if(is_map_warping){
        if(ang_vel_norm < 1e-8){
            eventOut.coord_3d = eventIn.coord_3d;
        }
        else{
            auto delta_t = eventIn.time_delta_reverse.array();
            eventOut.coord_3d = eventIn.coord_3d
                            + Eigen::MatrixXf(x_ang_vel_hat.array().colwise()
                            * delta_t.array()
                            + x_ang_vel_hat_square.array().colwise() 
                            * (0.5f * delta_t.array().square()) );
        }
    }
    else{
        if(ang_vel_norm < 1e-8){
            eventOut.coord_3d = eventIn.coord_3d;
        }
        else{            
            auto delta_t = eventIn.time_delta.array();
            eventOut.coord_3d = eventIn.coord_3d
                            + Eigen::MatrixXf( x_ang_vel_hat.array().colwise()
                            * delta_t.array()
                            + x_ang_vel_hat_square.array().colwise() 
                            * (0.5f * delta_t.array().square()) );
        }
    }  
    if(ang_pos_norm > 1e-8){
        eventOut.coord_3d = eventOut.coord_3d * SO3(temp_ang_pos).transpose();
    }
}


// warping function for backward warping(变速度模型)
void System::GetWarpedEventPointBack(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const bool &is_map_warping /*= false*/, const Eigen::Vector3f &temp_ang_pos /*= Eigen::Vector3f::Zero()*/){
    if(eventIn.size == 0){
        std::cout << "EventIn size is zero" << std::endl;
        return;
    }

    float ang_pos_norm = temp_ang_pos.norm();

    // Element-wise
    // points warping via first order approximation with Rodrigue's formular
    if(!is_map_warping){
        Eigen::MatrixXf x_ang_vel_hat, x_ang_acc1_hat, x_ang_acc2_hat;
        x_ang_vel_hat.resize(eventIn.size, 3);
        x_ang_acc1_hat.resize(eventIn.size, 3);
        x_ang_acc2_hat.resize(eventIn.size, 3);
        
        x_ang_vel_hat.col(0) = - temp_ang_vel(2) * eventIn.coord_3d.col(1) + temp_ang_vel(1) * eventIn.coord_3d.col(2);
        x_ang_vel_hat.col(1) = + temp_ang_vel(2) * eventIn.coord_3d.col(0) - temp_ang_vel(0) * eventIn.coord_3d.col(2);
        x_ang_vel_hat.col(2) = - temp_ang_vel(1) * eventIn.coord_3d.col(0) + temp_ang_vel(0) * eventIn.coord_3d.col(1);

        x_ang_acc1_hat.col(0) = - temp_ang_acc1(2) * eventIn.coord_3d.col(1) + temp_ang_acc1(1) * eventIn.coord_3d.col(2);
        x_ang_acc1_hat.col(1) = + temp_ang_acc1(2) * eventIn.coord_3d.col(0) - temp_ang_acc1(0) * eventIn.coord_3d.col(2);
        x_ang_acc1_hat.col(2) = - temp_ang_acc1(1) * eventIn.coord_3d.col(0) + temp_ang_acc1(0) * eventIn.coord_3d.col(1);

        x_ang_acc2_hat.col(0) = - temp_ang_acc2(2) * eventIn.coord_3d.col(1) + temp_ang_acc2(1) * eventIn.coord_3d.col(2);
        x_ang_acc2_hat.col(1) = + temp_ang_acc2(2) * eventIn.coord_3d.col(0) - temp_ang_acc2(0) * eventIn.coord_3d.col(2);
        x_ang_acc2_hat.col(2) = - temp_ang_acc2(1) * eventIn.coord_3d.col(0) + temp_ang_acc2(0) * eventIn.coord_3d.col(1);

        eventOut.coord_3d = eventIn.coord_3d
                                - Eigen::MatrixXf( x_ang_vel_hat.array().colwise() * eventIn.dt1.array()
                                + x_ang_acc1_hat.array().colwise() * eventIn.dt1.array().square() * 0.5f
                                + x_ang_vel_hat.array().colwise() * eventIn.dt2.array()
                                + x_ang_acc1_hat.array().colwise() * eventIn.dt1.array() * eventIn.dt2.array()
                                + x_ang_acc2_hat.array().colwise() * eventIn.dt2.array().square() * 0.5f);
    }
    else{
        eventOut.coord_3d = eventIn.coord_3d;
    }
    if(ang_pos_norm > 1e-8){
        eventOut.coord_3d = eventOut.coord_3d * SO3(temp_ang_pos).transpose();
        
    }

}

// warping function for forward warping(变速度模型)
void System::GetWarpedEventPointFor(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2){
    if(eventIn.size == 0){
        std::cout << "EventIn size is zero" << std::endl;
        return;
    }

    // Element-wise
    // points warping via first order approximation with Rodrigue's formular

    Eigen::MatrixXf x_ang_vel_hat, x_ang_acc1_hat, x_ang_acc2_hat;
    x_ang_vel_hat.resize(eventIn.size, 3);
    x_ang_acc1_hat.resize(eventIn.size, 3);
    x_ang_acc2_hat.resize(eventIn.size, 3);
    
    x_ang_vel_hat.col(0) = - temp_ang_vel(2) * eventIn.coord_3d.col(1) + temp_ang_vel(1) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(1) = + temp_ang_vel(2) * eventIn.coord_3d.col(0) - temp_ang_vel(0) * eventIn.coord_3d.col(2);
    x_ang_vel_hat.col(2) = - temp_ang_vel(1) * eventIn.coord_3d.col(0) + temp_ang_vel(0) * eventIn.coord_3d.col(1);

    x_ang_acc1_hat.col(0) = - temp_ang_acc1(2) * eventIn.coord_3d.col(1) + temp_ang_acc1(1) * eventIn.coord_3d.col(2);
    x_ang_acc1_hat.col(1) = + temp_ang_acc1(2) * eventIn.coord_3d.col(0) - temp_ang_acc1(0) * eventIn.coord_3d.col(2);
    x_ang_acc1_hat.col(2) = - temp_ang_acc1(1) * eventIn.coord_3d.col(0) + temp_ang_acc1(0) * eventIn.coord_3d.col(1);

    x_ang_acc2_hat.col(0) = - temp_ang_acc2(2) * eventIn.coord_3d.col(1) + temp_ang_acc2(1) * eventIn.coord_3d.col(2);
    x_ang_acc2_hat.col(1) = + temp_ang_acc2(2) * eventIn.coord_3d.col(0) - temp_ang_acc2(0) * eventIn.coord_3d.col(2);
    x_ang_acc2_hat.col(2) = - temp_ang_acc2(1) * eventIn.coord_3d.col(0) + temp_ang_acc2(0) * eventIn.coord_3d.col(1);

    eventOut.coord_3d = eventIn.coord_3d
                            + Eigen::MatrixXf( x_ang_vel_hat.array().colwise() * eventIn.dt1p.array()
                            + x_ang_acc1_hat.array().colwise() * eventIn.dt1.array() * eventIn.dt1p.array()
                            + x_ang_acc1_hat.array().colwise() * eventIn.dt1p.array().square() * 0.5f 
                            + x_ang_vel_hat.array().colwise() * eventIn.dt2p.array()
                            + x_ang_acc1_hat.array().colwise() * eventIn.dt2p.array() * eventIn.duration * 0.5f
                            + x_ang_acc2_hat.array().colwise() * eventIn.dt2.array() * eventIn.dt2p.array()
                            + x_ang_acc2_hat.array().colwise() * eventIn.dt2p.array().square() * 0.5f);
    
}

std::vector<uint16_t> System::GetValidIndexFromEvent(const EventBundle & event){
    std::vector<uint16_t> valid;
    uint16_t valid_counter = 0;
    for (auto e_iter = event.isInner.begin(); e_iter != event.isInner.end(); e_iter++){
        if(*e_iter){
            valid.push_back(valid_counter);
        }
        valid_counter++;
    }
    return valid;
}

void System::GetWarpedEventImageBack(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const Eigen::Vector3f &temp_ang_pos){
    GetWarpedEventPointBack(eventUndistorted, eventWarpedBack, temp_ang_vel, temp_ang_acc1, temp_ang_acc2, false, temp_ang_pos);
    eventWarpedBack.Projection(K);
    eventWarpedBack.SetXY();
    eventWarpedBack.DiscriminateInner(width - 1, height - 1);
    if(is_polarity_on){
        GetEventImage(eventWarpedBack, SIGNED_EVENT_IMAGE).convertTo(warped_event_image, CV_32FC1);
    }
    else{
        GetEventImage(eventWarpedBack, UNSIGNED_EVENT_IMAGE).convertTo(warped_event_image, CV_32FC1);
    }
}

void System::GetWarpedEventImageFor(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2){
    GetWarpedEventPointFor(eventUndistorted, eventWarpedFor, temp_ang_vel, temp_ang_acc1, temp_ang_acc2);
    eventWarpedFor.Projection(K);
    eventWarpedFor.SetXY();
    eventWarpedFor.DiscriminateInner(width - 1, height - 1);
    if(is_polarity_on){
        GetEventImage(eventWarpedFor, SIGNED_EVENT_IMAGE).convertTo(warped_event_image_b, CV_32FC1);
    }
    else{
        GetEventImage(eventWarpedFor, UNSIGNED_EVENT_IMAGE).convertTo(warped_event_image_b, CV_32FC1);
    }
}

void System::GetWarpedEventMap(const Eigen::Vector3f &temp_ang_pos){
    if( eventMap.size == 0 ){
        return;
    }
    GetWarpedEventPointBack(eventMap, eventWarpedBackMap, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), true, -temp_ang_pos);
    eventWarpedBackMap.Projection(K);
    eventWarpedBackMap.SetXY();
    eventWarpedBackMap.DiscriminateInner(width - 1, height - 1, map_sampling_rate);
    GetEventImage(eventWarpedBackMap, UNSIGNED_EVENT_IMAGE, false).convertTo(warped_map_image, CV_32FC1);
}


cv::Mat System::GetWarpedImageNew(const PlotOption &option /*= UNSIGNED_EVENT_IMAGE*/){
    cv::Mat result = cv::Mat(height, width, CV_32FC3);
    result = cv::Scalar(0, 0, 0);
    // eventWarpedBack.Projection(K);
    // eventWarpedBack.SetXY();
    // eventWarpedBack.DiscriminateInner(width - 1, height - 1);
    GetEventImage(eventWarpedBack, option, true).convertTo(result, CV_32FC3);
    return result;
}

cv::Mat System::GetFrameNew(const PlotOption &option /*= UNSIGNED_EVENT_IMAGE*/){
    cv::Mat result = cv::Mat(height, width, CV_32FC3);
    result = cv::Scalar(0, 0, 0);
    eventOrg.Projection(K);
    eventOrg.SetXY();
    eventOrg.DiscriminateInner(width - 1, height - 1);
    GetEventImage(eventOrg, option, true).convertTo(result, CV_32FC3);
    return result;
}