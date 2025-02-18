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

#include "Database.h"
EventBundle::EventBundle(){
    size = 0;
    coord.resize(size, 2);
    coord_3d.resize(size, 3);
}

EventBundle::EventBundle(const EventBundle& temp){
    coord = temp.coord;
    coord_3d = temp.coord_3d;

    angular_velocity = temp.angular_velocity;
    angular_position = temp.angular_position;

    time_delta = temp.time_delta;
    time_delta_reverse = temp.time_delta_reverse;

    dt1 = temp.dt1;
    dt1p = temp.dt1p;
    dt2 = temp.dt2;
    dt2p = temp.dt2p;

    time_stamp = temp.time_stamp;
    polarity = temp.polarity;
    x = temp.x;
    y = temp.y;
    isInner = temp.isInner;
    size = temp.size;

    duration = temp.duration;
    split_time = temp.split_time;
}
EventBundle::~EventBundle(){

}

void EventBundle::Append(const EventBundle &ref){
    
    size += ref.size;
    
    Eigen::MatrixXf temp_coord = coord;
    Eigen::MatrixXf temp_coord_3d = coord_3d;
    Eigen::VectorXf temp_time_delta = time_delta;
    Eigen::VectorXf temp_time_delta_reverse = time_delta_reverse;

    coord.resize(size, 2);
    coord_3d.resize(size, 3);

    time_delta.resize(size);
    time_delta_reverse.resize(size);

    coord << temp_coord, ref.coord;
    coord_3d << temp_coord_3d, ref.coord_3d;
    time_delta << temp_time_delta, ref.time_delta;
    time_delta_reverse << temp_time_delta_reverse, ref.time_delta_reverse;

    time_stamp.insert(time_stamp.end(), ref.time_stamp.begin(), ref.time_stamp.end());
    polarity.insert(polarity.end(), ref.polarity.begin(), ref.polarity.end());


}

void EventBundle::Clear(){
    size = 0;

    coord.resize(size, 2);
    coord_3d.resize(size, 3);

    time_delta.resize(size);
    time_delta_reverse.resize(size);

    dt1.resize(size);
    dt1p.resize(size);
    dt2.resize(size);
    dt2p.resize(size);

    angular_velocity = Eigen::Vector3f::Zero();
    angular_position = Eigen::Vector3f::Zero();

    time_stamp.clear();
    polarity.clear();
    x.clear();
    y.clear();
    isInner.clear();

    duration = 0.0;
    split_time = 0.0;
}

void EventBundle::Copy(const EventBundle &ref){
    time_delta = ref.time_delta;
    time_delta_reverse = ref.time_delta_reverse;

    dt1 = ref.dt1;
    dt1p = ref.dt1p;
    dt2 = ref.dt2;
    dt2p = ref.dt2p;

    time_stamp = ref.time_stamp;
    polarity = ref.polarity;
    size = ref.size;

    angular_velocity = ref.angular_velocity;
    angular_position = ref.angular_position;

    coord.resize(size,2);
    coord_3d.resize(size,3);

    duration = ref.duration;
    split_time = ref.split_time;
}

void EventBundle::SetCoord(){
    size = x.size();
    if(size != 0){
        Eigen::Map<Eigen::VectorXf> eigen_x(x.data(), size);
        Eigen::Map<Eigen::VectorXf> eigen_y(y.data(), size);
        coord = Eigen::MatrixXf(size, 2);
        coord << eigen_x, eigen_y;
    }
}

void EventBundle::erase(size_t iter){
    time_stamp.erase(time_stamp.begin() + iter);
    polarity.erase(polarity.begin() + iter);
    x.erase(x.begin() + iter);
    y.erase(y.begin() + iter);
}

// Project coord_3d into coord
void EventBundle::Projection(Eigen::Matrix3f K){
    coord.col(0) = coord_3d.col(0).array()/coord_3d.col(2).array() * K(0, 0) + K(0, 2); 
    coord.col(1) = coord_3d.col(1).array()/coord_3d.col(2).array() * K(1, 1) + K(1, 2);
    coord = coord.array().round();
}

// Backproject coord into coord_3d
void EventBundle::InverseProjection(Eigen::Matrix3f K){
    Eigen::Map<Eigen::VectorXd> time_delta_(time_stamp.data(), size);
    time_delta = (time_stamp.front()- time_delta_.array()).cast<float>();
    time_delta_reverse = (time_stamp.back()- time_delta_.array()).cast<float>();
    coord_3d.col(0) = (coord.col(0).array() - K(0, 2))/K(0, 0);
    coord_3d.col(1) = (coord.col(1).array() - K(1, 2))/K(1, 1);
    coord_3d.col(2) = Eigen::MatrixXf::Ones(size, 1);
}

// Discriminate whether the coord is inside the image window or not
void EventBundle::DiscriminateInner(int width, int height, int map_sampling_rate /*= 1*/)
{
    isInner.resize(size);
    if (x.size() != size)
    {
        SetXY();
    }
    for (uint32_t pts_iter = 0; pts_iter < size; pts_iter++)
    {
        if (x[pts_iter] <= 0 || x[pts_iter] >= width || y[pts_iter] <= 0 || y[pts_iter] >= height || pts_iter % map_sampling_rate != 0)
        {
            isInner[pts_iter] = false;
        }
        else
        {
            isInner[pts_iter] = true;
        }
    }
}

// Synchronize the coord and the x, y
void EventBundle::SetXY(){
    x = std::vector<float>(coord.col(0).data(), coord.col(0).data() + size);
    y = std::vector<float>(coord.col(1).data(), coord.col(1).data() + size);
}

void EventBundle::Split(){
    duration = time_stamp.back() - time_stamp.front();
    split_time = time_stamp.front() + (time_stamp.back() - time_stamp.front()) * 0.5;

    int itr;
    for (itr = 0; itr < time_stamp.size(); itr++){
        if(time_stamp[itr] > split_time){
            break;
        }
    }
    std::vector<double> dt1_tmp;
    dt1_tmp.insert(dt1_tmp.begin(), time_stamp.begin(), time_stamp.begin()+itr);
    dt1_tmp.insert(dt1_tmp.begin()+itr, time_stamp.size()-itr, split_time);
    Eigen::Map<Eigen::VectorXd> dt1_tmp_map(dt1_tmp.data(), size);
    dt1 = (dt1_tmp_map.array() - time_stamp.front()).cast<float>();

    std::vector<double> dt1p_tmp;
    dt1p_tmp.insert(dt1p_tmp.begin(), time_stamp.begin(), time_stamp.begin()+itr);
    dt1p_tmp.insert(dt1p_tmp.begin()+itr, time_stamp.size()-itr, split_time);
    Eigen::Map<Eigen::VectorXd> dt1p_tmp_map(dt1p_tmp.data(), size);
    dt1p = (split_time - dt1p_tmp_map.array()).cast<float>();

    std::vector<double> dt2_tmp;
    dt2_tmp.insert(dt2_tmp.begin(), itr, split_time);
    dt2_tmp.insert(dt2_tmp.begin()+itr, time_stamp.begin()+itr, time_stamp.end());
    Eigen::Map<Eigen::VectorXd> dt2_tmp_map(dt2_tmp.data(), size);
    dt2 = (dt2_tmp_map.array() - split_time).cast<float>();

    std::vector<double> dt2p_tmp;
    dt2p_tmp.insert(dt2p_tmp.begin(), itr, time_stamp.back() - duration * 0.5);
    dt2p_tmp.insert(dt2p_tmp.begin()+itr, time_stamp.begin()+itr, time_stamp.end());
    Eigen::Map<Eigen::VectorXd> dt2p_tmp_map(dt2p_tmp.data(), size);
    dt2p = (time_stamp.back() - dt2p_tmp_map.array()).cast<float>();
}  