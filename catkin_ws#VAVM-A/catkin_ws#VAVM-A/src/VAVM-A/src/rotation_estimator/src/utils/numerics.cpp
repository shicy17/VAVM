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

#include "utils/numerics.h"

Eigen::Matrix3f hat(const Eigen::Vector3f &x)
{
    Eigen::Matrix3f x_hat;
    x_hat << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
    return x_hat;
}

Eigen::Vector3f unhat(const Eigen::Matrix3f &x_hat)
{
    Eigen::Vector3f x;
    x << x_hat(2, 1), x_hat(0, 2), x_hat(1, 0);
    return x;
}

Eigen::Matrix3f SO3(const Eigen::Vector3f &x)
{
    return Eigen::Matrix3f(hat(x).exp());
}

Eigen::Vector3f InvSO3(const Eigen::Matrix3f &x_hat)
{
    return Eigen::Vector3f(unhat((x_hat.log())));
}

Eigen::Vector3f SO3add(const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const bool &is_cyclic)
{ 
    //if (is_cyclic && (x1 + x2).norm() > M_PI)   
    if (is_cyclic && (x1 + x2).norm() > M_PI)   
    {
        return x1 + x2;
    }
    else
    {
        return InvSO3(SO3(x1) * SO3(x2));
    }
}

/**
* \brief from quaternion to euler anglers， return rpy(xyz) theta.
*/
Eigen::Vector3f toEulerAngles(Eigen::Quaternionf q){
    MyEulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    Eigen::Vector3f v3d(angles.roll, angles.pitch, angles.yaw);
    return v3d;
}

Eigen::Vector3f toEulerAnglesReverse(Eigen::Quaternionf q){
    MyEulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.z() + q.y() * q.x());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.z() * q.z());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.x() - q.z() * q.y());
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.y() + q.x() * q.z());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.x() * q.x());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    Eigen::Vector3f v3d(angles.roll, angles.pitch, angles.yaw);
    return v3d;
}

Eigen::Matrix4f toEulerMatrix(Eigen::Quaternionf q){
    Eigen::Matrix4f q_mat;
    q_mat << q.w(), - q.z(), q.y(), q.x(),
                        q.z(), q.w(), - q.x(), q.y(),
                        - q.y(), q.x(), q.w(), q.z(),
                        - q.x(), - q.y(), - q.z(), q.w();
    return q_mat;
}