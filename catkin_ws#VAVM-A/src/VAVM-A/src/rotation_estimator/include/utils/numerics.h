#ifndef NUMERICS_H
#define NUMERICS_H

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <math.h>

Eigen::Matrix3f hat(const Eigen::Vector3f &x);

Eigen::Vector3f unhat(const Eigen::Matrix3f &x_hat);

Eigen::Matrix3f SO3(const Eigen::Vector3f &x);

Eigen::Vector3f InvSO3(const Eigen::Matrix3f &x_hat);

Eigen::Vector3f SO3add(const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const bool &is_cyclic = false);

struct MyEulerAngles {
    float roll, pitch, yaw; };

Eigen::Vector3f toEulerAngles(Eigen::Quaternionf q);

Eigen::Vector3f toEulerAnglesReverse(Eigen::Quaternionf q);

Eigen::Matrix4f toEulerMatrix(Eigen::Quaternionf q);
#endif // NUMERICS_H