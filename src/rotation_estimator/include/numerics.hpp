#pragma once 

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <math.h>

#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

Eigen::Matrix3d hat(const Eigen::Vector3d &x); 
Eigen::Vector3d unhat(const Eigen::Matrix3d &x); 
Eigen::Matrix3d SO3(const Eigen::Vector3d &x); 
Eigen::Vector3d InvSO3(const Eigen::Matrix3d &x_hat);
Eigen::Vector3d SO3add(const Eigen::Vector3d x, const Eigen::Vector3d y, bool cycle); 


struct MyEulerAngles {
    double roll, pitch, yaw; };

Eigen::Vector3d toEulerAngles(Eigen::Quaterniond q);

Eigen::Quaterniond ToQuaternion(double yaw, double pitch, double roll);


double getVar(cv::Mat& image, int& nonZero, int type);