
#include "numerics.hpp"

#include <iostream> 
using namespace std;
/**
* \brief convert x[1x3] to asym matrix [3x3].
* \param[in] x .
*/
Eigen::Matrix3d hat(const Eigen::Vector3d &x)
{
    Eigen::Matrix3d x_hat; 
    x_hat << 0, -x(2), x(1), 
            x(2), 0, -x(0),
            -x(1), x(0), 0;
    return x_hat; 
}


/**
* \brief from asy matrix [3x3] to vector [1x3].
*/
Eigen::Vector3d unhat(const Eigen::Matrix3d &x_hat)
{
    Eigen::Vector3d x;
    x << x_hat(2,1), x_hat(0,2), x_hat(1,0);
    return x;
}

/**
* \brief from 3 AngleAxis theta to rotation matrix.
* \param rpy rotation angle in rad format. .
*/
Eigen::Matrix3d SO3(const Eigen::Vector3d &x)
{
    // cout << " hat(x) \n" <<  hat(x).transpose() << endl;
    // cout << " exp " << hat(x).exp().transpose() << endl;
    // cout << " matrix  \n " << Eigen::Matrix3d(hat(x).exp()) << endl;

    return Eigen::Matrix3d(hat(x).exp());
}

/**
* \brief Constructor.
* \param x_hat. 
*/
Eigen::Vector3d InvSO3(const Eigen::Matrix3d &R)
{

    Eigen::Matrix3d temp = R.log();

    // cout << " R.log() \n" <<  R.log() << endl;
    // cout << " unhat(R.log()) \n" <<  unhat(R.log()) << endl;
    // cout << " unhat(()) \n" <<  unhat((R.log())) << endl;
    // cout << "R.log xyz" <<  temp(2,1)<< "," <<  temp(0,2)<<"," <<  temp(1,0) << endl;
    // return unhat(R.log());
    return unhat(temp);

}   

/**
* \brief from 3 theta to rotation matrix.
* \param rpy rotation angle in rad format. .
*/
Eigen::Vector3d SO3add(const Eigen::Vector3d x1, const Eigen::Vector3d x2, bool circle)
{
    if (circle && (x1 + x2).norm() > M_PI)
    {
        return x1 + x2;
    }
    else
    {
        // cout << " SO3(x1) \n" <<  SO3(x1) << endl;
        // cout << " SO3(x1) * 2\n " << SO3(x1) * SO3(x2) << endl;
        // cout << " return  \n " << InvSO3(SO3(x1) * SO3(x2)) << endl;
        return InvSO3(SO3(x1) * SO3(x2));
    }
}


/**
* \brief from quaternion to euler anglers， return rpy(xyz) theta.
*/
Eigen::Vector3d toEulerAngles(Eigen::Quaterniond q){
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

    Eigen::Vector3d v3d(angles.roll, angles.pitch, angles.yaw);
    return v3d;
}


Eigen::Quaterniond ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;

    
    Eigen::Quaterniond q(w,x,y,z);
    // std::cout <<"Quaterniond: "<< q.coeffs().transpose() << ", norm: "<<  q.coeffs().norm() << std::endl;
    return q;
}


/**
* \brief .
* \param type is opencv type CV_float or 
*/
double getVar(cv::Mat& image, int& count_nozero, int type)
{
    // std::cout << image.channels() << std::endl;
    
    // cv::Mat grayimage; 
    // // cv::normalize(image, grayimage,0,255,cv::NORM_MINMAX, CV_8UC3);
    // image.convertTo(grayimage,CV_8UC3);
    // cv::cvtColor(grayimage, grayimage, cv::COLOR_BGR2GRAY, 1);
    // cv::imshow("image", image) ;
    // cv::imshow("normalize", grayimage) ;
    // cv::imshow("grayimage", grayimage) ;
    // cv::waitKey(0);

    assert(image.channels() == 1);
    // assert(image.type() == CV_16UC1);

    double mean = 0; 
    count_nozero = 1;
    for(int x = 0; x < image.cols; x++)
        for(int y = 0; y < image.rows; y++)
    {
        double value = 1000;
        if(type == CV_16U)
            value = image.at<unsigned short>(y,x);
        if(type == CV_32F)
            value = image.at<float>(y,x);

        if(value>0)
        {
            mean += value; 
            count_nozero++; 
        }
    }

    mean /= count_nozero; 

    double var = 0;
    for(int x = 0; x < image.cols; x++)
    for(int y = 0; y < image.rows; y++)
    {

        double value = 1000;
        if(type == CV_16U)
            value = image.at<unsigned short>(y,x);
        if(type == CV_32F)
            value = image.at<float>(y,x);
        if(value>0)
        {
            var += std::pow(value-mean,2);
        }
    }
    var /= count_nozero;
    return var;
}