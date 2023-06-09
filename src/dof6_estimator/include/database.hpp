#pragma once 

// std 
#include <vector>
#include <string> 
#include <iostream>
#include <fstream>

// ros 
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std; 


struct EventData
{
    double time_stamp; 
    vector<dvs_msgs::Event> event;
};

struct ImageData
{
    double time_stamp; 
    cv::Mat image;
    uint32_t seq; 
};

struct PoseData
{
    double time_stamp;          // starts from 0s
    ros::Time time_stamp_ros;   // ros time 
    uint32_t seq;
    Eigen::Quaterniond quat;    // quanterion
    Eigen::Vector3d pose;       // xyz
};

struct CameraPara
{

    CameraPara(); 
    CameraPara(string filename); 
    double fx;
    double fy;
    double cx;
    double cy;
    double k1, k2, p1, p2, k3;
    // double rd1;
    // double rd2;

    int width, height, height_map, width_map;

    cv::Mat cameraMatrix, distCoeffs ;
    
    Eigen::Matrix3d eg_cameraMatrix, eg_MapMatrix;
    

};

enum PlotOption
{
    U16C3_EVNET_IMAGE_COLOR,
    U16C1_EVNET_IMAGE,
    U8C1_EVNET_IMAGE,
    S16C1_EVNET_IMAGE,
    TIME_SURFACE,
    F32C1_EVENT_COUNT,
};


/**
* \brief a set of local events, stores as Eigen::Matrix2xf.
*/
struct EventBundle{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EventBundle();
    EventBundle(const EventBundle& eb); 
    ~EventBundle();

    // core opearte
    void Append(std::vector<dvs_msgs::Event>& vec_eventData);
    void Append(std::vector<dvs_msgs::Event>& vec_eventData, std::vector<double>& vec_eventDepth);
    void CopySize(const EventBundle& eb); 

    void Clear(); 
    void DiscriminateInner(int widht, int height);

    // image process 
    // GetEventImage() // options with signed or color 
    void InverseProjection(Eigen::Matrix3d& K);
    void InverseProjection(Eigen::Matrix3d& K, Eigen::Matrix3Xd& raw_coord_3d);  // given depth 
    void Projection(Eigen::Matrix3d& K);


    // events in eigen form used as 3d porjection    
    Eigen::Matrix2Xd coord;                   // row, col = [2,pixels], used for Eigen rotation 
    Eigen::Matrix3Xd coord_3d;                // row, col = [3,pixels], used for Eigen rotation 

    // relative time of event
    Eigen::VectorXd time_delta;               // later is positive
    Eigen::VectorXd time_delta_rev;           // currently not used 

    // the estimate para of local events
    Eigen::Vector3d angular_velocity,         // angleAxis current velocity, used to sharp local events
                    angular_position;         // angleAxis current pos, warp cur camera to world coord.

    // Eigen::Matrix3d rotation_cur2ref;      // from camera to world s

    // events in vector form, used as data storage
    // double abs_tstamp;                     // receiving time at ROS system
    
    ros::Time first_tstamp, last_tstamp;      // event time in ros system. 
    // vector<double> x, y;                      // original event coor  used for opencv
    Eigen::VectorXf polar;                       //  0, 1 event polar
    Eigen::VectorXf isInner;                     //  0, 1 indicating the event is inner 
    size_t size; 

};
