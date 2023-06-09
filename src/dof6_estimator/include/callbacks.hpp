#pragma once 


// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/EventDepthArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>


// std 
#include <vector> 
#include <string>


// self 
#include "system.hpp"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(System* sys): system(sys){}
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    System* system; 
    // ros::Time begin_time; 
}; 


class EventGrabber
{
public:
    EventGrabber(System* sys): system(sys){}
    
    void GrabEvent(const dvs_msgs::EventDepthArrayConstPtr& msg);
    System* system; 
}; 

class PoseGrabber
{
public:
    PoseGrabber(System* sys): system(sys) {
        // que_valid = false;
        gt_velocity_file = fstream("/home/hxt/Desktop/data/gt_theta_velocity.txt", ios::out);
        // gt_velocity_file_quat = fstream("/home/hxt/Desktop/data/evo_data/gt_theta_velocity.txt", ios::out);
    }
    void GrabPose(const geometry_msgs::PoseStampedConstPtr& msg);
    System* system; 


    fstream gt_velocity_file; 
    // fstream gt_velocity_file_quat;    //evo esti

    // bool que_valid;
    queue<PoseData> que_last_poseData;
    // ros::Time begin_time; 
};

