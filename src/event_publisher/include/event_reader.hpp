#pragma once 

// std
#include <fstream>
#include <sstream>
#include <vector> 
#include <string> 

// ros 
#include <ros/ros.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>


// third party 
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;


class Event_reader
{

public:
    Event_reader(std::string yaml="", 
        ros::Publisher* event_array_pub = nullptr, 
        ros::Publisher* event_image_pub = nullptr);

    bool read(int event_size = 0, double event_interval = 0);
    void publish();
    void render();    
    double sleep_rate;


    // for txt 
    bool acquire(dvs_msgs::EventArrayPtr ptr);
    bool acquire(dvs_msgs::EventArrayPtr ptr, std::vector<double>& vec_depth);


private:

    ros::Publisher* event_array_pub_; 
    ros::Publisher* event_image_pub_; 

    std::vector<dvs_msgs::Event> eventData;  // single event 
    int eventData_counter; 

    int height, width;  // event image size
    
    int event_bundle_size;    // maximum size of event vector 
    
    int read_max_lines; 
    int read_start_lines;

    double delta_time; // 
    double curr_time;  

    int store2txt; 
    int using_fixed_time; 
    double fixed_interval;

    size_t count_pos;  // start pos of each publish

    dvs_msgs::EventArrayPtr msg_ptr; // for publish

    cv_bridge::CvImage event_image; 

    std::ifstream openFile; // read file 
    int count_liens; 
};







