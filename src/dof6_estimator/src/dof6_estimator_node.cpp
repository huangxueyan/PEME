// std
#include <iostream>
#include <string> 

// third party 

// ros 
#include <ros/ros.h>
#include "callbacks.hpp"
#include "system.hpp"
#include "event_reader.hpp"
// #include <event_publisher/event_reade.hpp>


using namespace std; 


// rosbag play -r 0.1 ~/Documents/rosbag/Mono-rosbag/slider_depth.bag

int main(int argc, char** argv)
{

    ros::init(argc, argv, "rotation_estimator");
    ros::start();

    ros::NodeHandle nh("~");

    string yaml;  // system configration 
    nh.param<string>("yaml", yaml, "");

    System* sys = new System(yaml); 
    if(!sys->file_opened())
    {
        cout << "failed opening file " << endl;
        return 0;
    }

    /* Event ROS version */
    // ImageGrabber imageGrabber(&sys); 
    // ros::Subscriber image_sub = nh.subscribe("/dvs/image_raw", 10, &ImageGrabber::GrabImage, &imageGrabber);
    // // PoseGrabber poseGrabber(sys);
    // // ros::Subscriber pose_sub = nh.subscribe("/optitrack/davis", 10, &PoseGrabber::GrabPose, &poseGrabber);
    // EventGrabber eventGrabber(sys);
    // ros::Subscriber event_sub = nh.subscribe("/events_depth", 10, &EventGrabber::GrabEvent, &eventGrabber);
    // ros::spin();
    
    /** Event TXT version */ 
    // ros::Time t1 = ros::Time::now(); 
    Event_reader event_reader(yaml); 
    while (true)
    {
        // read data 
        dvs_msgs::EventArrayPtr msg_ptr = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
        std::vector<double> vec_depth;
        event_reader.acquire(msg_ptr, vec_depth);

        if(msg_ptr==nullptr || msg_ptr->events.empty() )
        {
            cout << "wrong reading events, msgptr==null" << int(msg_ptr==nullptr) << "empty events " << int(msg_ptr->events.empty()) << endl;
            break;
        }

        sys->pushEventData(msg_ptr->events, vec_depth);
        // cout << "success reveive" << endl;
        // break;
    }
    // ros::Time t2 = ros::Time::now(); 
    cout << "total optimize time " << sys->total_evaluate_time << endl;

    cout << "shutdown rotation estimator" << endl;
    return 0;
}

