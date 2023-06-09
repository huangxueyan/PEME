// std
#include <iostream>
#include <string> 

// third party 

// ros 
#include <ros/ros.h>
// #include "callbacks.hpp"
#include "system.hpp"
#include "event_reader.hpp"
// #include <event_publisher/event_reade.hpp>


using namespace std; 

class EventGrabber
{
public:
    EventGrabber(System* sys) : system(sys) {}

    void GrabEvent(const dvs_msgs::EventArrayConstPtr& msg);

    System* system;
};

void EventGrabber::GrabEvent(const dvs_msgs::EventArrayConstPtr& msg)
{
    system->pushEventData(msg->events);
}

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
        // return 0;
    }
    
    ros::Time t1, t2; 
    t1 = ros::Time::now();
    /* Event ROS version */
        // EventGrabber eventGrabber(sys);
        // ros::Subscriber event_sub = nh.subscribe("/dvs/events", 10, &EventGrabber::GrabEvent, &eventGrabber);
        // ros::spin();
    
    /** Event TXT version */ 
    double total_event_acquire_time = 0;
    Event_reader event_reader(yaml); 
    while (true)
    {
        // read data 
        // ros::Time t1, t2; 
        ros::Time t3 = ros::Time::now();
            dvs_msgs::EventArrayPtr msg_ptr = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
            event_reader.acquire(msg_ptr);
        // t2 = ros::Time::now();
        total_event_acquire_time += (ros::Time::now() - t3).toSec();
        // cout << "event_reader.acquire time " << (t2-t1).toSec() << endl;  // 0.50643


        if(msg_ptr==nullptr || msg_ptr->events.empty() )
        {
            cout << "wrong reading events, msgptr==null" << int(msg_ptr==nullptr) << "empty events " << int(msg_ptr->events.empty()) << endl;
            break;
        }


        // t1 = ros::Time::now();
            sys->pushEventData(msg_ptr->events);
        // t2 = ros::Time::now();
        // cout << "sys pushEventData" << (t2-t1).toSec() << endl;  // 0.00691187 s

        // cout << "success reveive" << endl;
        // break;
    }
    
    
    double total_program_runtime = (ros::Time::now() - t1).toSec(); 
    cout << " total program time "<< total_program_runtime << endl;
    cout << " total total_event_acquire_time "<< total_event_acquire_time << endl;
    // cout << " total read events time "<< sys->total_readevents_time << endl;
    cout << " total create event bundle time "<< sys->total_eventbundle_time << endl;
    cout << " total evaluate time "<< sys->total_evaluate_time << endl;
    cout << " total warpevents time "<< sys->total_warpevents_time << endl; 
    cout << " total timesurface time "<< sys->total_timesurface_time << endl; 
    cout << " total ceres time "<< sys->total_ceres_time << endl;
    cout << " total undistort time "<< sys->total_undistort_time << endl;
    cout << " total visual time "<< sys->total_visual_time << endl;

    cout << "shutdown rotation estimator" << endl;
    return 0;
}

