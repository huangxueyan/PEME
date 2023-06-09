#include "callbacks.hpp"
#include "database.hpp"

ros::Time begin_time = ros::Time(0); 

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    if(begin_time == ros::Time(0))
    {
        begin_time =  msg->header.stamp; 
        system->setBeginTime(msg->header.stamp);
    } 

    static int last_img_seq = msg->header.seq; 

    cv_bridge::CvImageConstPtr cv_ptr; 
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg,"mono8"); // gray 8char
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("receive image error: %s", e.what());
        return ;
    }

    // fixme not need to constru a new imageData obj 
    ImageData ImageData; 
    ImageData.image = cv_ptr->image.clone(); 
    ImageData.seq = msg->header.seq; 
    ImageData.time_stamp = (cv_ptr->header.stamp - begin_time).toSec();
    cout << "receiving image t: " << ImageData.time_stamp<< endl;

    system->pushimageData(ImageData);
}


void EventGrabber::GrabEvent(const dvs_msgs::EventArrayConstPtr& msg) 
{
    if(msg->events.empty() || msg->events.size()<1000) return; 

    if(begin_time == ros::Time(0)) 
    {
        begin_time = msg->events[0].ts; 
        system->setBeginTime(msg->events[0].ts);
        cout << "begin time " << msg->events[0].ts.sec << "." <<msg->events[0].ts.nsec  << endl;
    }


    // not need to copy eventdata obj
    // EventData eventdata; 
    // eventdata.event = msg->events; // vector<dvsmsg::event> 
    double time_stamp = (msg->events[0].ts - begin_time).toSec(); 
    double delta_time = (msg->events.back().ts-msg->events.front().ts).toSec();
    cout<<"receiving events " << msg->events.size() <<", time: " << msg->events[0].ts.toSec()<<", delta time " << delta_time <<endl;

    // system->que_vec_eventData_mutex.lock();
    system->pushEventData(msg->events);
    // system->que_vec_eventData_mutex.unlock();

}


void PoseGrabber::GrabPose(const geometry_msgs::PoseStampedConstPtr& msg)
{

    if(begin_time == ros::Time(0))
    {
        begin_time =  msg->header.stamp; 
        system->setBeginTime(msg->header.stamp);
    } 
    
    // not need to copy eventdata obj
    PoseData poseData; 
    poseData.time_stamp = (msg->header.stamp-begin_time).toSec(); 
    poseData.time_stamp_ros = msg->header.stamp; 
    
    // vector<geometry::pose> 
    poseData.pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    // input w,x,y,z. output and store: x,y,z,w
    poseData.quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
    
    cout<<"receiving poses t: " << poseData.time_stamp << "， ros: " << std::to_string(poseData.time_stamp_ros.toSec()) << endl;
    cout << "----(xyz)(wxyz)" << poseData.pose.transpose() <<  poseData.quat.coeffs().transpose() << endl;


    if(que_last_poseData.size() < 10)
    {
        que_last_poseData.push(poseData);
    }
    else
    {
        PoseData last_poseData = que_last_poseData.front(); 
        que_last_poseData.pop(); 
        
        Eigen::Quaterniond t1_t2 =  last_poseData.quat.inverse() * poseData.quat;
        // last_poseData.quat.angularDistance(poseData.quat);

        Eigen::Vector3d velocity =  toEulerAngles(t1_t2) * 1 / (poseData.time_stamp_ros - last_poseData.time_stamp_ros).toSec();
        
        // get middle timestamp of event bundle 
        uint32_t second = last_poseData.time_stamp_ros.sec;
        uint32_t nsecond;
        if(last_poseData.time_stamp_ros.nsec > poseData.time_stamp_ros.nsec)
        {
            // example: first 1.12, last 1.15;
            nsecond = last_poseData.time_stamp_ros.nsec +  (poseData.time_stamp_ros.nsec-last_poseData.time_stamp_ros.nsec)/2;
        }
        else
        {
            uint32_t delta_to_second =  uint32_t(1000000000) - last_poseData.time_stamp_ros.nsec;
            // example: first 1.96, last 1.01;
            if(delta_to_second > poseData.time_stamp_ros.nsec)
            {
                nsecond = last_poseData.time_stamp_ros.nsec + (delta_to_second+poseData.time_stamp_ros.nsec) / 2;
            }
            // example: first 1.99, last 1.02;
            else
            {
                nsecond = (poseData.time_stamp_ros.nsec - delta_to_second) / 2;
                second++;
            }
        }

        gt_velocity_file << poseData.time_stamp_ros.sec << " " << poseData.time_stamp_ros.nsec <<" " << velocity.transpose() << endl;

        // evo part
        // // todo from quat to agnles axis and divide by time, get velocity
        // Eigen::AngleAxisd angle_axis =  Eigen::AngleAxisd(t1_t2);
        // // cout << "angle axis " << angle_axis.axis() << "," << angle_axis.angle() << endl;
        // angle_axis.angle() = angle_axis.angle() /  (poseData.time_stamp_ros - last_poseData.time_stamp_ros).toSec();
        // // cout << "angle axis " << angle_axis.axis() << "," << angle_axis.angle() << endl;
        // Eigen::Quaterniond q = Eigen::Quaterniond(angle_axis);
        
        // gt_velocity_file_quat << poseData.time_stamp_ros << " 0 0 0 " << q.x() << " "
        // << q.y() << " "  << q.z() << " " << q.w() << endl;

        last_poseData = poseData;
        que_last_poseData.push(poseData);
    }


    // system->pushPoseData(poseData);
}