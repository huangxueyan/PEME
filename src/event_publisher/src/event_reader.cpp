#include "event_reader.hpp"   


Event_reader::Event_reader(std::string yaml, 
    ros::Publisher* event_array_pub, ros::Publisher* event_image_pub)
    :event_array_pub_(event_array_pub), event_image_pub_(event_image_pub)
{


    cv::FileStorage fSettings(yaml, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error(std::string("Could not open file: ") + yaml);
    }


    count_pos = 0;
    read_max_lines = fSettings["read_max_lines"];
    read_start_lines = fSettings["read_start_lines"];
    event_bundle_size = fSettings["Event.bundle_size"];
    using_fixed_time =  fSettings["using_fixed_time"];
    fixed_interval = fSettings["fixed_interval"];
    sleep_rate = fSettings["sleep_rate"];

    store2txt = fSettings["store2txt"];

    // read(fSettings["groundtruth_dir"]);

    string dir = fSettings["groundtruth_dir"]; 
    cout << "opening " << dir << endl;
    openFile = std::ifstream(dir.c_str(),std::ios_base::in);
    if(!openFile.is_open())
    {
        std::cout << "file not opened " << std::endl;
        return;
    }

    count_liens = 0;

};


/**
* \brief read file.
* \param return bool indicating read success.
*/
bool Event_reader::read(int target_size, double target_interval)
{
    // std::vector<dvs_msgs::Event> vec_events; 
    // std::ifstream openFile(dir.c_str(),std::ios_base::in);
    // if(!openFile.is_open())
    // {
    //     std::cout << "file not opened " << std::endl;
    //     return;
    // }

    string line, token; 
    std::vector<std::string> vToken;

    int start_line = count_liens;
    double start_time = eventData.empty() ? 0: eventData.back().ts.toSec();

    dvs_msgs::Event msg;
    while(getline(openFile, line)) // already opened in initial function 
    {
        if(count_liens++ < read_start_lines) continue;
        if(count_liens > read_max_lines)
            return false;
            
        std::stringstream ss(line); 
        while (getline(ss, token, ' '))
            vToken.push_back(token);

        if(vToken.size() == 4)
        {
            char* temp; 
            msg.ts = ros::Time(std::strtod(vToken[0].c_str(), &temp));
            msg.x = uint16_t(std::strtod(vToken[1].c_str(),&temp));
            msg.y = uint16_t(std::strtod(vToken[2].c_str(), &temp));
            msg.polarity = uint8_t(std::strtod(vToken[3].c_str(), &temp));
            eventData.emplace_back(msg);
        }
        vToken.clear();


        if(target_size > 0 && count_liens-start_line > target_size) break;
        if(target_interval > 0 && eventData.back().ts.toSec()-start_time > target_interval) break;
    }

    return true;

}


/** \brief 
    \param ptr given ptr, target_size>0 means fixed size reader, target_interval>0 means fixed time reader
*/
bool Event_reader::acquire(dvs_msgs::EventArrayPtr ptr)
{
    // cout <<"reading events" << endl;
    // ptr->events.clear();
    // ptr->events.reserve(event_bundle_size);

    
    int target_size = 0;
    double target_interval = 0;
    if(using_fixed_time==0)
    {
        target_size = event_bundle_size;    
        // cout << "using fixed size " << event_bundle_size << endl;
    }
    else
    {
        target_interval = fixed_interval;
        cout << "using fixed time " << fixed_interval << endl;
    }

    dvs_msgs::Event msg;

    // reading 
    string line, token; 
    std::vector<std::string> vToken;

    int start_line = max(count_liens, read_start_lines);
    // double start_time = eventData.empty() ? 0: eventData.back().ts.toSec();

    while(getline(openFile, line)) // already opened in initial function 
    {
        if(count_liens++ < read_start_lines) continue;
        if(count_liens > read_max_lines) break;
            
        std::stringstream ss(line); 
        while (getline(ss, token, ' '))
            vToken.push_back(token);

        if(vToken.size() == 4)
        {
            char* temp; 
            msg.ts = ros::Time(std::strtod(vToken[0].c_str(), &temp));
            msg.x = uint16_t(std::strtod(vToken[1].c_str(),&temp));
            msg.y = uint16_t(std::strtod(vToken[2].c_str(), &temp));
            msg.polarity = uint8_t(std::strtod(vToken[3].c_str(), &temp));
            // eventData.emplace_back(msg);
        }
        vToken.clear();

        ptr->events.push_back(msg);

        if(target_size > 0 && count_liens-start_line >= target_size) break;
        if(target_interval > 0 && (ptr->events.back().ts-ptr->events.front().ts).toSec() > target_interval) break;
    }

    // cout <<"sucesss events " << endl;

    return true;

}

/** \brief 
    \param ptr given ptr, target_size>0 means fixed size reader, target_interval>0 means fixed time reader
*/
bool Event_reader::acquire(dvs_msgs::EventArrayPtr ptr, std::vector<double>& vec_depth)
{
    // cout <<"reading events" << endl;
    // ptr->events.clear();
    // ptr->events.reserve(event_bundle_size);

    // cout << "new acquire " << endl;
    
    int target_size = 0;
    double target_interval = 0;
    if(using_fixed_time==0)
    {
        target_size = event_bundle_size;    
        // cout << "using fixed size " << event_bundle_size << endl;
    }
    else
    {
        target_interval = fixed_interval;
        cout << "using fixed time " << fixed_interval << endl;
    }

    dvs_msgs::Event msg;

    // reading 
    string line, token; 
    std::vector<std::string> vToken;

    int start_line = max(count_liens, read_start_lines);
    // double start_time = eventData.empty() ? 0: eventData.back().ts.toSec();

    while(getline(openFile, line)) // already opened in initial function 
    {
        if(count_liens++ < read_start_lines) continue;
        if(count_liens > read_max_lines) break;
            
        std::stringstream ss(line); 
        while (getline(ss, token, ' '))
            vToken.push_back(token);

        if(vToken.size() == 5)
        {
            char* temp; 
            msg.ts = ros::Time(std::strtod(vToken[0].c_str(), &temp));
            msg.x = uint16_t(std::strtod(vToken[1].c_str(),&temp));
            msg.y = uint16_t(std::strtod(vToken[2].c_str(), &temp));
            msg.polarity = uint8_t(std::strtod(vToken[4].c_str(), &temp));
            // eventData.emplace_back(msg);
            vec_depth.push_back(std::strtod(vToken[3].c_str(), &temp));
            // cout << msg.ts <<" " << msg.x <<" " << msg.y <<" " << msg.polarity << endl;
        }
        vToken.clear();

        ptr->events.push_back(msg);

        if(target_size > 0 && count_liens-start_line >= target_size) break;
        if(target_interval > 0 && (ptr->events.back().ts-ptr->events.front().ts).toSec() > target_interval) break;
    }

    // cout <<"sucesss events " << endl;

    return true;

}

void Event_reader::publish()
{
    
    if (!msg_ptr)
    {
        msg_ptr = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
    }

    // fixed number
    if(!using_fixed_time)
    {
        read(event_bundle_size, 0);  // read fixed size 
        int current_size = 0;

        fstream est_velocity_file;

        if(store2txt)
        {
            est_velocity_file = fstream("/home/hxy/Desktop/data/seg_events/seg_start_" + 
                    std::to_string(count_pos) + ".txt", ios::out);
        }

        while(count_pos < eventData.size())
        {
            msg_ptr->events.push_back(eventData[count_pos]);
            msg_ptr->header.seq = count_pos;
            msg_ptr->header.stamp = eventData[count_pos].ts;

            if(store2txt)
            {
                est_velocity_file << eventData[count_pos].ts
                                    << " " <<eventData[count_pos].x  
                                    << " " <<eventData[count_pos].y   
                                    << " " << (eventData[count_pos].polarity>0) << endl;
            }

            current_size++;
            count_pos++;
        }

        est_velocity_file.close(); 

    } 
    else // fixed interval 
    {
        read(0, fixed_interval);  // read fixed time 
        double interval = 0;
        while(count_pos < eventData.size())
        {
            msg_ptr->events.push_back(eventData[count_pos]);
            msg_ptr->header.seq = count_pos;
            msg_ptr->header.stamp = eventData[count_pos].ts;

            interval = (msg_ptr->events.back().ts - msg_ptr->events.front().ts).toSec();
            count_pos++;
        }

    }

    if(!msg_ptr->events.empty())
    {
        msg_ptr->height = 180;
        msg_ptr->width = 240;
        event_array_pub_->publish(msg_ptr);
        cout << "current time " << msg_ptr->events[0].ts << 
            ", msg.size " << msg_ptr->events.size() << ", interval " <<(msg_ptr->events.back().ts - msg_ptr->events.front().ts).toSec() << endl;
    }
    msg_ptr.reset(); 
}




/**
* \brief accumulate events to event frame.
*/
void Event_reader::render()
{

}    