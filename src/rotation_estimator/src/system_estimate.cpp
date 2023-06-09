
#include "system.hpp"
#include "numerics.hpp"
#include <sophus/so3.hpp>

using namespace std;


/**
* \brief given angular veloity(t1->t2), warp local event bundle become shaper
*/
cv::Mat System::getWarpedEventImage(const Eigen::Vector3d & cur_ang_vel, EventBundle& event_out,  const PlotOption& option, bool ref_t1, float timerange)
{
    // cout << "get warpped event image " << endl;
    // cout << "eventUndistorted.coord.cols() " << event_undis_Bundle.coord.cols() << endl;
    /* warp local events become sharper */
    ros::Time t1, t2, t3;

    event_out.CopySize(event_undis_Bundle);
    // t1 = ros::Time::now(); 

    getWarpedEventPoints(event_undis_Bundle, event_out, cur_ang_vel, ref_t1, timerange); 
    // t2= ros::Time::now(); 
    
    event_out.Projection(camera.eg_cameraMatrix);

    event_out.DiscriminateInner(camera.width, camera.height);
    // t3 = ros::Time::now(); 


    // cout << "   getWarpedEventPoints time " << (t2-t1).toSec() << endl;
    // cout << "   DiscriminateInner time " << (t3-t2).toSec() << endl;
    return getImageFromBundle(event_out, option, timerange);
}

/**
* \brief given angular veloity, warp local event bundle(t2) to the reference time(t1)
    using kim RAL21, eqation(11), since the ratation angle is ratively small
* \param cur_ang_vel angleAxis/delta_t from t1->t2, if multiply minus time, it becomes t2->t1. (AngleAxis inverse only add minus) 
* \param cur_ang_pos from t2->t0. default set (0,0,0) default, so the output is not rotated. 
* \param delta_time all events warp this time, if delta_time<0, warp to t1. 
*/
void System::getWarpedEventPoints(const EventBundle& eventIn, EventBundle& eventOut, 
    const Eigen::Vector3d& cur_ang_vel,  bool ref_t1, float timerange)
{
    // cout << "projecting " << endl;
    // the theta of rotation axis
    float ang_vel_norm = cur_ang_vel.norm(); 
    
    eventOut.CopySize(eventIn);
    if(ang_vel_norm < 1e-8) 
    {
        // cout << "  small angle vec " << ang_vel_norm/3.14 * 180 << " degree /s" << endl;
        eventOut.coord_3d = eventIn.coord_3d ;
    }
    else if (timerange > 0.9)
    {   
        // cout << "using whole warp "  <<endl;
        Eigen::VectorXd vec_delta_time = eventBundle.time_delta;  // positive 
        if(ref_t1) vec_delta_time = eventBundle.time_delta.array() - eventBundle.time_delta(eventBundle.size-1);   // negative 

        // taylor
            Eigen::Matrix3Xd ang_vel_hat_mul_x, ang_vel_hat_sqr_mul_x;  /** equation 11 of kim */ 
            ang_vel_hat_mul_x.resize(3,eventIn.size);     // row, col 
            ang_vel_hat_sqr_mul_x.resize(3,eventIn.size); 
            ang_vel_hat_mul_x.row(0) = -cur_ang_vel(2)*eventIn.coord_3d.row(1) + cur_ang_vel(1)*eventIn.coord_3d.row(2);
            ang_vel_hat_mul_x.row(1) =  cur_ang_vel(2)*eventIn.coord_3d.row(0) - cur_ang_vel(0)*eventIn.coord_3d.row(2);
            ang_vel_hat_mul_x.row(2) = -cur_ang_vel(1)*eventIn.coord_3d.row(0) + cur_ang_vel(0)*eventIn.coord_3d.row(1);

            ang_vel_hat_sqr_mul_x.row(0) = -cur_ang_vel(2)*ang_vel_hat_mul_x.row(1) + cur_ang_vel(1)*ang_vel_hat_mul_x.row(2);
            ang_vel_hat_sqr_mul_x.row(1) =  cur_ang_vel(2)*ang_vel_hat_mul_x.row(0) - cur_ang_vel(0)*ang_vel_hat_mul_x.row(2);
            ang_vel_hat_sqr_mul_x.row(2) = -cur_ang_vel(1)*ang_vel_hat_mul_x.row(0) + cur_ang_vel(0)*ang_vel_hat_mul_x.row(1);

            // first order version 
            {
                // eventOut.coord_3d = eventIn.coord_3d
                //                             + Eigen::MatrixXd( 
                //                                 ang_vel_hat_mul_x.array().rowwise() 
                //                                 * (vec_delta_time.transpose().array()));
            }
                

            // kim second order version;
            {
                eventOut.coord_3d = eventIn.coord_3d
                                            + Eigen::MatrixXd( 
                                                ang_vel_hat_mul_x.array().rowwise() 
                                                * (vec_delta_time.transpose().array())
                                                + ang_vel_hat_sqr_mul_x.array().rowwise() 
                                                * (0.5f * vec_delta_time.transpose().array().square()) );
            }

        // cout << "usingg est" << cur_ang_vel.transpose() << endl;
        // cout << "original  " << eventIn.coord_3d.topLeftCorner(3,5)<< endl;
        // cout << "ang_vel_hat_mul_x " << ang_vel_hat_mul_x.topLeftCorner(3,5)<< endl;
        // cout << "delta time " << vec_delta_time.topRows(5).transpose() << endl;
        // cout << "final \n" << eventOut.coord_3d.topLeftCorner(3,5) <<  endl;

        // rodrigues version wiki
            // Eigen::Matrix<double,3,1> axis = cur_ang_vel.normalized();
            // Eigen::VectorXd angle_vec = vec_delta_time * ang_vel_norm ;

            // Eigen::VectorXd cos_angle_vec = angle_vec.array().cos();
            // Eigen::VectorXd sin_angle_vec = angle_vec.array().sin();

            // Eigen::Matrix3Xd first = eventIn.coord_3d.array().rowwise() * cos_angle_vec.transpose().array(); 
            // Eigen::Matrix3Xd second = (-eventIn.coord_3d.array().colwise().cross(axis)).array().rowwise() * sin_angle_vec.transpose().array();
            // Eigen::VectorXd third1 = axis.transpose() * eventIn.coord_3d;
            // Eigen::VectorXd third2 = third1.array() * (1-cos_angle_vec.array()).array();;
            // Eigen::Matrix3Xd third = axis * third2.transpose();
            // eventOut.coord_3d = first + second + third; 

        // cout << "last \n " << eventOut.coord_3d.bottomRightCorner(3,5) <<  endl;

    }
    else  // warp events within given timerange 
    {
        int n_num = int(eventIn.size * timerange);
        Eigen::VectorXd vec_delta_time = eventBundle.time_delta.topRows(n_num);  // positive 
        if(ref_t1) vec_delta_time = eventBundle.time_delta.topLeftCorner(1,n_num).array() - eventBundle.time_delta(eventBundle.size-1);   // negative 


        // cout << "warp num " << n_num << ", size " << eventOut.coord_3d.topLeftCorner(3,n_num).size() <<  endl;

        // taylor
            Eigen::Matrix<double, 3, -1> temp_eventIn = eventIn.coord_3d.topLeftCorner(3,n_num);
            Eigen::Matrix3Xd ang_vel_hat_mul_x, ang_vel_hat_sqr_mul_x;  /** equation 11 of kim */ 
            ang_vel_hat_mul_x.resize(3, n_num);     // row, col 
            ang_vel_hat_sqr_mul_x.resize(3, n_num); 
            ang_vel_hat_mul_x.row(0) = -cur_ang_vel(2)*temp_eventIn.row(1) + cur_ang_vel(1)*temp_eventIn.row(2);
            ang_vel_hat_mul_x.row(1) =  cur_ang_vel(2)*temp_eventIn.row(0) - cur_ang_vel(0)*temp_eventIn.row(2);
            ang_vel_hat_mul_x.row(2) = -cur_ang_vel(1)*temp_eventIn.row(0) + cur_ang_vel(0)*temp_eventIn.row(1);


        // cout << "vec_delta_time " << vec_delta_time.rows() << endl;
        // cout << "temp_eventIn " << temp_eventIn.cols() << endl;
        // cout << "ang_vel_hat_mul_x " << ang_vel_hat_mul_x.cols() << endl;

            // first order version 
            {
                eventOut.coord_3d.topLeftCorner(3,n_num) = temp_eventIn
                                            + Eigen::MatrixXd( 
                                                ang_vel_hat_mul_x.array().rowwise() 
                                                * (vec_delta_time.transpose().array()));
                
                eventOut.coord_3d.bottomRightCorner(1,eventIn.size-n_num).array() = 1;   // set z axis to 1
        // cout << "eventOut \n " << eventOut.coord_3d.block(0,1000,3,5) <<  endl;


            }
                

            // kim second order version;
            // {
            //     ang_vel_hat_sqr_mul_x.row(0) = -cur_ang_vel(2)*ang_vel_hat_mul_x.row(1) + cur_ang_vel(1)*ang_vel_hat_mul_x.row(2);
            //     ang_vel_hat_sqr_mul_x.row(1) =  cur_ang_vel(2)*ang_vel_hat_mul_x.row(0) - cur_ang_vel(0)*ang_vel_hat_mul_x.row(2);
            //     ang_vel_hat_sqr_mul_x.row(2) = -cur_ang_vel(1)*ang_vel_hat_mul_x.row(0) + cur_ang_vel(0)*ang_vel_hat_mul_x.row(1);
            //     eventOut.coord_3d = eventIn.coord_3d
            //                                 + Eigen::MatrixXd( 
            //                                     ang_vel_hat_mul_x.array().rowwise() 
            //                                     * (vec_delta_time.transpose().array())
            //                                     + ang_vel_hat_sqr_mul_x.array().rowwise() 
            //                                     * (0.5f * vec_delta_time.transpose().array().square()) );
            // }

    }

    // cout << "sucess getWarpedEventPoints" << endl;
}


/**
* \brief given start and end ratio (0~1), and samples_count, return noise free sampled events. 
* \param vec_sampled_idx 
* \param samples_count 
*/
void System::getSampledVec(vector<int>& vec_sampled_idx, int samples_count, double sample_start, double sample_end)
{
    cv::RNG rng(int(ros::Time::now().nsec));
    // get samples, filtered out noise  
    for(int i=0; i< samples_count;)
    {
        int sample_idx = rng.uniform(int(event_undis_Bundle.size*sample_start), int(event_undis_Bundle.size*sample_end));

        // check valid 8 neighbor hood existed 
        int sampled_x = event_undis_Bundle.coord.col(sample_idx)[0];
        int sampled_y = event_undis_Bundle.coord.col(sample_idx)[1];
        if(sampled_x >= 239  ||  sampled_x < 1 || sampled_y >= 179 || sampled_y < 1 ) 
        {
            // cout << "x, y" << sampled_x << "," << sampled_y << endl;
            continue;
        }

        int count = 0;
        for(int j=-1; j<2; j++)
            for(int k=-1; k<2; k++)
            {
                count += (  curr_undis_event_image.at<cv::Vec3f>(sampled_y+j,sampled_x+k)[0] + 
                            curr_undis_event_image.at<cv::Vec3f>(sampled_y+j,sampled_x+k)[1] +
                            curr_undis_event_image.at<cv::Vec3f>(sampled_y+j,sampled_x+k)[2] ) > 0;
                // count += (  curr_undis_event_image.at<float>(sampled_y+j,sampled_x+k) != default_value)
            }

        // valid denoised
        if(count > yaml_denoise_num)  // TODO 
        {
            vec_sampled_idx.push_back(sample_idx);
            i++;
        }
    }
    
}



/**
* \brief given event_warpped_Bundle and rotation matrix, 
* \param vec_Bundle_Maps,  
* \param curr_map_image, output 
*/
void System::getMapImage()
{
    cout << "mapping global" << endl;
    
    /* warp current event to t0 using gt */
    Eigen::Matrix3d R_b2f = get_global_rotation_b2f(0,vec_gt_poseData.size()-1);
    Eigen::AngleAxisd angAxis_b2f(R_b2f);

    /* warp current event to t0 using estimated data */


    event_Map_Bundle.CopySize(event_warpped_Bundle);
    getWarpedEventPoints(event_warpped_Bundle,event_Map_Bundle, angAxis_b2f.axis()*angAxis_b2f.angle());
    event_Map_Bundle.Projection(camera.eg_MapMatrix);
    event_Map_Bundle.DiscriminateInner(camera.width_map, camera.height_map);
    event_Map_Bundle.angular_position = angAxis_b2f.axis() * angAxis_b2f.angle(); 
    vec_Bundle_Maps.push_back(event_Map_Bundle);

    // cout << "test " << vec_Bundle_Maps[0].coord.topLeftCorner(2,5) << endl;



    /* get map from all events to t0 */
    cv::Mat temp_img; 
    curr_map_image.setTo(0);

    int start_idx = (vec_Bundle_Maps.size()-3) > 0 ? vec_Bundle_Maps.size()-3 : 0; 
    for(size_t i=start_idx; i<vec_Bundle_Maps.size(); i++)
    {
        // get 2d image 
        getImageFromBundle(vec_Bundle_Maps[i], PlotOption::U16C1_EVNET_IMAGE).convertTo(temp_img, CV_32F);
        // cout << "temp_img.size(), " << temp_img.size() << "type " << temp_img.type() << endl;
        
        curr_map_image += temp_img;

    }

    curr_map_image.convertTo(curr_map_image, CV_32F);

    // cout << "mask type " << mask.type() << "size " <<mask.size() <<  ", mask 5,5 : \n" << mask(cv::Range(1,6),cv::Range(1,6)) << endl;
    // cout << "curr_map_image type " << curr_map_image.type()<< ", curr_map_image size" << curr_map_image.size() << endl;
    // curr_map_image.setTo(255, mask);

    cout << "  get mapping sucess " << endl;
}


