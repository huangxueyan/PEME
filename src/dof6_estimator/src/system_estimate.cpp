
#include "system.hpp"
#include "numerics.hpp"
#include <sophus/so3.hpp>

using namespace std;


/**
* \brief given angular veloity(t1->t2), warp local event bundle become shaper
*/
cv::Mat System::getWarpedEventImage(const Eigen::Vector3d & cur_ang_vel, const Eigen::Vector3d& cur_trans_vel, EventBundle& event_out, const PlotOption& option, bool ref_t1)
{
    // cout << "get warpped event image " << endl;
    // cout << "eventUndistorted.coord.cols() " << event_undis_Bundle.coord.cols() << endl;
    /* warp local events become sharper */
    event_out.CopySize(event_undis_Bundle);
    getWarpedEventPoints(event_undis_Bundle, event_out, cur_ang_vel,cur_trans_vel, -1, ref_t1); 
    event_out.Projection(camera.eg_cameraMatrix);
    event_out.DiscriminateInner(camera.width, camera.height);
    // getImageFromBundle(event_out, option, false).convertTo(curr_warpped_event_image, CV_32F);

    return getImageFromBundle(event_out, option, false);

    // testing 

    // getWarpedEventPoints(event_undis_Bundle, event_out, Eigen::Vector3d(0,10,0)); 
    // event_out.Projection(camera.eg_cameraMatrix);
    // event_out.DiscriminateInner(camera.width, camera.height);
    // getImageFromBundle(event_out, option, false).convertTo(y_img, CV_32F);
    // getWarpedEventPoints(event_undis_Bundle, event_out, Eigen::Vector3d(0,5,0)); 
    // event_out.Projection(camera.eg_cameraMatrix);
    // event_out.DiscriminateInner(camera.width, camera.height);
    // getImageFromBundle(event_out, option, false).convertTo(y5_img, CV_32F);

    // cv::imshow("x ", x_img);
    // cv::imshow("y ", y_img);
    // cv::imshow("z ", z_img);
    // cv::imshow("x 5", x5_img);
    // cv::imshow("y 5", y5_img);
    // cv::imshow("z 5", z5_img);
    // cv::waitKey(0);

    // cout << "  success get warpped event image " << endl;
}

/**
* \brief given angular veloity, warp local event bundle(t2) to the reference time(t1)
    using kim RAL21, eqation(11), since the ratation angle is ratively small
* \param cur_ang_vel angleAxis/delta_t from t1->t2, if multiply minus time, it becomes t2->t1. (AngleAxis inverse only add minus) 
* \param cur_ang_pos from t2->t0. default set (0,0,0) default, so the output is not rotated. 
* \param delta_time all events warp this time, if delta_time<0, warp to t1. 
*/
void System::getWarpedEventPoints(const EventBundle& eventIn, EventBundle& eventOut, 
    const Eigen::Vector3d& cur_ang_vel, const Eigen::Vector3d& cur_trans_vel, double delta_time,  bool ref_t1 )
{
    // cout << "projecting " << endl;
    // the theta of rotation axis
    float ang_vel_norm = cur_ang_vel.norm(); 

    Eigen::Matrix3Xd ang_vel_hat_mul_x, ang_vel_hat_sqr_mul_x;
    
    
    /** equation 11 of kim */ 
    // ang_vel_hat_mul_x.resize(3,eventIn.size);     // row, col 
    // ang_vel_hat_sqr_mul_x.resize(3,eventIn.size); 
    // ang_vel_hat_mul_x.row(0) = -cur_ang_vel(2)*eventIn.coord_3d.row(1) + cur_ang_vel(1)*eventIn.coord_3d.row(2);
    // ang_vel_hat_mul_x.row(1) =  cur_ang_vel(2)*eventIn.coord_3d.row(0) - cur_ang_vel(0)*eventIn.coord_3d.row(2);
    // ang_vel_hat_mul_x.row(2) = -cur_ang_vel(1)*eventIn.coord_3d.row(0) + cur_ang_vel(0)*eventIn.coord_3d.row(1);


    // ang_vel_hat_sqr_mul_x.row(0) = -cur_ang_vel(2)*ang_vel_hat_mul_x.row(1) + cur_ang_vel(1)*ang_vel_hat_mul_x.row(2);
    // ang_vel_hat_sqr_mul_x.row(1) =  cur_ang_vel(2)*ang_vel_hat_mul_x.row(0) - cur_ang_vel(0)*ang_vel_hat_mul_x.row(2);
    // ang_vel_hat_sqr_mul_x.row(2) = -cur_ang_vel(1)*ang_vel_hat_mul_x.row(0) + cur_ang_vel(0)*ang_vel_hat_mul_x.row(1);


    eventOut.CopySize(eventIn);
    if(false && ang_vel_norm < 1e-8)  // TODO always computes
    {
        // cout << "  small angle vec " << ang_vel_norm/3.14 * 180 << " degree /s" << endl;
        eventOut.coord_3d = eventIn.coord_3d ;
    }
    else
    {   
        Eigen::VectorXd vec_delta_time = eventBundle.time_delta;  
        if(ref_t1) vec_delta_time = eventBundle.time_delta.array() - eventBundle.time_delta(eventBundle.size-1);  
                
        // kim second order version;
        {
            // eventOut.coord_3d = eventIn.coord_3d
            //                             + Eigen::MatrixXd( 
            //                                 ang_vel_hat_mul_x.array().rowwise() 
            //                                 * (-vec_delta_time.transpose().array()));
            //                                 + ang_vel_hat_sqr_mul_x.array().rowwise() 
            //                                 * (0.5f * vec_delta_time.transpose().array().square()) );
            // cout << "usingg est" << cur_ang_vel.transpose() << endl;
            // cout << "original  " << eventIn.coord_3d.topLeftCorner(3,5)<< endl;
            // cout << "ang_vel_hat_mul_x " << ang_vel_hat_mul_x.topLeftCorner(3,5)<< endl;
            // cout << "delta time " << vec_delta_time.topRows(5).transpose() << endl;
            // cout << "final \n" << eventOut.coord_3d.topLeftCorner(3,5) <<  endl;
        }

        // exactly 
        if(ref_t1 == false) // warp to t1 
        {
            // TODO: projection, for unit rotation and zero translation remain the same points
            Eigen::Matrix<double,3,1> axis = cur_ang_vel.normalized();
            Eigen::Matrix<double,3,3> skew_m; 
            skew_m << 0.0, -axis(2), axis(1), axis(2), 0.0, -axis(0), -axis(1), axis(0), 0.0; 

            Eigen::Matrix<double,3,3> rotation;
            for(int i=0; i<eventOut.coord_3d.cols(); i++)
            {
                double theta = ang_vel_norm * vec_delta_time(i);
                rotation = Eigen::Matrix<double,3,3>::Identity() + sin(theta)*skew_m + (1.0-cos(theta))*skew_m*skew_m;
                // eventOut.coord_3d.col(i) = rotation.transpose() * (eventIn.coord_3d.col(i).array() - (cur_trans_vel*vec_delta_time(i)).array()).matrix();    
                eventOut.coord_3d.col(i) = rotation * eventIn.coord_3d.col(i) + cur_trans_vel*vec_delta_time(i);    
            }
        }
        else  // warp to t2, p2 = R12^T * p1 - R12^T * Trans12
        {
            Eigen::Matrix<double,3,1> axis = cur_ang_vel.normalized();
            Eigen::Matrix<double,3,3> skew_m; 
            skew_m << 0.0, -axis(2), axis(1), axis(2), 0.0, -axis(0), -axis(1), axis(0), 0.0; 

            Eigen::Matrix<double,3,3> rotation;
            for(int i=0; i<eventOut.coord_3d.cols(); i++)
            {
                double theta = ang_vel_norm * vec_delta_time(i);
                rotation = Eigen::Matrix<double,3,3>::Identity() + sin(theta)*skew_m + (1.0-cos(theta))*skew_m*skew_m;
                // eventOut.coord_3d.col(i) = rotation.transpose() * (eventIn.coord_3d.col(i).array() - (cur_trans_vel*vec_delta_time(i)).array()).matrix();    
                eventOut.coord_3d.col(i) = rotation * eventIn.coord_3d.col(i) + rotation * cur_trans_vel*vec_delta_time(i);    
            }
        }

        // cout << "last \n " << eventOut.coord_3d.bottomRightCorner(3,5) <<  endl;

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
        if(sampled_x >= camera.width-2  ||  sampled_x < 1 || sampled_y >= camera.height-2 || sampled_y < 1 ) 
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



