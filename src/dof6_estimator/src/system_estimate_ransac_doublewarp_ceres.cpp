
#include "system.hpp"
#include "numerics.hpp"
#include <sophus/so3.hpp>
#include <algorithm>
#include <ceres/cubic_interpolation.h>
using namespace std;



/**
* \brief used for ceres to implement CM methods, automatie version. .
*/
struct ResidualCostFunction
{

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ResidualCostFunction(
        const Eigen::Vector3d& points, 
        const double delta_time_early, 
        const double delta_time_later, 
        const Eigen::Matrix3d& K, 
        ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>* interpolator_early_ptr,
        ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>* interpolator_later_ptr,
        Eigen::Matrix<double, 6, 1>& last_est_var):
            points_(points), delta_time_early_(delta_time_early), delta_time_later_(delta_time_later), 
            intrisic_(K), interpolator_early_ptr_(interpolator_early_ptr), interpolator_later_ptr_(interpolator_later_ptr),
            last_est_var_(last_est_var)
    {

    }

    // operator 
    template<typename T> 
    bool operator()(const T* const ag_v_ang, T* residual) const  // ag_v_ang 8 vars of homography 
    {
        // 6dof: r + v 
        Eigen::Matrix<T, 3, 1> points_T;
        points_T(0) = T(points_(0));
        points_T(1) = T(points_(1));
        points_T(2) = T(points_(2));
        
        Eigen::Matrix<T, 3, 1> delta_points_T, delta_second_points_T;
        Eigen::Matrix<T, 3, 1> points_early_T, points_later_T;
        Eigen::Matrix<T, 2, 1> points_2D_early_T, points_2D_later_T;

        // rotation part 
        // {
            Eigen::Matrix<T, 3, 1> ag_early = {ag_v_ang[0], ag_v_ang[1], ag_v_ang[2]};
            ag_early = ag_early * T(delta_time_early_);         
            Eigen::Matrix<T, 3, 1> ag_later = {ag_v_ang[0], ag_v_ang[1], ag_v_ang[2]};
            ag_later = ag_later * T(delta_time_later_);  // negative time 

            Eigen::Matrix<T, 3, 3> Rotation_early;
            Eigen::Matrix<T, 3, 3> Rotation_later;
            ceres::AngleAxisToRotationMatrix(&ag_early(0), &Rotation_early(0));
            ceres::AngleAxisToRotationMatrix(&ag_later(0), &Rotation_later(0));
        // }
        
        Eigen::Matrix<T, 3, 1> vel; // translation velocity
        vel(0) = ag_v_ang[3];
        vel(1) = ag_v_ang[4];
        vel(2) = ag_v_ang[5];
        
        // translation part new using matrix  
        {
            points_early_T = Rotation_early * points_T + T(delta_time_early_) * vel;
            points_later_T = Rotation_later * points_T + T(delta_time_later_) * Rotation_later  * vel;
        }

        points_2D_early_T(0) = points_early_T(0)/points_early_T(2)*T(intrisic_(0,0)) + T(intrisic_(0,2));
        points_2D_early_T(1) = points_early_T(1)/points_early_T(2)*T(intrisic_(1,1)) + T(intrisic_(1,2));
        
        points_2D_later_T(0) = points_later_T(0)/points_later_T(2)*T(intrisic_(0,0)) + T(intrisic_(0,2));
        points_2D_later_T(1) = points_later_T(1)/points_later_T(2)*T(intrisic_(1,1)) + T(intrisic_(1,2));

        // cout << "points "<< points(0) << ", "<< points(1) << endl;

        /* ceres interpolate version  */
        {
            // TODO double warp 
            T early_loss = T(0), later_loss = T(0);
            interpolator_early_ptr_->Evaluate(points_2D_early_T(1), points_2D_early_T(0), &early_loss);
            interpolator_later_ptr_->Evaluate(points_2D_later_T(1), points_2D_later_T(0), &later_loss);

            // cout << "intered " << early_loss << ", later " << later_loss << endl;
            // residual[0] = early_loss; 
            T norm_residual = T(0);
            for(int i=0;i<6;i++)
            {
                norm_residual += ceres::pow(T(last_est_var_(i,0)) - ag_v_ang[i], 2);
            }
            residual[0] = early_loss + later_loss + T(0.01) * norm_residual; 
        }

        return true;
    }

    // make ceres costfunction 
    static ceres::CostFunction* Create(
        const Eigen::Vector3d& points, 
        const double delta_time_early, 
        const double delta_time_later, 
        const Eigen::Matrix3d& K,
        ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>* interpolator_early_ptr,
        ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>* interpolator_later_ptr,
        Eigen::Matrix<double, 6, 1>& last_est_var)
        {
            return new ceres::AutoDiffCostFunction<ResidualCostFunction,1, 6>(
                new ResidualCostFunction(points, delta_time_early, delta_time_later, K, 
                    interpolator_early_ptr, interpolator_later_ptr, last_est_var));
        }

    // inputs     
    Eigen::Vector3d points_;
    double delta_time_early_, delta_time_later_;
    Eigen::Matrix3d intrisic_; 

    ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>> *interpolator_early_ptr_;
    ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>> *interpolator_later_ptr_;
    Eigen::Matrix<double, 6, 1> last_est_var_;
    // Eigen::Matrix3Xd ang_vel_hat_mul_x, ang_vel_hat_sqr_mul_x;
};




/**
* \brief using time as distance, self boosting, using ceres as optimizer, const gradient for each optimizing.
  \param [ts_start, ts_end]: time_range to form timesurface template  
  \param sample_num: the sample count beyond the time_range  
*/
void System::EstimateMotion_ransca_ceres(double ts_start, double ts_end, int sample_num, int total_iter_num)
{
    cout <<seq_count<< " time "<< eventBundle.first_tstamp.toSec() <<  ", total "
            << eventBundle.size << ", duration " << (eventBundle.last_tstamp - eventBundle.first_tstamp).toSec() 
            << ", sample " << sample_num << ", ts_start " << ts_start<<"~"<< ts_end << ", iter " << total_iter_num <<endl;

    bool show_time_info = false;
    // measure time 
    ros::Time t1, t2; 


    double residuals; 
    double ag_v_ang[6] = {est_angleAxis(0), est_angleAxis(1), est_angleAxis(2), 
                          est_trans_velocity(0), est_trans_velocity(1), est_trans_velocity(2)};
    // double ag_v_ang[8] = {0.815533,  0.102288,  -1.35302,  0.203447,  0.183943,  -1.53504,   0.19243, -0.103444};

    for(int iter_= 1; iter_<= total_iter_num; iter_++)
    {
        // get timesurface earlier 
        Eigen::Vector3d eg_angleAxis(ag_v_ang[0],ag_v_ang[1],ag_v_ang[2]);
        Eigen::Vector3d eg_trans_vel(ag_v_ang[3],ag_v_ang[4],ag_v_ang[5]);
        // cout << "before angleaxis " << eg_angleAxis.transpose() << endl;
         
        // double timesurface_range = iter_/50.0 + 0.2;  // original 
        // double timesurface_range = (iter_)/60.0 + 0.2;
        double timesurface_range = ts_start + iter_/float(total_iter_num) * (ts_end-ts_start);  
        cv::Mat cv_earlier_timesurface = cv::Mat(camera.height,camera.width, CV_32FC1); 
        cv::Mat cv_later_timesurface = cv::Mat(camera.height,camera.width, CV_32FC1); 


        // cv::Mat visited_map = cv::Mat(180,240, CV_8U); visited_map.setTo(0);
        float early_default_value = eventBundle.time_delta(int(eventBundle.size*timesurface_range));
        cv_earlier_timesurface.setTo(early_default_value * yaml_default_value_factor);
        // cout << "default early " << default_value << endl; 
        float later_default_value = eventBundle.time_delta(eventBundle.size-1) - eventBundle.time_delta(int(eventBundle.size*(1-timesurface_range)));
        cv_later_timesurface.setTo(later_default_value * yaml_default_value_factor);
        // cout << "default later " << default_value << endl; 

        // get t0 time surface of warpped image using latest angleAxis
        
        t1 = ros::Time::now();
        getWarpedEventImage(eg_angleAxis, eg_trans_vel, event_warpped_Bundle).convertTo(curr_warpped_event_image, CV_32FC3);  // get latest warpped events 
        t2 = ros::Time::now();
        if(show_time_info)
            cout << "getWarpedEventImage time " << (t2-t1).toSec() * 2 << endl;  // 0.00691187 s
        
        // visual optimizng process 
        // if(iter_ < 5) 
        // {
        //     cv::Mat temp_img;
        //     getWarpedEventImage(eg_angleAxis, event_warpped_Bundle).convertTo(temp_img, CV_32FC3);
        //     cv::imshow("opti", temp_img);
        //     // cv::Mat temp_img_char;
        //     // cv::threshold(temp_img, temp_img_char, 0.1, 255, cv::THRESH_BINARY);
        //     // cv::imwrite("/home/hxy/Desktop/data/optimize/opti_" + std::to_string(iter_) + ".png", temp_img_char);
        //     cv::waitKey(10);
        // }


    t1 = ros::Time::now();
        // get early timesurface
        for(int i= event_warpped_Bundle.size*timesurface_range; i >=0; i--)
        {
            
            int sampled_x = std::round(event_warpped_Bundle.coord.col(i)[0]), sampled_y = std::round(event_warpped_Bundle.coord.col(i)[1]); 

            if(event_warpped_Bundle.isInner[i] < 1) continue;               // outlier 
            // linear add TODO improve to module 
                cv_earlier_timesurface.at<float>(sampled_y, sampled_x) = eventBundle.time_delta(i);  
        } 
    t2 = ros::Time::now();
    if(show_time_info)
        cout << "cv_earlier_timesurface time " << (t2-t1).toSec() * 2 << endl; // 0.000106088

        // get t1 time surface of warpped image
        getWarpedEventImage(eg_angleAxis, eg_trans_vel, event_warpped_Bundle, PlotOption::U16C1_EVNET_IMAGE, true);
        // get later timesurface
        for(int i= event_warpped_Bundle.size*(1-timesurface_range); i< event_warpped_Bundle.size; i++)
        {
            
            int sampled_x = std::round(event_warpped_Bundle.coord.col(i)[0]), sampled_y = std::round(event_warpped_Bundle.coord.col(i)[1]); 

            if(event_warpped_Bundle.isInner[i] < 1) continue;               // outlier 
            // linear add TODO improve to module 
                cv_later_timesurface.at<float>(sampled_y, sampled_x) = eventBundle.time_delta(event_warpped_Bundle.size-1) - eventBundle.time_delta(i);  
        } 

            // /* visualize timesurface */  
            // {
                
            //     cv::Mat cv_earlier_timesurface_8U, cv_earlier_timesurface_color; 
            //     cv::normalize(cv_earlier_timesurface, cv_earlier_timesurface_8U, 255, 0, cv::NORM_MINMAX , CV_8UC1 );
            //     // cv_earlier_timesurface.convertTo(cv_earlier_timesurface_8U, CV_8UC1);
            //     cv::applyColorMap(cv_earlier_timesurface_8U, cv_earlier_timesurface_color, cv::COLORMAP_JET);
            //     cv::imshow("timesurface_early", cv_earlier_timesurface_color);
            //     cv::waitKey(10);
            // }
            // {
            //     // visualize timesurface 
            //     cv::Mat cv_later_timesurface_8U, cv_later_timesurface_color; 
            //     cv::normalize(cv_later_timesurface, cv_later_timesurface_8U, 255, 0, cv::NORM_MINMAX , CV_8UC1 );
            //     // cv_earlier_timesurface.convertTo(cv_earlier_timesurface_8U, CV_8UC1);
            //     cv::applyColorMap(cv_later_timesurface_8U, cv_later_timesurface_color, cv::COLORMAP_JET);
            //     cv::imshow("timesurface_later", cv_later_timesurface_color);
            //     cv::waitKey(10);
            // }

        // add gaussian on cv_earlier_timesurface
        cv::Mat cv_earlier_timesurface_blur, cv_later_timesurface_blur;
        int gaussian_size = yaml_gaussian_size;
        float sigma = yaml_gaussian_size_sigma;
        cv::GaussianBlur(cv_earlier_timesurface, cv_earlier_timesurface_blur, cv::Size(gaussian_size, gaussian_size), sigma);
        cv::GaussianBlur(cv_later_timesurface, cv_later_timesurface_blur, cv::Size(gaussian_size, gaussian_size), sigma);

        // get timesurface in ceres 
    t1 = ros::Time::now();
        // vector<float> line_grid_early; line_grid_early.assign((float*)cv_earlier_timesurface.data, (float*)cv_earlier_timesurface.data + 180*240);
        // vector<float> line_grid_later; line_grid_later.assign((float*)cv_later_timesurface.data, (float*)cv_later_timesurface.data + 180*240);
        vector<float> line_grid_early; line_grid_early.assign((float*)cv_earlier_timesurface_blur.data, (float*)cv_earlier_timesurface_blur.data + camera.width*camera.height);
        vector<float> line_grid_later; line_grid_later.assign((float*)cv_later_timesurface_blur.data, (float*)cv_later_timesurface_blur.data + camera.width*camera.height);
    t2 = ros::Time::now();
    if(show_time_info)
        cout << "convert inter time " << (t2-t1).toSec() << endl; // 0.000256303, using pointer reduce to 2.6943e-05

        ceres::Grid2D<float,1> grid_early(line_grid_early.data(), 0, camera.height, 0, camera.width);
        ceres::Grid2D<float,1> grid_later(line_grid_later.data(), 0, camera.height, 0, camera.width);
        auto* interpolator_early_ptr = new ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>(grid_early);
        auto* interpolator_later_ptr = new ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>(grid_later);  

        // sample events 
        // select 100 random points, and warp delta_t < min(t_point_delta_t). 
        // accumulate all time difference before and after warpped points. 
    t1 = ros::Time::now();
        std::vector<int> vec_sampled_idx; 
        int samples_count = std::min(sample_num, int(eventBundle.size)); 
        getSampledVec(vec_sampled_idx, samples_count, 0, 1);
    t2 = ros::Time::now();
    if(show_time_info)
        cout << "getSampledVec time " << (t2-t1).toSec() << endl; // 0.000473817

    t1 = ros::Time::now();  
        // init problem 
        ceres::Problem problem; 
        // add residual 
        for(int loop_temp =0; loop_temp < vec_sampled_idx.size(); loop_temp++)
        {
            size_t sample_idx = vec_sampled_idx[loop_temp];
            double early_time =  eventBundle.time_delta(sample_idx);
            double later_time =  eventBundle.time_delta(sample_idx) - eventBundle.time_delta(event_warpped_Bundle.size-1);

            ceres::CostFunction* cost_function = ResidualCostFunction::Create(
                                                    event_undis_Bundle.coord_3d.col(sample_idx),
                                                    early_time, later_time, 
                                                    camera.eg_cameraMatrix,
                                                    interpolator_early_ptr, interpolator_later_ptr,
                                                    last_est_var);

            problem.AddResidualBlock(cost_function, nullptr, &ag_v_ang[0]);
        }
    t2 = ros::Time::now();
    if(show_time_info)
        cout << "add residual time " << (t2-t1).toSec() << endl;  // 0.00168042

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = yaml_ceres_iter_thread;
        // options.logging_type = ceres::SILENT;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.use_nonmonotonic_steps = true;
        options.max_num_iterations = yaml_ceres_iter_num;
        // options.initial_trust_region_radius = 1;
        problem.SetParameterLowerBound(&ag_v_ang[0],0,-0.2);
        problem.SetParameterUpperBound(&ag_v_ang[0],0, 0.2);
        problem.SetParameterLowerBound(&ag_v_ang[0],1,-0.6);
        problem.SetParameterUpperBound(&ag_v_ang[0],1, 0.6);
        problem.SetParameterLowerBound(&ag_v_ang[0],2,-0.2);
        problem.SetParameterUpperBound(&ag_v_ang[0],2, 0.2);

        problem.SetParameterLowerBound(&ag_v_ang[0],3,-2);
        problem.SetParameterUpperBound(&ag_v_ang[0],3, 2);
        problem.SetParameterLowerBound(&ag_v_ang[0],4,-2);
        problem.SetParameterUpperBound(&ag_v_ang[0],4, 2);
        problem.SetParameterLowerBound(&ag_v_ang[0],5,-1); // outdoor(0-8)
        problem.SetParameterUpperBound(&ag_v_ang[0],5, 8); // outdoor(0-8)

        ceres::Solver::Summary summary; 

        // evaluate: choose init velocity, test whether using last_est or {0,0,0},
        if(false && iter_ == 1)
        {
            double cost = 0;
            vector<double> residual_vec; 
            // previous old velocity  
            ag_v_ang[0] = est_angleAxis(0); 
            ag_v_ang[1] = est_angleAxis(1); 
            ag_v_ang[2] = est_angleAxis(2); 
            ag_v_ang[3] = est_trans_velocity(0); 
            ag_v_ang[4] = est_trans_velocity(1); 
            ag_v_ang[5] = est_trans_velocity(2); 

            problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residual_vec, nullptr, nullptr);
            double residual_sum_old = std::accumulate(residual_vec.begin(), residual_vec.end(), 0.0);
            // 0 init 
            memset(ag_v_ang, 0.1, sizeof(double)*6); // skip the norm

            problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residual_vec, nullptr, nullptr);
            double residual_sum_0 = std::accumulate(residual_vec.begin(), residual_vec.end(), 0.0);

            if(residual_sum_old < residual_sum_0)
            {
                ag_v_ang[0] = est_angleAxis(0); 
                ag_v_ang[1] = est_angleAxis(1); 
                ag_v_ang[2] = est_angleAxis(2); 
                ag_v_ang[3] = est_trans_velocity(0); 
                ag_v_ang[4] = est_trans_velocity(1); 
                ag_v_ang[5] = est_trans_velocity(2); 
            }
            
            // cout << "using ang" << ag_v_ang[0] << "," << ag_v_ang[1] << "," << ag_v_ang[2] <<
            // " trans " << ag_v_ang[3] << "," << ag_v_ang[4] << "," << ag_v_ang[5] <<
            // ", residual size " << residual_vec.size() << ", sum_0: "<<  residual_sum_0 << ", sum_old: " <<residual_sum_old << endl; 
        }


        t1 = ros::Time::now();
        ceres::Solve(options, &problem, &summary);
        t2 = ros::Time::now();
        // if(show_time_info)
            cout << "   ceres time " << (t2-t1).toSec() << endl;  // 0.00383356 
        
        // cout << summary.BriefReport() << endl;

        // cout << "   iter " << iter_ << ", ceres iters " << summary.iterations.size()<< endl;
        // if(summary.BriefReport().find("NO_CONVERGENCE") != std::string::npos)
        // {
        //     cout << "   iter " << iter_ <<  ", No convergence Event bundle time " << eventBundle.first_tstamp.toSec() <<", size " << eventBundle.size << endl;
        //     // cout << summary.BriefReport() << endl;
        // }


        // bool visual_hot_c3 = false; 
        // if(iter_%5 == 1 && visual_hot_c3)
        // {

        //     // cout << "using angle axis " << est_angleAxis.transpose() << endl;
        //     // visualize warp events and get a timesurface image with indexs 
        //     {
        //         cv::Mat cv_timesurface; 
        //         cv_timesurface = getImageFromBundle(event_undis_Bundle, PlotOption::TIME_SURFACE);  
        //         cv::normalize(cv_timesurface, hot_image_C1, 0,255, cv::NORM_MINMAX, CV_8UC1);
        //         cv::cvtColor(hot_image_C1, hot_image_C3, cv::COLOR_GRAY2BGR);
        //     }

        //     int sample_green = 1;
        //     for(int i=0; i<vec_sampled_idx.size() ; i++)
        //     {
        //         // if(i<10) cout << "sampling " << sample_idx << endl;

        //         // viusal sample 
        //         if(i%sample_green != 0) continue;
        //         int x = int(event_undis_Bundle.coord(0, vec_sampled_idx[i]));
        //         int y = int(event_undis_Bundle.coord(1, vec_sampled_idx[i]));
        //         if(x>239 || y>179 || x<0 ||y<0) continue;
        //         hot_image_C3.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0);   // green of events before warp 
        //     }

        //     // compare with gt
        //     // cout << "estimated angleAxis " <<  est_angleAxis.transpose() << endl;   
            
        //     // plot earlier timestamps oringe 22，G：07，B：201
        //     for(int i=0; i<eventBundle.size/15 ; i++)
        //     {
        //         // viusal sample 
        //         // cout << "hello " << endl;
        //         if(i%sample_green != 0) continue;
        //         int x = int(event_undis_Bundle.coord(0, i));
        //         int y = int(event_undis_Bundle.coord(1, i));
        //         if(x>239 || y>179 || x<0 ||y<0) continue;
        //         hot_image_C3.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 165, 255);   // bottom events earlier  
        //     }

        //     // visualize est
        //     int sample_red = 1;
        //     for(int i=0; i<vec_sampled_idx.size(); i++)
        //     {
        //         // viusal sample 
        //         if(i%sample_red != 0) continue;
        //         int x = int(event_warpped_Bundle.coord(0, vec_sampled_idx[i])); 
        //         int y = int(event_warpped_Bundle.coord(1, vec_sampled_idx[i]));
        //         if(x>239 || y>179 || x<0 ||y<0) continue;
        //         hot_image_C3.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);   // red after warp
        //         // cout << "all inlier red" << endl;
        //     }
        // }

    }


    // output data 
    est_angleAxis = Eigen::Vector3d(ag_v_ang[0],ag_v_ang[1],ag_v_ang[2]);
    est_trans_velocity = Eigen::Vector3d(ag_v_ang[3],ag_v_ang[4],ag_v_ang[5]);

    cout << "    Loss: " << 0 << ", est_angleAxis " << est_angleAxis.transpose() << 
    ", trans " << est_trans_velocity.transpose() << endl;

    last_est_var << est_angleAxis, est_trans_velocity;  
}

