#ifndef SYSTEM_H
#define SYSTEM_H

// #include <cmath>
#include <ros/ros.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// eigen
#include <eigen3/Eigen/Core>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>

#include <thread>
// #include <chrono>

#include "Database.h"
#include "utils/numerics.h"

#include <fstream>
#include <iostream>

#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

enum PlotOption{
    DVS_RENDERER,
    SIGNED_EVENT_IMAGE,
    SIGNED_EVENT_IMAGE_COLOR,
    UNSIGNED_EVENT_IMAGE,
    GRAY_EVENT_IMAGE
};


class System
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    System(const std::string& strSettingsFile);

    ~System();

    // Data read
    void PushEventData(EventData eventData, std::vector<double>  &imu_pos, std::vector<double>  &imu_ort);

private:
    /// functions
    void Run();
    void RunUptoIdx();
    
    void BindEvent();
    void BindMap();
    void ClearEvent();

    void UndistortEvent();
    void UndistortImage();
    void UndistortEventOrg();
    void GenerateMesh();

    void EstimateMotion();
    void GetWarpedEventImageBack(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const Eigen::Vector3f &temp_ang_pos);
    void GetWarpedEventImageFor(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2);
    void GetWarpedEventMap(const Eigen::Vector3f &temp_ang_pos);
    void GetWarpedEventPoint(const EventBundle &eventIn, 
                            EventBundle &eventOut, 
                            const Eigen::Vector3f &temp_ang_vel, 
                            const bool &is_map_warping = false,
                            const Eigen::Vector3f &temp_ang_pos = Eigen::Vector3f::Zero());
    void GetWarpedEventPointBack(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const bool &is_map_warping = false, const Eigen::Vector3f &temp_ang_pos = Eigen::Vector3f::Zero());
    void GetWarpedEventPointFor(const EventBundle &eventIn, EventBundle &eventOut, const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2);
    void DeriveErrAnalyticBack(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2, const Eigen::Vector3f &temp_ang_pos);
    void DeriveErrAnalyticFor(const Eigen::Vector3f &temp_ang_vel, const Eigen::Vector3f &temp_ang_acc1, const Eigen::Vector3f &temp_ang_acc2);
    void DeriveErrAnalyticMap(const Eigen::Vector3f &temp_ang_pos);
    std::vector<uint16_t> GetValidIndexFromEvent(const EventBundle &event);
    void ComputeAngularPosition();

    void Visualize();
    void Renderer();
    cv::Mat GetEventImage(const EventBundle& e, const PlotOption &option, const bool &is_large_size = 0);
    
    void GenerateFrame();
    cv::Mat GetWarpedImageNew(const PlotOption &option = UNSIGNED_EVENT_IMAGE);
    cv::Mat GetFrameNew(const PlotOption &option = UNSIGNED_EVENT_IMAGE);


    
    /// variables
    // data stack
    std::vector<EventData> vec_event_data;
    std::vector<EventBundle> vec_event_bundle_data;

    // event points
    EventBundle eventBundle;
    EventBundle eventUndistorted;
    EventBundle eventUndistortedPrev;
    EventBundle eventWarpedBack;
    EventBundle eventWarpedFor;
    EventBundle eventAlignedPoints;
    EventBundle eventMap;
    EventBundle eventWarpedBackMap;
    std::vector<EventBundle> eventMapPoints;
    EventBundle eventBundleOrg;
    EventBundle eventWarpedBackOrg;
    EventBundle eventUndistortedOrg;
    
    EventBundle eventOrg;

    // Image
    ImageData current_image;
    ImageData undistorted_image;
    cv::Mat undistorted_render;
    cv::Mat map_render;
    cv::Mat undist_mesh_x;
    cv::Mat undist_mesh_y;

    // time
    double event_delta_time;
    double last_event_time;
    double estimation_time_interval;
    double estimation_time_interval_prev;


    // counter
    uint16_t map_iter_begin;

    size_t event_data_iter;

    // samplier
    int sampling_rate;
    int map_sampling_rate;

    // camera parameters
    Eigen::Matrix3f K;
    Eigen::Matrix3f K_map;
    CameraParam camParam;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int width, height;
    int width_map, height_map;
    float map_scale;

    // camera pose
    std::vector<Eigen::Vector3f> vec_angular_velocity;
    std::vector<Eigen::Vector3f> vec_angular_position;

    Eigen::Vector3f angular_velocity;   //w_1'
    Eigen::Vector3f angular_velocity_prev;
    Eigen::Vector3f angular_acc1;
    Eigen::Vector3f angular_acc2;
    Eigen::Vector3f angular_position;   //R_2_w
    Eigen::Vector3f angular_position_init;
    Eigen::Vector3f angular_position_prev;   //R_1'_w
    Eigen::Vector3f angular_position_pprev;
    Eigen::Matrix3f angular_position_increment;  ////R_2_1'
    Eigen::Vector3f angular_position_compensator;   //R_1'_1

    Eigen::Vector3f update_angular_velocity;
    Eigen::Vector3f update_angular_acc1;
    Eigen::Vector3f update_angular_acc2;
    Eigen::Vector3f update_angular_position_compensator;

    Eigen::Vector3f update_angular_position;

    //groundtruth
    Eigen::Quaternionf q;
    Eigen::Vector3f euler;

    Eigen::Vector3f gt_position;
    Eigen::Quaternionf gt_orientation;
    Eigen::Vector4f gt_orientation_new;
    Eigen::Vector3f gt_euler;
    Eigen::Quaternionf gt_orientation_correction;

    //benchmark
    double er_euler_total;
    double mean_er_euler_total;
    double total_square_error_x, total_square_error_y, total_square_error_z;
    double mean_square_error_x, mean_square_error_y, mean_square_error_z; 
    double root_mean_square_error_x, root_mean_square_error_y, root_mean_square_error_z;

    // visualize 
    float plot_denom;
    float denominator;

    cv::Mat warped_event_image;
    cv::Mat warped_event_image_th;
    cv::Mat warped_event_image_b;
    cv::Mat warped_event_image_b_th;
    cv::Mat warped_event_image_grabbed;
    cv::Mat warped_map_image;
    cv::Mat warped_map_result;
    cv::Mat warped_map_result_th;
    cv::Mat event_frame;
    cv::Mat event_frame_th;
    cv::Mat frame_render;

    /// Estimator variables
    // optimization parameters
    float nu_event; // exponential average of squares of gradients
    float mu_event; // step size | optimization rate
    float rho_event; // smoothing factor | the degree of weigthing decrease in geometric moving average
    
    float nu_event0, nu_event1, nu_event2; // exponential average of squares of gradients
    float mu_event0, mu_event1, mu_event2; // step size | optimization rate
    float rho_event0, rho_event1, rho_event2; // smoothing factor | the degree of weigthing decrease in geometric moving average

    float nu_map;
    float mu_map;
    float rho_map;

    int optimizer_max_iter;
    int optimizer_max_iter0;
    bool is_polarity_on;

    // error analytics
    cv::Mat grad_x_kernel;
    cv::Mat grad_y_kernel;

    Eigen::VectorXf xp_zp;
    Eigen::VectorXf yp_zp;

    Eigen::ArrayXf xx_zz;
    Eigen::ArrayXf yy_zz;
    Eigen::ArrayXf xy_zz;

    Eigen::VectorXf xp;
    Eigen::VectorXf yp;
    Eigen::VectorXf zp;

    cv::Mat Ix_sum;
    cv::Mat Iy_sum;

    cv::Mat Ix;
    cv::Mat Iy;
    Eigen::VectorXf Ix_interp;
    Eigen::VectorXf Iy_interp;

    Eigen::VectorXf xp_map;
    Eigen::VectorXf yp_map;
    Eigen::VectorXf zp_map;

    cv::Mat blur_image;
    cv::Mat blur_image_map;
    cv::Mat truncated_map, truncated_event, truncated_sum;
    cv::Mat truncated_event_b, blur_image_b;
    cv::Mat Ix_b;
    cv::Mat Iy_b;
    
    cv::Mat Ix_map;
    cv::Mat Iy_map;
    Eigen::VectorXf Ix_interp_map;
    Eigen::VectorXf Iy_interp_map;

    Eigen::VectorXf warped_image_delta_t;
    Eigen::VectorXf warped_map_delta_t;

    Eigen::MatrixXf Jacobian;
    Eigen::MatrixXf Jacobian_mapp;

    Eigen::Vector3f Gradient;
    Eigen::Vector3f Gradient_mapp;

    Eigen::VectorXf Jacobian_dt_vel;
    Eigen::VectorXf Jacobian_dt_acc1;
    Eigen::VectorXf Jacobian_dt_acc2;
    Eigen::VectorXf Jacobian_dt_map;

    Eigen::VectorXf Jacobian_dt_vel_b;
    Eigen::VectorXf Jacobian_dt_acc1_b;
    Eigen::VectorXf Jacobian_dt_acc2_b;
    Eigen::MatrixXf Jacobian_acc_b;

    Eigen::MatrixXf Jacobian_vel;
    Eigen::MatrixXf Jacobian_acc;
    Eigen::MatrixXf Jacobian_map;

    Eigen::Vector3f Gradient_vel;
    Eigen::Vector3f Gradient_acc1;
    Eigen::Vector3f Gradient_acc2;
    Eigen::Vector3f Gradient_map;

    int event_image_threshold;
    int rotation_estimation;
    int run_index;
    float mapping_interval;
    int optimization_view_index;
    bool optimization_view_flag;

    std::string filePath;
    std::ofstream writeFile;

    int cnt_iter;

    // evaluation matrics
    double mean_average_evaluation_time;
    double total_total_evaluation_time;
    size_t total_data_size;
};

#endif // SYSTEM_H