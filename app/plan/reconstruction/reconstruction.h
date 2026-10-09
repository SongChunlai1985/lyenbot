#ifndef LYN_RECONSTRUCTION_H
#define LYN_RECONSTRUCTION_H

#include <lyn_info.h>
#include <lyn_log.h>
#include <kalman_filter.h>
#include "imu_complementary_filter.h"
#include <imu_filter_madgwick.h>
class reconstruction
{
private:

#define MAX_RECONSTRUCTION_THREADS 5
    lyn_log logger;

    std::thread thread[MAX_RECONSTRUCTION_THREADS];

    int cv_transform(cv::Mat &src, cv::Mat &dst, cv::Mat m);

    enum thread_name
    {
        h12_imu_data,
        mid360_point_cloud,
        orbbec_point_cloud,
        realsense_point_cloud,
        output
    };

    std::unordered_map<int, std::string> thread_name_string =
    {
        {h12_imu_data, "h12_imu_data"},
        {mid360_point_cloud, "mid360_point_cloud"},
        {orbbec_point_cloud, "orbbec_point_cloud"},
        {realsense_point_cloud, "realsense_point_cloud"},
        {output, "output"},
    };

    std::unordered_map<std::string, std::mutex> mutex;
    std::mutex mutex_merge[MAX_RECONSTRUCTION_THREADS];
    std::unordered_map<std::string, lyn_Infos> input_public;
    std::unordered_map<std::string, lyn_Infos> output_public;
    std::unordered_map<std::string, std::condition_variable> condition_variable;
    std::unordered_map<std::string, std::atomic<lyn_step>> step;
    std::unordered_map<std::string, std::string> names =
    {
        {"imu_reconstruction", "imu_reconstruction"},
        {"LivoxLidarp_point_data_mat_infos", "LivoxLidarp_point_data_mat_infos"},
        {"orbbec", "orbbec"},
        {"reconstruction_output", "reconstruction_output"}
    };

public:
    reconstruction();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    int running = 1;

    meta IMU;
    meta IMU_Hi12;                                                                   // 因为meta里的cv::Mat没有线程安全性，所以要加锁

    lyn_Infos orbbec_RawMats_info;

    cv::Mat pointcloud_MatD_small;
    cv::Mat colorRawMatD_small;
    cv::Mat colorRawMatD_copy;
    cv::Mat pointcloud_MatD_small_copy;

    lyn_Infos reconstruction_output_infos;
    cv::Mat point_data_mat_xyzs_Copy;

    cv::Point3d imu_accel;
    cv::Point3d world_gravity = cv::Point3d(0, 1, 0) ;                                             //重力
    cv::Point3d world_accel;
    cv::Point3d rpy;
    cv::Mat point_data_mat_xyzs;

    int point_data_mat_xyzs_height = 200;

    int notify[3] = {0, 0, 0};

    meta current_gravity;

    int ColorRawMatScale = 4;

    cv::Point3d accel_bias = cv::Point3d(0, 0, 0) ;                                                // 加速计偏置

    cv::Point3d curr_accel;
    void thread_orbbec_point_cloud();
    void thread_output();
    void thread_mid360_point_cloud();
    void thread_h12_imu_data();

    void output_h12_imu_data();
    void output_mid360_point_cloud();
    void output_orbbec_point_cloud();

    std::deque<cv::Point3d> gyro_data;
    std::deque<cv::Point3d> gyro_filtered_data;

    std::deque<cv::Point3d> accel_data;
    std::deque<cv::Point3d> accel_filtered_data;

    meta MT;
    meta MT_madg;
    void thread_realsense_point_cloud();
};
#endif // LYN_RECONSTRUCTION_H
