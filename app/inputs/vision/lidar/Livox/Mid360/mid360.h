#ifndef LYN_MID360_H
#define LYN_MID360_H

#include <livox_lidar_def.h>
#include <livox_lidar_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <lyn_log.h>
#include <lyn_info.h>
#include <mutex>

#include <cstring>
#include <cstdint>

static cv::Mat LivoxLidarp_point_data_mat;
static cv::Mat LivoxLidarp_point_data_mat_reflectivity;
static cv::Mat LivoxLidarp_point_data_mat_tag;

static cv::Mat LivoxLidarp_point_data_mat_reflectivity_Public;
static cv::Mat LivoxLidarp_point_data_mat_tag_Public;

static lyn_Infos LivoxLidarp_point_data_mat_infos;
static lyn_Infos LivoxLidarp_point_data_mat_infos_Public;

static void* LivoxLidarp_Imu_frame;
static cv::Mat LivoxLidarp_Imu_data_frame;
static cv::Mat LivoxLidarp_Imu_data_mat;
static int LivoxLidarp_Imu_data_mat_update;

static lyn_log logger_mid360_PointCloudCallback;
static lyn_log logger_mid360_ImuDataCallback;
static lyn_log logger_mid360;

static std::mutex logger_mid360_mutex;
static long LivoxLidarp_Imu_timestamp;

class mid360
{
private:

public:
    mid360();
    ~mid360();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    static std::atomic<lyn_step> step;
};
#endif // LYN_MID360_H
