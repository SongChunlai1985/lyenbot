#ifndef LYN_IMU_H
#define LYN_IMU_H

#include <thread>
#include <lyn_log.h>
#include <lyn_info.h>
#include <lyn_usbtty.h>

class imu
{
private:
    lyn_log logger;
    std::thread thread;

    std::unordered_map<std::string, std::atomic<lyn_step>> step;

    void public_info(const std::string &name,
                     cv::Mat Hi12_Imu_data_mat,
                     uint32_t Hi12_Imu_timestamp);
    std::unordered_map<std::string, std::string> name =
    {
        {"imu_reconstruction", "imu_reconstruction"},
        {"imu_direct_control", "imu_direct_control"},
        {"imu_onnx_inference", "imu_onnx_inference"}
    };


public:
    imu();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

    int running = 1;

    std::unordered_map<std::string, lyn_Infos> output_public;

};
#endif // LYN_IMU_H
