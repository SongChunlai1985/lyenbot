#ifndef REAL_SENSE_H
#define REAL_SENSE_H

#include <lyn_info.h>
#include <lyn_log.h>
#include <thread>
#include <mutex>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

class real_sense
{
private:
    std::mutex printerMutex;                                                                       //必须有相机连接才能正常启动。
    int running = 1;
    std::thread worker;

public:
    real_sense();
    ~real_sense();

    int stop();

    ulong color_index;
    cv::Mat colorRawMat;
    cv::Mat colorRawMatD;

    ulong depth_index;
    cv::Mat depthRawMat;

    cv::Mat pointcloud_Mat;
    cv::Mat pointcloud_MatD;
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    lyn_Infos RawMats_info;

    std::atomic<lyn_step> step = lyn_step_free;
    lyn_log logger;
};
#endif // REAL_SENSE_H
