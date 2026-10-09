#ifndef ORBBEC_H
#define ORBBEC_H

#include <libobsensor/hpp/Pipeline.hpp>
#include <libobsensor/ObSensor.hpp>
#include <lyn_info.h>
#include <lyn_log.h>
#include <libobsensor/hpp/Error.hpp>
#include <thread>
#include <mutex>

struct gyroSensorData
{
    ulong timeStamp;
    ulong index;
    float temperature;
    cv::Point3f gyro;
    gyroSensorData() {}
};

struct accelSensorData
{
    ulong timeStamp;
    ulong index;
    float temperature;
    cv::Point3f accel;
    accelSensorData() {}
};

class orbbec
{
private:
    std::mutex printerMutex;
#define HAVE_PIPELINE                                                                              //必须有相机连接才能正常启动。
    int running = 1;
    std::thread worker;
    std::shared_ptr<ob::Frame> frame = nullptr;

public:
    orbbec();
    ~orbbec();

    int stop();
    std::vector<cv::Point3d> getPosition(std::vector<cv::Point3d> p);
    cv::Mat colorRawMatD;
    gyroSensorData gyro;
    accelSensorData accel;

    cv::Mat pointcloud_MatD;
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    lyn_Infos RawMats_info;

    std::atomic<lyn_step> step = lyn_step_free;
    lyn_log logger;
};
#endif // ORBBEC_H
