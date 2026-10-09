#ifndef LYN_AVOIDING_H
#define LYN_AVOIDING_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <thread>
#include <mutex>

class avoiding
{
private:
    void wait_get(std::string name, cv::Mat &colorRawMat, cv::Mat &pointcloud_Mat);
    lyn_log logger;
    cv::Mat colorRawMat;
    cv::Mat colorRawMatD;
    cv::Mat pointcloud_Mat;
    cv::Mat pointcloud_MatD;

    std::thread thread;

    std::unordered_map<std::string, lyn_Infos> input_public;

    std::unordered_map<std::string, std::atomic<lyn_step>> step;

    std::unordered_map<std::string, std::condition_variable> condition_variable;
    std::mutex mutex;
    std::unordered_map<std::string, std::string> names =
    {
        {"orbbec_avoiding", "orbbec_avoiding"}
    };

public:
    avoiding();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    int running = 1;
    int i = 0;

};
#endif // LYN_AVOIDING_H
