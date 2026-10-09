#ifndef LYN_INFO_H
#define LYN_INFO_H
#include <opencv2/opencv.hpp>
#include <variant>
#include <atomic>
#include <condition_variable>

#ifndef LYN_PI
#define LYN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L
#define DEG double(LYN_PI) / 180.0
#define RAD 180.0 / double(LYN_PI)
#endif

std::string getCurrentTimeWithMicroseconds();
void print_mat_formatted(const cv::Mat& mat, int precision = 4);

typedef std::unordered_map
    <
    std::string, std::variant
                 <
                     int,
                     uint,
                     double,
                     float,
                     long,
                     ulong,
                     std::string,
                     cv::Mat
                 >
    > lyn_Info;                                                                                    // 此条信息名， 此信息
typedef std::vector<lyn_Info> lyn_Infos;                                                           // 很多条信息


enum lyn_step
{
    lyn_step_free,
    lyn_step_request,
    lyn_step_updating,
    lyn_step_update_complete,
    lyn_step_copying,
    lyn_step_copy_complete,
    lyn_step_disable
};

static std::unordered_map<int, std::string> SetpString =
{
    {lyn_step_free, "lyn_step_free"},
    {lyn_step_request, "lyn_step_request"},
    {lyn_step_updating, "lyn_step_updating"},
    {lyn_step_update_complete, "lyn_step_update_complete"},
    {lyn_step_copying, "lyn_step_copying"},
    {lyn_step_copy_complete, "lyn_step_copy_complete"},
    {lyn_step_disable, "lyn_step_disable"}
};

class lyn_info                                                                                     // 所有模块的信息，包含每个模块的信息包
{
public:
    lyn_info();
    std::unordered_map<std::string, lyn_Infos> infos;                                                        // 信息包名，包含一个模块的很多条信息
    std::unordered_map<std::string, int> steps;

    int key_press = 0;

    int load(std::string &name,
                  std::unordered_map<std::string, std::atomic<lyn_step> > &step,
                  std::unordered_map<std::string, lyn_Infos> &output_public);
    int unload(std::string &name,
                  std::unordered_map<std::string,
                  std::atomic<lyn_step>> &step,
                  std::unordered_map<std::string, lyn_Infos> &input_public,
                  std::unordered_map<std::string, std::condition_variable> &condition_variable);
    int index_min = -1;
};
#endif // LYN_INFO_H
