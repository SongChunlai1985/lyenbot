#include "lyn_info.h"
#include <execinfo.h>
lyn_info::lyn_info()
{

}

int lyn_info::unload(std::string &name,
                  std::unordered_map<std::string, std::atomic<lyn_step>> &step,
                  std::unordered_map<std::string, lyn_Infos> &input_public,
                  std::unordered_map<std::string, std::condition_variable> &condition_variable)
{
    if(steps[name] == lyn_step_update_complete)
    {
        steps[name] = lyn_step_copying;
        input_public[name] = infos[name];
        step[name].store(lyn_step_copying);
        condition_variable[name].notify_one();
    }

    if(step[name].load() == lyn_step_copy_complete)
    {
        steps[name] = lyn_step_free;
        step[name].store(lyn_step_free);
    }

    return 0;
}

int lyn_info::load(std::string &name,
                   std::unordered_map<std::string, std::atomic<lyn_step>> &step,
                   std::unordered_map<std::string, lyn_Infos> &output_public)
{
    if(steps[name] == lyn_step_free)
    {
        steps[name] = lyn_step_request;
    }

    if(steps[name] == lyn_step_request)
    {
        step[name].store(lyn_step_updating);
        steps[name] = lyn_step_updating;
    }

    if(step[name].load() == lyn_step_update_complete)
    {
        infos[name] = std::move(output_public[name]);
        steps[name] = lyn_step_update_complete;
        step[name].store(lyn_step_free);
    }
    return 0;
}

std::string getCurrentTimeWithMicroseconds()
{
    auto now = std::chrono::system_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>
            (now.time_since_epoch());
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm local_time = *std::localtime(&time_t_now);
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0')
        << (microseconds.count() % 1000000);
    return oss.str();
}

void print_mat_formatted(const cv::Mat& mat, int precision)
{
    cv::Mat direct_output_deg = cv::Mat(mat, cv::Rect(0, 0, 23, 1));
    std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] "
    << std::fixed << std::setprecision(precision);
    for (int i = 0; i < direct_output_deg.rows; i++)
    {
        for (int j = 0; j < direct_output_deg.cols; j++)
        {
            std::cout << std::setw(10) << direct_output_deg.at<float>(i, j) << ", ";
        }
    }
    std::cout << std::resetiosflags(std::ios_base::floatfield);
}
