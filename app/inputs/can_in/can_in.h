#ifndef LYN_CAN_IN_H
#define LYN_CAN_IN_H

#include <thread>
#include <lyn_log.h>
#include <lyn_info.h>
#include <lyn_can.h>
#include <shared_mutex>

class can_in
{
private:
    std::thread thread;

    enum step_name
    {
        joints_shaft_position,
        joints_absolute_position,
        h12_imu_data,
        step_input,
    };

    static void update_info(std::string name, std::string Info_name, cv::Mat mat);
    int running = 1;

    can Can[4];

    static void can_call_back(std::string name,
                              unsigned int& can_id,
                              unsigned char& can_dlc,
                              unsigned char* dat);

    static cv::Mat motor_angle2axis_angle(cv::Mat &joint_angles);

    void thread_joint_angles();
    int work_push(lyn_info &info, std::string name);

    std::unordered_map<std::string, std::string> names =
    {
        {"can_data_mat_infos", "can_data_mat_infos"},
        {"can_data_mat_infos_direct_control", "can_data_mat_infos_direct_control"},
        {"motor_UIDs", "motor_UIDs"}
    };
public:
    can_in();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

};
#endif // LYN_CAN_IN_H
