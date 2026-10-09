#ifndef LYN_AXIS_ANGEL_H
#define LYN_AXIS_ANGEL_H
#include <thread>
#include <lyn_log.h>
#include <lyn_info.h>
#include <lyn_can.h>

class axis_angle
{
private:

    lyn_log logger;
    lyn_log logger_control;
    std::thread thread[2];

    std::unordered_map<std::string, std::atomic<lyn_step>> step;
    std::unordered_map<std::string, std::mutex> mutex;
    std::unordered_map<std::string, lyn_Infos> input_public;
    std::unordered_map<std::string, std::condition_variable> condition_variable;

    void wait_get(std::string name, cv::Mat &joint_angles);
    std::unordered_map<std::string, std::string> names =
    {
        {"direct_output_axis_angle", "direct_output_axis_angle"}
    };

    int send(std::string joint_name, std::string cmd, std::string cmd2, double value, double value2 = 0);
    int send_frames(std::string cmd, std::string cmd2, std::vector<float> dof_frame);
    void read_power(int microsecond, int sample_rate = 50);

    struct Record
    {
        double sample_rate = 0;
        std::vector<std::vector<float>> dof_frame;
        Record() {}
    };

    int test_motor(double min, double max, double interval);
    int send_frame(std::string cmd, std::string cmd2, double value);
    cv::Mat axis_angle2motor_angle(cv::Mat joint_angles);
    void enable_all_motor();
    void disable_all_motor();
    void key_control();

    std::atomic<int> key_press = 0;
    std::atomic<int> all_motor_enable = 0;

    std::vector<std::string> motor_list =
    {
        "LAnklePitch",
        "LAnkleRoll",

        "RAnklePitch",
        "RAnkleRoll"
    };

    void enable_some_motor();
    void disable_some_motor();
    void broadcast(std::string cmd, std::string cmd2, double value);

    void read_position_ring_config(std::string filename);
    void write_position_ring_config(std::string filename);
    std::unordered_map<std::string, float> position_ring;
    std::unordered_map<std::string, float> PD_mode;
    void get_info(const std::string &name, std::string Info_name, cv::Mat &info_mat);
    void read_PD_mode_config(std::string filename);
    void write_PD_mode_config(std::string filename);
public:
    axis_angle();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

    int running = 1;
    can Can[4];
    cv::Mat motor_UIDs;
};
#endif // LYN_AXIS_ANGEL_H
