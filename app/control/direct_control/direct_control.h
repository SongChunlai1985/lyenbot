#ifndef LYN_DIRECT_CONTROL_H
#define LYN_DIRECT_CONTROL_H
#include <robot.h>
#include <walk_main.h>
class direct_control
{
public:
    direct_control();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

    double make_angle(double current_time_tickt,
                      double angle_min,
                      double angle_max,
                      double frequency);

    void shake_joint(std::string name, double current_time_tickt,
                     double angle_min,
                     double angle_max,
                     double frequency);

    void shake_all_joint(double t_ms);
    void action_playback(uint index, std::vector<std::vector<float> > &action_data);
    void shake_joint_in_time(double start_time_tickt,
                             double end_time_tickt,
                             std::string name,
                             double current_time_tickt,
                             double angle_min,
                             double angle_max,
                             double frequency);
    void move_joint_in_time(double start_time_tickt,
                            double end_time_tickt,
                            uint index,
                            double current_time_tickt,
                            double angle_start,
                            double angle_end);
    void move_to_posture(double current_time_tickt,
                         double start_time_tickt,
                         double end_time_tickt,
                         std::vector<double> angle_start,
                         std::vector<double> angle_end);
    void read_posture(std::string filename);
private:
    lyn_log logger;
    std::unordered_map<std::string, std::thread> thread;
    robot Robot;

    std::unordered_map<std::string, std::atomic<lyn_step>> step;
    cv::Mat axis_absolute_position;

    void thread_walk();
    std::unordered_map<std::string, std::condition_variable> condition_variable;
    std::unordered_map<std::string, std::mutex> mutex;

    cv::Mat direct_output = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));

    std::vector<std::vector<float> > read_aciton_file(std::string filename);
    std::atomic<int> key_press = 0;
    std::vector<std::vector<double>> angle_posture;
    std::vector<double> start_time_tickts;
    std::vector<double> end_time_tickts;
    std::vector<double> squat_angle_posture_current;

    std::unordered_map<std::string, lyn_Infos> input_public;
    std::unordered_map<std::string, lyn_Infos> output_public;
    void thread_can_data();

    int running = 1;
    void thread_joint_angles();
    cv::Mat joint_angles = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));
    void public_info(const std::string &name);
    void public_info();
    std::unordered_map<std::string, std::string> names =
    {
        {"direct_output_axis_angle", "direct_output_axis_angle"},
        {"direct_output_robot", "direct_output_robot"},
        {"can_data_mat_infos_direct_control", "can_data_mat_infos_direct_control"},
        {"imu_direct_control", "imu_direct_control"}
    };
    void wait_get(const std::string &name, cv::Mat &Hi12_Imu_data_mat);
    void attention(std::string &action,
                   double &time_speed,
                   double &current_time_tickt,
                   double &send_intervel,
                   int &attention_is_start,
                   cv::Mat &joint_angles_clone,
                   cv::Mat &start_posture,
                   cv::Mat &end_posture);

    void action_walk(double &time_speed, int &ready_to_run, int &going_to_ready_target, double &current_time_tickt, double &send_intervel, cv::Mat &joint_angles_clone, cv::Mat &start_posture, cv::Mat &end_posture, cv::Mat &Frame, Walk &walk);
};
#endif // LYN_DIRECT_CONTROL_H
