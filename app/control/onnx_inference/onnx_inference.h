#ifndef LYN_ONNX_INFERENCE_H
#define LYN_ONNX_INFERENCE_H
#include <lyn_info.h>
#include <thread>
#include <lyn_log.h>
#include <random>

enum onnx_net_type
{
    lyn_net_stop,
    lyn_net_walk,
    lyn_net_run
};

struct active_observation_268
{
    float base_lin_vel[3];                       // IMU加速度
    float base_ang_vel[3];                       // IMU角速度
    float projected_gravity[3];                  // 重力方向
    float velocity_commands[3];                  // 运动命令矢量
    float joint_pos[23];                         // 关节位置，从电机获取
    float joint_vel[23];                         // 关节速度
    float actions[23];                           // 直接动作，从直接控制获取
    float height_scan[187];                      // 高程图，从三维地图中获取
};

class onnx_inference
{
private:
    void thread_walk();
    lyn_log logger;
    lyn_Infos onnx_output_public;
    std::string modelPath[3] = {"/home/lyenbot/modles/policy_stop.onnx",
                                "/home/lyenbot/modles/policy_walk.onnx",
                                "/home/lyenbot/modles/policy_run.onnx"};
    cv::dnn::Net net[3];
    cv::Mat input = cv::Mat(1, 128, CV_32FC1);
    std::thread thread;
    onnx_net_type cur_net;
    active_observation_268 Active_observation_268;
    std::atomic<lyn_step> step[2];
    enum step_name
    {
        joints_value,
        h12_imu_data,
        step_input,
    };
    void set_input(cv::Mat Hi12_Imu_data_mat,
                   cv::Mat onnx_output,
                   cv::Mat onnx_output_last,
                   active_observation_268 &Active_observation_268);
    lyn_Infos Hi12_Imu_data_mat_infos;
    void simulate_joint_pos(active_observation_268 &obs, cv::Mat onnx_output);
    void simulate_joint_vel(active_observation_268 &obs, cv::Mat onnx_output, cv::Mat onnx_output_last);
    std::mutex mutex;
public:
    onnx_inference();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    std::mt19937 gen;
    void simulate_flat_ground(active_observation_268 &obs, float base_height = 0.5f);
    void simulate_joint_actions(active_observation_268 &obs, cv::Mat onnx_output);
};
#endif // LYN_ONNX_INFERENCE_H
