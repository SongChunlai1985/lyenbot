#ifndef LYN_ROBOT_H
#define LYN_ROBOT_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <components.h>
#include <lyn_urdf_parser.h>
class robot                                      // 机器人绘制模块
{
public:
    robot();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
private:
    lyn_log logger;
    std::string urdf_file = "/home/lyenbot/config/LB_URDFfiles/urdf/LB_URDFfiles.urdf";
    lyn_urdf_parser Lyn_urdf_parser;
    cv::Mat Points;
    std::mutex mutex;

    enum step_name
    {
        robot_meshes_data,
        robot_joints,
        joints_value
    };
    int running = 1;
    std::vector<double> angles;
    void thread_robot_matrixs();
    components Components;
    void assembly(components &Comp_parent);
    void update_robot_angles(components &Comp_parent,
                             int &index,
                             std::vector<double> &angles,
                             cv::Mat &onnx_output);
    void get_meshes(components Components,
                    lyn_Infos &robot_info_meshes,
                    int depth = 0);
    void get_joints_points(components &Comp_parent,
                           cv::Mat &Points,
                           int &index,
                           const cv::Mat &gl_rotate_matrix);
    std::thread thread;
    void get_robot_run_transforms(components &Comp_parent,
                                  lyn_Infos &robot_info,
                                  int &index,
                                  std::vector<double> &angles);
    const cv::Mat gl_rotate_matrix = get_rotate_matrix(cv::Point3d(-90 ANGLE, -90 ANGLE, 0 ANGLE),
                                                       meta(), Rotate_Sequence_XYZ);
    void get_robot_float_transforms(components &Comp_parent,
                                    lyn_Infos &robot_info,
                                    int depth,
                                    int &index,
                                    std::vector<double> &angles);
    void get_robot_info(components &Comp_parent, lyn_Infos &robot_info);

    int get_key(lyn_info &info);

    std::unordered_map<std::string, std::string> names =
    {
        {"robot_info_meshes", "robot_info_meshes"},
        {"robot_info_matrixs", "robot_info_matrixs"},
        {"robot_info_joints_points", "robot_info_joints_points"},
        {"info_joints_points", "info_joints_points"},
        {"can_data_mat_infos", "can_data_mat_infos"},
        {"direct_output_robot", "direct_output_robot"},
        {"onnx_output_public", "onnx_output_public"}
    };

    std::unordered_map<std::string, lyn_Infos> input_public;
    std::unordered_map<std::string, lyn_Infos> output_public;
    std::unordered_map<std::string, std::atomic<lyn_step>> step;
    std::unordered_map<std::string, std::condition_variable> condition_variable;

    std::atomic<int> first_run = 0;

    std::atomic<int> key_press = 0;
    std::string joint_source = "can_data_mat_infos";
    int drop_expired_info();
    void wait_get(std::string name, cv::Mat &joint_angles);
    void key_control();
    void public_info(const std::string &name, std::string subname, cv::Mat mat);
};
#endif // LYN_ROBOT_H
