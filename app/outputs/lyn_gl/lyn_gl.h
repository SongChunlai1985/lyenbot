#ifndef LYN_GL_H
#define LYN_GL_H

#include <thread>
#include <lyn_info.h>
#include <lyn_shader.hpp>
#include <lyn_meta.h>
#include <lyn_log.h>
#include <mutex>
#include <shared_mutex>

class lyn_gl
{
public:
    lyn_gl();
    ~lyn_gl();
    int run(int argc, char** argv );
    int work(lyn_info &info);
    static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
    static void mouse_callback(GLFWwindow *window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    static meta camera;
    static cv::Point3d camera_position;
    static glm::vec3 trans_xyz;
    static glm::vec3 scale;

    void thread_reconstruction_data();
    void thread_robot_meshes_data();
    void thread_robot_joints();
    void thread_robot_info_matrixs();
    void thread_gl_main();
    void gl_start();

    void draw();

    GLuint sk1;
    int window_width = 2200;                                                                       //  1850, 1136
    int window_height = 1500;

private:
    #define MAX_GL_THREADS 5

    enum thread_name
    {
        gl_init,
        gl_main,
        reconstruction_data,
        robot_meshes_data,
        robot_joints
    };

    std::unordered_map<int, std::string> thread_name_string =
    {
        {gl_main, "gl_main"},
        {reconstruction_data, "reconstruction_data"},
        {robot_meshes_data, "robot_meshes_data"},
        {robot_joints, "robot_joints"},
    };

    GLFWwindow* window = nullptr;
    static lyn_shader shader;

    std::shared_mutex render_mutex;

    glm::vec3 to_glm_vec3(cv::Point3d cv_point3d);
    cv::Point3f to_cv_Point3f(glm::vec3 vec3);

    enum {left, right, middle};

    cv::Mat pointcloud_MatD_small = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat colorRawMatD_copy = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat point_data_mat_xyzs_copy = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat IMU_Points = cv::Mat(100, 3, CV_64FC3, cv::Scalar(0));
    cv::Mat IMU_Points_h12 = cv::Mat(3704, 1, CV_64FC3, cv::Scalar(0));
    cv::Mat Points = cv::Mat(276, 1, CV_32FC3, cv::Scalar(0));
    std::vector<cv::Mat> robot_matrixs;
    volatile int update[6];
    enum update_names
    {
        pointcloud_MatD_small_update,
        colorRawMatD_update,
        point_data_mat_xyzs_copy_update,
        IMU_Points_update,
        IMU_Points_h12_update,
        robot_info_matrix_update
    };

    enum VertexAttrib_names { attrib_0, attrib_1, attrib_2 };
    enum VAO_names
    {
        color_cloud,
        point_cloud,
        color_lines,
        color_lines_h12,
        color_lines_robot_joints
    };
    enum VBO_names
    {
        color_cloud_position_buffer,
        color_cloud_color_buffer,
        point_cloud_position_buffer,
        point_cloud_color_buffer,
        color_lines_buffer,
        color_lines_buffer_h12,
        color_lines_robot_joints_buffer
    };
    int running = 1;
    lyn_log logger;
    std::unordered_map<std::string, std::mutex> mutex;

#define VAO_amount 5
#define VBO_amount 10

    GLuint VAO[VAO_amount];
    GLuint VBO[VBO_amount];

    cv::Mat pointcloud_MatD_small_copy = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat colorRawMatD_copy_copy = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat colorRawMatD_copy_copy_BGR = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat colorRawMatD_copy_copy_32f = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat point_data_mat_xyzs_copy_copy = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat point_data_mat_xyzs_color = cv::Mat(180, 320, CV_64FC3, cv::Scalar(0));
    cv::Mat IMU_Points_copy = cv::Mat(100, 3, CV_64FC3, cv::Scalar(0));
    cv::Mat IMU_Points_h12_copy = cv::Mat(3704, 1, CV_64FC3, cv::Scalar(0));
    std::vector<cv::Mat> robot_matrixs_copy;

    void setup_VAO();

    GLuint VAO_MESHES[100];
    GLuint VBO_MESHES[100];
    GLsizeiptr VBO_GLsizeiptrs[100];
    cv::Mat mesh01_mats[100];
    glm::mat4 model;
    void gl_update_data();
    meta camera_gl;
    glm::mat4 view;
    const cv::Mat gl_rotate_matrix = get_rotate_matrix(cv::Point3d(-90 ANGLE, -90 ANGLE, 0),
                                              meta(), Rotate_Sequence_XYZ);

    std::vector<glm::vec3> getColorPalette24 =
    {
        glm::vec3(0.2f, 0.6f, 0.9f),      // 浅橙            盆骨

        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿            左腿
        glm::vec3(0.9f, 0.6f, 0.2f),      // 浅橙
        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿
        glm::vec3(0.6f, 0.2f, 0.9f),      // 紫罗兰
        glm::vec3(0.9f, 0.2f, 0.6f),      // 粉红
        glm::vec3(0.2f, 0.6f, 0.9f),      // 浅天蓝

        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿            右腿
        glm::vec3(0.9f, 0.6f, 0.2f),      // 浅橙
        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿
        glm::vec3(0.6f, 0.2f, 0.9f),      // 紫罗兰
        glm::vec3(0.9f, 0.2f, 0.6f),      // 粉红
        glm::vec3(0.2f, 0.6f, 0.9f),      // 浅天蓝

        glm::vec3(0.6f, 0.9f, 0.2f),      // 黄绿            上身

        glm::vec3(0.2f, 0.6f, 0.9f),      // 浅天蓝          左手
        glm::vec3(0.9f, 0.6f, 0.2f),      // 浅橙
        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿
        glm::vec3(0.6f, 0.2f, 0.9f),      // 紫罗兰
        glm::vec3(0.9f, 0.2f, 0.6f),      // 粉红

        glm::vec3(0.2f, 0.6f, 0.9f),      // 浅天蓝          右手
        glm::vec3(0.9f, 0.6f, 0.2f),      // 浅橙
        glm::vec3(0.2f, 0.9f, 0.6f),      // 蓝绿
        glm::vec3(0.6f, 0.2f, 0.9f),      // 紫罗兰
        glm::vec3(0.9f, 0.2f, 0.6f),      // 粉红

    };

    std::unordered_map<std::string, lyn_Infos> input_public;
    std::unordered_map<std::string, std::string> names =
    {
        {"reconstruction_output", "reconstruction_output"},
        {"robot_info_meshes", "robot_info_meshes"},
        {"robot_info_joints_points", "robot_info_joints_points"},
        {"robot_info_matrixs", "robot_info_matrixs"}
    };

    std::unordered_map<std::string, std::atomic<lyn_step>> step;
    std::unordered_map<std::string, std::condition_variable> condition_variable;
    std::unordered_map<std::string, std::thread > thread;
    double get_distance(cv::Point3f p);

    int index_min = -1;
};
#endif // LYN_GL_H
