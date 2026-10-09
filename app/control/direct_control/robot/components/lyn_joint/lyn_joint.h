#ifndef LYN_JOINT_H
#define LYN_JOINT_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <lyn_meta.h>
enum lyn_joint_environment
{
    env_components,                              // 散件态，位姿相对世界原点
    env_base,                                    // 基础态，立正姿态
    env_float,                                   // 漂浮态，固定base
    env_run,                                     // 运动态，浮动base

    env_components_axis,
    env_base_axis,
    env_float_axis,
    env_run_axis,
};

class lyn_joint
{
public:
    lyn_joint();
    meta Meta[8];                                // 4个形态，第四维度2个在三维空间里位置重合的元, 控制的核心就是元的矩阵的控制
    std::string name;
    std::string parent_link_name;
    std::string child_link_name;
    int type;

    double dynamics_damping = 0.0;
    double dynamics_friction = 0.0;

    double limits_lower = 0.0;
    double limits_upper = 0.0;
    double limits_effort = 0.0;
    double limits_velocity = 0.0;

    double soft_upper_limit = 0.0;
    double soft_lower_limit = 0.0;
    double k_position = 0.0;
    double k_velocity = 0.0;

};
#endif // LYN_JOINT_H
