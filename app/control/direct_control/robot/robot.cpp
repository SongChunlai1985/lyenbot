#include <robot.h>
#include <GLFW/glfw3.h>
robot::robot()
{
    step["robot_meshes_data"].store(lyn_step_free);
    step["robot_info_joints_points"].store(lyn_step_free);
    step["joints_value"].store(lyn_step_free);
}

void robot::get_meshes(components Components, lyn_Infos &robot_info_meshes, int depth)             // 一次
{
    std::string indent(ulong(depth * 4), ' ');
#if 1
    logger << " 连杆名称: " << Components.link.name
           << " 关节名称: " << Components.joint.name
           // << " 关节子连杆: " << Components.joint.child_link_name                                    // 本连杆
           << " 关节类型: " << jointTypeString[Components.joint.type]
           << " 父连杆: " << Components.joint.parent_link_name
           << " 下限: " << Components.joint.limits_lower
           << " 上限: " << Components.joint.limits_upper
           << " 最大速度: " << Components.joint.limits_velocity
           << " 最大力/力矩: " << Components.joint.limits_effort

           << " 位置 (相对于父连杆):"
           << Components.joint.Meta[env_components].position << " m"
           << " 姿态 (相对于父连杆，rpy 弧度):"
           << Components.joint.Meta[env_components].attitude_angle
           << " 质量: " << Components.link.mass << " kg"
           << " 重心坐标 (xyz): " << Components.link.Meta.position << " m"
           << " xx: " << Components.link.ixx << " kg·m²"
           << " yy: " << Components.link.iyy << " kg·m²"
           << " zz: " << Components.link.izz << " kg·m²"
           << " xy: " << Components.link.ixy << " kg·m²"
           << " xz: " << Components.link.ixz << " kg·m²"
           << " yz: " << Components.link.iyz << " kg·m²"
           << " rgba: " << Components.link.visual_material_color << std::endl;
#endif
    cv::Mat visual_vertexData = Components.link.visual_vertexData;
    lyn_Info mesh;
    mesh[Components.link.name] = std::move(visual_vertexData);
    robot_info_meshes.push_back(std::move(mesh));
    for (std::pair<std::string, components> comp : Components.Components)                          // 递归遍历子组件
    {
        logger << indent << " Subcomponent: " << comp.first << std::endl;                          // 键
        get_meshes(comp.second, robot_info_meshes, depth + 1);                                     // 值
    }
}

void robot::key_control()
{
    if(key_press.load() == GLFW_KEY_F7)
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "示教模式" << std::endl;
        joint_source = "can_data_mat_infos";
        drop_expired_info();
    }

    if(key_press.load()  == GLFW_KEY_F8)
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "直接控制模式" << std::endl;
        joint_source = "direct_output_robot";
        drop_expired_info();
    }

    if(key_press.load() == GLFW_KEY_F9)
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "RL模式" << std::endl;
        joint_source = "onnx_output_public";
        drop_expired_info();
    }
    key_press.store(0);
}

int robot::run(int argc, char *argv[])
{
    std::ignore = argc;
    std::ignore = argv;

    logger.run("robot.log", false);
    cv::Mat rotate_matrix = get_rotate_matrix(cv::Point3d(-90 ANGLE, -90 ANGLE, 0),
                                              meta(), Rotate_Sequence_XYZ);
    Components = Lyn_urdf_parser.parser(urdf_file);
    get_meshes(Components, output_public["robot_info_meshes"], 0);
    assembly(Components);
    angles.resize(500);

    cv::Mat onnx_output(1, 500, CV_32FC1);
    int index = 0;
    meta IMU;
    Components.joint.Meta[env_run_axis] = IMU;
    cv::Mat robot_info_matrix = IMU.get_transformation_with_attitude_angle() * rotate_matrix;
    lyn_Info Info;
    Info["robot_info_matrix"] = robot_info_matrix;
    output_public["robot_info_matrixs"].push_back(Info);
    cv::Mat this_matrix = robot_info_matrix;
    update_robot_angles(Components, index, angles, onnx_output);
    lyn_Info info_joints_points;
    info_joints_points["robot_info_joints_points"] = Points;
    output_public["robot_info_joints_points"].push_back(info_joints_points);
    angles.resize(uint(index));

    thread = std::thread([=]()                                                                     // 这个进程只能用lambda
    {
        {
            std::unique_lock<std::mutex> lock(mutex);
            condition_variable["robot_info_meshes"].wait(lock, [=]()
            { return step["robot_info_meshes"].load() == lyn_step_updating; });
        }
        step["robot_info_meshes"].store(lyn_step_update_complete);

        cv::Mat joint_angles;
        while (running)
        {
            key_control();
            wait_get(joint_source, joint_angles);

            if (step["robot_info_matrixs"].load() == lyn_step_updating)
            {
                int index = 0;
                meta IMU;
                Components.joint.Meta[env_float_axis] = IMU;
                cv::Mat robot_info_matrix = IMU.get_transformation_with_attitude_angle();
                lyn_Info Info;
                Info["robot_info_matrix"] = std::move(robot_info_matrix);
                output_public["robot_info_matrixs"].push_back(std::move(Info));
                update_robot_angles(Components, index, angles, joint_angles);
//                print_mat_formatted(joint_angles, 2);
                index = 0;
                get_robot_float_transforms(Components, output_public["robot_info_matrixs"], 0, index, angles);
                index = 0;
                get_joints_points(Components, Points, index, gl_rotate_matrix);

                get_robot_info(Components, output_public["robot_info_matrixs"]);
                step["robot_info_matrixs"].store(lyn_step_update_complete);

                lyn_Info info_joints_points;
                info_joints_points["robot_info_joints_points"] = std::move(Points);

                if (step["robot_info_joints_points"].load() == lyn_step_updating)
                {
                    output_public["robot_info_joints_points"].push_back(info_joints_points);
                    step["robot_info_joints_points"].store(lyn_step_update_complete);
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(5 * 1000));
        }
    });
    return 0;
}

void robot::assembly(components &Comp_parent)
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)                         // 递归遍历子组件
    {
        std::string Comp_this_name = comp.first;
        components &Comp_this = Comp_parent.Components[Comp_this_name];

        Comp_this.joint.Meta[env_base] =
                Comp_parent.joint.Meta[env_base] *
                Comp_this.joint.Meta[env_components];

        assembly(Comp_this);
    }
}

void robot::update_robot_angles(components &Comp_parent,
                                int &index,
                                std::vector<double> &angles,
                                cv::Mat &onnx_output)
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)
    {
        components &Comp_this = Comp_parent.Components[comp.first];
        angles[ulong(index)] = double(onnx_output.at<float>(0, index));
        index ++;
        update_robot_angles(Comp_this, index, angles, onnx_output);
    }
}

void robot::get_robot_float_transforms(components &Comp_parent,
                       lyn_Infos &robot_info,
                       int depth,
                       int &index,
                       std::vector<double> &angles)                                                // 计算所有轴的矩阵
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)
    {
        components &Comp_this = Comp_parent.Components[comp.first];

        meta &axis = Comp_this.joint.Meta[env_base_axis];                                          // 在设计位置构造轴
        meta &Meta_env_components = Comp_this.joint.Meta[env_components];                          // 在设计位置构造轴

        axis.position = Comp_this.joint.Meta[env_base].position;
        axis.point_z = Comp_this.joint.Meta[env_base].position +
                Comp_this.joint.Meta[env_components_axis].point_z;                                 // x、y是隐式的，无法计算

        axis.attitude_angle = cv::Point3d(0, 0, angles[ulong(index)]);
        axis.matrix =
                get_rotate_matrix(Meta_env_components.attitude_angle, meta()) *                    // urdf joint origin rpy 旋转
                get_translation(Comp_this.joint.Meta[env_base].position -
                                      Comp_parent.joint.Meta[env_base].position) *
                matrix_rotate_line(axis.point_z - axis.position, axis.attitude_angle.z);           // 使用部件的拼装位置和绕单位向量旋转，计算轴的矩阵, 为什么减去上一个组件的position尚不清楚
#if 0
        logger << " axis.position: " << axis.position
               << " axis.point_z: " << axis.point_z
               << " axis.attitude_angle.z: " << axis.attitude_angle.z
               << " name: " << Comp_this.name
               << " axis.matrix:" << "\n"
               << axis.matrix
               << "\n" << std::endl;
#endif
        Comp_this.joint.Meta[env_float].matrix =
                Comp_parent.joint.Meta[env_float].matrix *
                Comp_this.joint.Meta[env_base_axis].matrix                                         // 连续乘轴矩阵
                ;

        index ++;
        get_robot_float_transforms(Comp_this, robot_info, depth + 1, index, angles);
    }
}

void robot::get_robot_run_transforms(components &Comp_parent,
                       lyn_Infos &robot_info,
                       int &index,
                       std::vector<double> &angles)
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)
    {
        components &Comp_this = Comp_parent.Components[comp.first];

        get_robot_run_transforms(Comp_this, robot_info, index, angles);
    }
}

void robot::get_robot_info(components &Comp_parent, lyn_Infos &robot_info)
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)
    {
        components &Comp_this = Comp_parent.Components[comp.first];
        lyn_Info Info;
        Info["robot_info_matrix"] = Comp_this.joint.Meta[env_float].matrix;                        // 拼装好的
        robot_info.push_back(Info);
        get_robot_info(Comp_this, robot_info);
    }
}

void robot::get_joints_points(components &Comp_parent,
                              cv::Mat &Points,
                              int &index,
                              const cv::Mat &gl_rotate_matrix)
{
    for (std::pair<std::string, components> comp : Comp_parent.Components)
    {
        components &Comp_this = Comp_parent.Components[comp.first];

        draw_meta(gl_rotate_matrix *
                  Comp_this.joint.Meta[env_float].matrix *
                  meta(), Points, 100);                                                            // R * meta(): 矩阵的可视化 length: 可视化的边长
//        logger << "index: " << index << "\n"
//               << gl_rotate_matrix * Comp_this.joint.Meta[env_float].matrix
//               << "\n" << std::endl;
        index ++;
        get_joints_points(Comp_this, Points, index, gl_rotate_matrix);
    }
}

int robot::drop_expired_info()
{
    step["can_data_mat_infos"].store(lyn_step_copy_complete);
    step["direct_output_robot"].store(lyn_step_copy_complete);
    step["onnx_output_public"].store(lyn_step_copy_complete);
    return 0;
}

void robot::wait_get(std::string name, cv::Mat &joint_angles)
{
    std::unique_lock<std::mutex> lock(mutex);
    condition_variable[name].wait(lock, [=](){ return step[name].load() == lyn_step_copying; });
    if(std::holds_alternative<cv::Mat>(input_public[name][0]["joint_angles"]))
    {
        joint_angles = std::move(std::get<cv::Mat>(input_public[name][0]["joint_angles"]));
    } else { logger << "std::get: wrong index for variant, matrix" << std::endl; }
    step[name].store(lyn_step_copy_complete);
}

int robot::work(lyn_info &info)
{
    if(info.key_press > 0) { key_press.store(info.key_press); }
    info.unload(names["can_data_mat_infos"], step, input_public, condition_variable);
    info.unload(names["direct_output_robot"], step, input_public, condition_variable);
    info.unload(names["onnx_output_public"], step, input_public, condition_variable);

    info.load(names["robot_info_meshes"], step, output_public);
    if(step["robot_info_meshes"].load() == lyn_step_updating)
    {
        condition_variable["robot_info_meshes"].notify_one();
    }

    info.load(names["robot_info_matrixs"], step, output_public);
    info.load(names["robot_info_joints_points"], step, output_public);

#if 0
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "robot::work: "
              << joint_source << std::endl;
#endif
    return 0;
}
