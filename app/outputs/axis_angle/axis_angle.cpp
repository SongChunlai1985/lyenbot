#include <axis_angle.h>
#include <math.h>
#include <calculate_ankle_motor_joint.h>
#include <GLFW/glfw3.h>
axis_angle::axis_angle()
{
    motor_UIDs = cv::Mat(1, 128, CV_32FC1);
}

void sleep_for(long __rtime){std::this_thread::sleep_for(std::chrono::microseconds(__rtime));}

int axis_angle::send(std::string joint_name, std::string cmd, std::string cmd2, double value, double value2)
{
    logger << "can" << std::to_string(can_name2interface[joint_name]) << " "
           << joint_name << Can[0].can_name_white[joint_name] << " "
           << cmd << " " << cmd2 << " " << value << std::endl;

#if 0
    {
        if(joint_name == "LAnklePitch" || joint_name == "LAnkleRoll" ||
                joint_name == "RAnklePitch" || joint_name == "RAnkleRoll")
        {
            std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                      << joint_name << " = " << value  * 180.0 / 3.1415926 << std::endl;
        }
    }
#endif

    return Can[can_name2interface[joint_name]].send(joint_name, cmd, cmd2, value, value2);
}

const double degree_angle =  3.141592654 / 180;

void can_up()
{
    system("echo \"jokio.369*\"| sudo -S ip link set can0 down");
    system("echo \"jokio.369*\"| sudo -S ip link set can0 type can bitrate 1000000");
    system("echo \"jokio.369*\"| sudo -S ip link set can0 up");
    system("echo \"jokio.369*\"| sudo -S ip link set can1 down");
    system("echo \"jokio.369*\"| sudo -S ip link set can1 type can bitrate 1000000");
    system("echo \"jokio.369*\"| sudo -S ip link set can1 up");
    system("echo \"jokio.369*\"| sudo -S ip link set can2 down");
    system("echo \"jokio.369*\"| sudo -S ip link set can2 type can bitrate 1000000");
    system("echo \"jokio.369*\"| sudo -S ip link set can2 up");
    system("echo \"jokio.369*\"| sudo -S ip link set can3 down");
    system("echo \"jokio.369*\"| sudo -S ip link set can3 type can bitrate 1000000");
    system("echo \"jokio.369*\"| sudo -S ip link set can3 up");
}

void axis_angle::read_power(int microsecond, int sample_rate)
{
    int sample_total = int(microsecond / 1000000.0 * sample_rate);
    int sample_time = 1000 * 1000 / sample_rate;
    for (int i = 0; i < sample_total; ++i)
    {
        send_frame("读取命令", "关节当前母线电压&母线电流值", 0);
        logger << std::endl;
        sleep_for(sample_time);
    }
    logger << std::endl;
}

int axis_angle::send_frames(std::string cmd, std::string cmd2, std::vector<float> dof_frame)
{
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        if(send(joint_name, cmd, cmd2, double(dof_frame[index] * motor_direction[joint_name] )))
        {
            sleep_for(100);
        } else
        {
            logger << "动作帧发送失败！";
        }
    }
    return 0;
}

int axis_angle::send_frame(std::string cmd, std::string cmd2, double value)
{
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        send(joint_name, cmd, cmd2, value);
        sleep_for(1000);
    }
    return 0;
}

int axis_angle::test_motor(double min, double max, double interval)
{
//    send_frame("读取命令", "执行器输出轴当前绝对位置、速度、力矩", 0); sleep_for(5 * 1000);
//    send_frame("读取命令", "设置关节目标位置", 0); sleep_for(10 * 1000);
    send_frame("快写命令", "设置关节目标位置", min); sleep_for(long(interval * 1000 * 1000));
//    send_frame("读取命令", "执行器输出轴当前绝对位置、速度、力矩", 0); sleep_for(5 * 1000);    //read_power(500 * 1000); // 写入命令 快写命令
//    send_frame("读取命令", "设置关节目标位置", 0); sleep_for(10 * 1000);
    send_frame("快写命令", "设置关节目标位置", max); sleep_for(long(interval * 1000 * 1000)); //read_power(500 * 1000); //
    return 0;
}

cv::Mat axis_angle::axis_angle2motor_angle(cv::Mat joint_angles)
{
    cv::Mat new_joint_angles = joint_angles.clone();
#if 0
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                  << "axis_angle2motor_angle Left: "
                  << "LAnklePitch = "
                  << std::setw(10) << joint_angles.at<float>
                     (0, int(can_name_to_mat_index_map["LAnklePitch"])) * 180.0f / 3.1415926f << " "
                  << "LAnkleRoll = "
                  << std::setw(10) << joint_angles.at<float>
                     (0, int(can_name_to_mat_index_map["LAnkleRoll"])) * 180.0f / 3.1415926f << " "
                  << std::endl;
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                  << "axis_angle2motor_angle Right: "
                  << "RAnklePitch = "
                  << std::setw(10) << joint_angles.at<float>
                     (0, int(can_name_to_mat_index_map["RAnklePitch"])) * 180.0f / 3.1415926f << " "
                  << "RAnkleRoll = "
                  << std::setw(10) << joint_angles.at<float>
                     (0, int(can_name_to_mat_index_map["RAnkleRoll"])) * 180.0f / 3.1415926f << " "
                  << std::endl;
    }
#endif

    std::vector<float> ankle_motor_jointL = calculate_ankle_motor_joint
            (joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnklePitch"])),
            joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnkleRoll"])), 1);

    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnklePitch"])) = ankle_motor_jointL[0];
    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnkleRoll"])) = ankle_motor_jointL[1];

    std::vector<float> ankle_motor_jointR = calculate_ankle_motor_joint
            (joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnklePitch"])),
            joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnkleRoll"])), 0);

    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnklePitch"])) = ankle_motor_jointR[0];
    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnkleRoll"])) = ankle_motor_jointR[1];

    return new_joint_angles;
}

void axis_angle::broadcast(std::string cmd, std::string cmd2, double value)
{
    Can[0].send("广播", cmd, cmd2, value); sleep_for(10 * 1000);
    Can[1].send("广播", cmd, cmd2, value); sleep_for(10 * 1000);
    Can[2].send("广播", cmd, cmd2, value); sleep_for(10 * 1000);
    Can[3].send("广播", cmd, cmd2, value); sleep_for(10 * 1000);
};

void axis_angle::enable_all_motor()
{
    logger_control << "[" << getCurrentTimeWithMicroseconds() << "] " << "使能机器人全部关节" << std::endl;

    Can[0].send("广播", "快写命令", "使能/失能状态", 1); sleep_for(10 * 1000);
    Can[1].send("广播", "快写命令", "使能/失能状态", 1); sleep_for(10 * 1000);
    Can[2].send("广播", "快写命令", "使能/失能状态", 1); sleep_for(10 * 1000);
    Can[3].send("广播", "快写命令", "使能/失能状态", 1); sleep_for(10 * 1000);

    Can[0].send("广播", "快写命令", "设置工作模式", 5); sleep_for(10 * 1000);
    Can[1].send("广播", "快写命令", "设置工作模式", 5); sleep_for(10 * 1000);
    Can[2].send("广播", "快写命令", "设置工作模式", 5); sleep_for(10 * 1000);
    Can[3].send("广播", "快写命令", "设置工作模式", 5); sleep_for(10 * 1000);
}

void axis_angle::disable_all_motor()
{
    logger_control << "[" << getCurrentTimeWithMicroseconds() << "] " << "失能机器人全部关节" << std::endl;
    Can[0].send("广播", "写入命令", "使能/失能状态", 0); sleep_for(10 * 1000);
    Can[1].send("广播", "写入命令", "使能/失能状态", 0); sleep_for(10 * 1000);
    Can[2].send("广播", "写入命令", "使能/失能状态", 0); sleep_for(10 * 1000);
    Can[3].send("广播", "写入命令", "使能/失能状态", 0); sleep_for(10 * 1000);
}

void axis_angle::enable_some_motor()
{
    logger_control << "[" << getCurrentTimeWithMicroseconds() << "] " << "使能机器人部分关节" << " ";

    for(std::string motor : motor_list)
    {
        logger_control << motor << " ";
        send(motor, "写入命令", "使能/失能状态", 1); sleep_for(10 * 1000);
        send(motor, "读取命令", "使能/失能状态", 1); sleep_for(10 * 1000);
        send(motor, "写入命令", "设置工作模式", 5); sleep_for(10 * 1000);
    }
    sleep_for(200 * 1000);
    logger_control << std::endl;
}

void axis_angle::disable_some_motor()
{
    logger_control << "[" << getCurrentTimeWithMicroseconds() << "] " << "失能机器人部分关节" << " ";

    for(std::string motor : motor_list)
    {
        logger_control << motor << " ";
        send(motor, "写入命令", "使能/失能状态", 0); sleep_for(10 * 1000);
    }
    sleep_for(200 * 1000);
    logger_control << std::endl;
}

void axis_angle::read_position_ring_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        fs[joint_name + "_KP"] >> position_ring[joint_name + "_KP"];
        fs[joint_name + "_KI"] >> position_ring[joint_name + "_KI"];
        send(joint_name, "写入命令", "位置环kp", double(position_ring[joint_name + "_KP"])); sleep_for(10 * 1000);
        send(joint_name, "写入命令", "位置环ki", double(position_ring[joint_name + "_KI"])); sleep_for(10 * 1000);
        send(joint_name, "写入命令", "数据保存", double(motor_UIDs.at<float>(0, int(can_name_to_mat_index_map[joint_name]))), 1); sleep_for(10 * 1000);
    }
    fs.release();
}

void axis_angle::write_position_ring_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        fs << joint_name + "_KP" << position_ring[joint_name + "_KP"];
        fs << joint_name + "_KI" << position_ring[joint_name + "_KI"];
    }
    fs.release();
}

void axis_angle::read_PD_mode_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        fs[joint_name + "_KP"] >> PD_mode[joint_name + "_KP"];
        fs[joint_name + "_KD"] >> PD_mode[joint_name + "_KD"];
        send(joint_name, "写入命令", "PD模式kp", double(PD_mode[joint_name + "_KP"])); sleep_for(10 * 1000);
        send(joint_name, "写入命令", "PD模式kd", double(PD_mode[joint_name + "_KD"])); sleep_for(10 * 1000);
        send(joint_name, "写入命令", "数据保存", double(motor_UIDs.at<float>(0, int(can_name_to_mat_index_map[joint_name]))), 1); sleep_for(10 * 1000);
    }
    fs.release();
}

void axis_angle::write_PD_mode_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
    {
        std::string joint_name = mat_index_can_name_to_map[index];
        fs << joint_name + "_KP" << PD_mode[joint_name + "_KP"];
        fs << joint_name + "_KD" << PD_mode[joint_name + "_KD"];
    }
    fs.release();
}

void axis_angle::key_control()
{
    if(key_press.load() == GLFW_KEY_F1)
    {
        key_press.store(0);
        enable_some_motor();
        all_motor_enable.store(1);
    }

    if(key_press.load() == GLFW_KEY_F2)
    {
        key_press.store(0);
        disable_some_motor();
        all_motor_enable.store(0);
    }

    if(key_press.load() == GLFW_KEY_F5)
    {
        key_press.store(0);
        enable_all_motor();
        enable_all_motor();
        enable_all_motor();
//        write_position_ring_config("/home/lyenbot/config/KP_KI_config.yml");
//        read_position_ring_config("/home/lyenbot/config/KP_KI_config.yml");
//        write_PD_mode_config("/home/lyenbot/config/PD_mode_config.yml");
        read_PD_mode_config("/home/lyenbot/config/PD_mode_config.yml");
        send_frame("读取命令", "位置环kp", 0); sleep_for(10 * 1000);
        send_frame("读取命令", "位置环ki", 0); sleep_for(10 * 1000);
        send_frame("读取命令", "PD模式kp", 0); sleep_for(10 * 1000);
        send_frame("读取命令", "PD模式kd", 0); sleep_for(10 * 1000);

        send_frame("读取命令", "使能/失能状态", 1); sleep_for(10 * 1000);
        send_frame("读取命令", "设置工作模式", 1); sleep_for(10 * 1000);
        all_motor_enable.store(1);
    }

    if(key_press.load() == GLFW_KEY_F6)
    {
        key_press.store(0);
        disable_all_motor();
        disable_all_motor();
        disable_all_motor();
        send_frame("读取命令", "使能/失能状态", 1); sleep_for(10 * 1000);
        send_frame("读取命令", "设置工作模式", 1); sleep_for(10 * 1000);
        all_motor_enable.store(0);
    }
}

int axis_angle::run(int argc, char *argv[])
{
    logger.run("axis_angle.log", false);
    logger_control.run("axis_angle_control.log", true);
    can_up();
    sleep_for(500 * 1000);
    for (int i = 0; i < 4; ++i)
    {
        Can[i].name = "can" + std::to_string(i);
        Can[i].run(argc, argv);
    }
    sleep_for(500 * 1000);
    thread[0] = std::thread([=]()
    {
        logger_control << "检测机器人关节状态" << std::endl;
        send_frame("读取命令", "软件版本号", 0);     sleep_for(10 * 1000);
        send_frame("读取命令", "最小位置", 0);       sleep_for(10 * 1000);
        send_frame("读取命令", "最大位置", 0);       sleep_for(10 * 1000);
        send_frame("读取命令", "关节当前温度值", 0);  sleep_for(10 * 1000);
        send_frame("读取命令", "使能/失能状态", 1);   sleep_for(10 * 1000);

        send_frame("读取命令", "软件限位", 0);  sleep_for(10 * 1000);
        send_frame("读取命令", "减速比", 0);    sleep_for(10 * 1000);
        send_frame("读取命令", "位置环kp", 0);  sleep_for(10 * 1000);
        send_frame("读取命令", "位置环ki", 0);  sleep_for(10 * 1000);

        send_frame("读取命令", "UID", 0); sleep_for(10 * 1000);

        // disable_all_motor();
        // enable_all_motor();

        logger_control << "运行机器人控制线程" << std::endl;
        // if(err) return 0;
        cv::Mat joint_angles;
        std::string name = "direct_output_axis_angle";
        while (running)
        {
            get_info("motor_UIDs", "motor_UIDs", motor_UIDs);
            wait_get(name, joint_angles);
#if 0
            print_mat_formatted(axis_angle2motor_angle(joint_angles) * RAD, 2);
#endif
            if(all_motor_enable.load())
            {
                cv::Mat new_joint_angles = axis_angle2motor_angle(joint_angles);
                send_frames("快写命令", "设置关节目标位置", new_joint_angles);
#if 0
                std::cout  << "[" << getCurrentTimeWithMicroseconds() << "] "
                           << "send_frames " ;
                print_mat_formatted(new_joint_angles * RAD, 4);
#endif
            }
            // test_motor(0 * degree_angle, 8 * degree_angle, 5);
//            logger << std::endl;
        }
    });

    thread[1] = std::thread([=]()
    {
        sleep_for(2000 * 1000);
        logger_control << "检测输出轴位置中" << std::endl;
        for (int i = 0; i < 200; ++i)
        {
            send_frame("读取命令", "输出轴位置", 0);                                                        //消耗 9 ms
            sleep_for(5 * 1000);
#if 0
            std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                      << "axis_angle: " << i << std::endl;
#endif
        }
        logger_control << "检测执行器输出轴当前绝对位置、速度、力矩中" << std::endl;
        while (running)
        {
            key_control();
            if(!all_motor_enable.load())
            {
                send_frame("读取命令", "执行器输出轴当前绝对位置、速度、力矩", 0);
                logger << std::endl;
            };
            sleep_for(5 * 1000);
        }
    });
    return 0;
}

void axis_angle::wait_get(std::string name, cv::Mat &joint_angles)
{
    {
        std::unique_lock<std::mutex> lock(mutex[name]);
        condition_variable[name].wait(lock, [=](){ return step[name].load() == lyn_step_copying; });
    }
    if(std::holds_alternative<cv::Mat>(input_public[name][0]["joint_angles"]))
    {
        joint_angles = std::move(std::get<cv::Mat>(input_public[name][0]["joint_angles"]));
    } else { logger << "std::get: wrong index for variant, matrix" << std::endl; }
    step[name].store(lyn_step_copy_complete);
}

void axis_angle::get_info(const std::string &name, std::string Info_name, cv::Mat &info_mat)
{
    if(step[name].load() == lyn_step_copying)
    {
        std::unique_lock<std::mutex> lock(mutex[name]);
        lyn_Infos infos = input_public[name];
        step[name].store(lyn_step_copy_complete);

        if(infos.size() && std::holds_alternative<cv::Mat>(infos[0][Info_name]))
        {
            info_mat = std::move(std::get<cv::Mat>(infos[0][Info_name]));
        } else { logger << "std::get: wrong index for variant, Hi12_Imu_data_mat" << std::endl; }
    }
}

int axis_angle::work(lyn_info &info)
{
    if(info.key_press > 0) { key_press.store(info.key_press); }
    info.unload(names["direct_output_axis_angle"], step, input_public, condition_variable);
    info.unload(names["motor_UIDs"], step, input_public, condition_variable);
    return 0;
}

