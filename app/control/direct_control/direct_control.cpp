#include <direct_control.h>
#include <lyn_can.h>
#include <GLFW/glfw3.h>
#include <walk_main.h>

direct_control::direct_control()
{
}

void sleep_for(long __rtime);

std::vector<std::string> split(const std::string& str, const std::string& delimiters = "  ")
{
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = 0;

    while ((end = str.find_first_of(delimiters, start)) != std::string::npos)
    {
        if (end != start)
        {
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + 1;
    }

    if (start < str.length())
    {
        tokens.push_back(str.substr(start));
    }
    return tokens;
}

std::vector<std::vector<float>> direct_control::read_aciton_file(std::string filename)
{
    std::ifstream control_log_file;
    control_log_file.open(filename);
    std::vector<std::vector<float>> action_data;
    std::string line;
    while(std::getline(control_log_file, line))
    {
        std::vector<std::string> frame_data;
        if(line.find("ctrl") != std::string::npos)
        {
            frame_data = split(split(line, ":")[1]);
            std::vector<float> action_frame;
            for (uint i = 0; i < frame_data.size(); ++i)
            {
                action_frame.push_back(std::stof(frame_data[i]));
            }
            action_data.push_back(action_frame);
        }
    }
    return action_data;
}

double direct_control::make_angle(double current_time_tickt, double angle_min, double angle_max, double frequency)
{
    double mid = (angle_min + angle_max) / 2.0;
    double range = (angle_max - angle_min) / 2.0;
    double t = current_time_tickt / 1000.0;
    return mid + range * std::sin(2 * M_PI * frequency * t);
}

void direct_control::shake_joint(std::string name,
                                 double current_time_tickt,
                                 double angle_min,
                                 double angle_max,
                                 double frequency)
{
    direct_output.at<float>(0, int(can_name_to_mat_index_map[name]))
            = float(make_angle(current_time_tickt, angle_min * 3.1415926 / 180, angle_max * 3.1415926 / 180, frequency));
}

void direct_control::shake_joint_in_time(
        double start_time_tickt,
        double end_time_tickt,std::string name,
        double current_time_tickt,
        double angle_min,
        double angle_max,
        double frequency)
{
    if(current_time_tickt > start_time_tickt && current_time_tickt <= end_time_tickt)
        shake_joint(name, current_time_tickt, angle_min, angle_max, frequency);
}

void direct_control::shake_all_joint(double current_time_tickt)
{
    shake_joint_in_time( 0, 10000000, "RHipPitch",      current_time_tickt,    -10,    10,    0.25);
    shake_joint_in_time( 0, 10000000, "RHipYaw",        current_time_tickt,      0,    10,    0.25);
    shake_joint_in_time( 0, 10000000, "RHipRoll",       current_time_tickt,    -10,     0,    0.25);
    shake_joint_in_time( 0, 10000000, "RKneePitch",     current_time_tickt,     20,    10,    0.25);
    shake_joint_in_time( 0, 10000000, "RAnklePitch",    current_time_tickt,    -20,    -5,    0.25);
    shake_joint_in_time( 0, 10000000, "RAnkleRoll",     current_time_tickt,      0,     1,    0.25);

    shake_joint_in_time( 0, 10000000, "LHipPitch",      current_time_tickt,     10,   -10,    0.25);
    shake_joint_in_time( 0, 10000000, "LHipYaw",        current_time_tickt,    -10,     0,    0.25);
    shake_joint_in_time( 0, 10000000, "LHipRoll",       current_time_tickt,      0,    10,    0.25);
    shake_joint_in_time( 0, 10000000, "LKneePitch",     current_time_tickt,     20,    10,    0.25);
    shake_joint_in_time( 0, 10000000, "LAnklePitch",    current_time_tickt,    -20,    -5,    0.25);
    shake_joint_in_time( 0, 10000000, "LAnkleRoll",     current_time_tickt,      0,     1,    0.25);

    shake_joint_in_time( 0, 10000000, "AbdomeYaw",      current_time_tickt,      0,     10,   0.25);

    shake_joint_in_time( 0, 10000000, "RShoulderPitch", current_time_tickt,     10,    -10,   0.25);
    shake_joint_in_time( 0, 10000000, "RShoulderRoll",  current_time_tickt,    -10,      0,   0.25);
    shake_joint_in_time( 0, 10000000, "RElbowYaw",      current_time_tickt,      0,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "RElbowRoll",     current_time_tickt,      0,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "RElbowPitch",    current_time_tickt,      0,     10,   0.25);

    shake_joint_in_time( 0, 10000000, "LShoulderPitch", current_time_tickt,    -10,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "LShoulderRoll",  current_time_tickt,      0,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "LElbowYaw",      current_time_tickt,      0,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "LElbowRoll",     current_time_tickt,      0,     10,   0.25);
    shake_joint_in_time( 0, 10000000, "LElbowPitch",    current_time_tickt,      0,     10,   0.25);
}

void direct_control::action_playback(uint index,  std::vector<std::vector<float>> &action_data)
{
    if(index < action_data.size())
    {
        for (uint i = 0; i < action_data[index].size(); ++i)
        {
            direct_output.at<float>(0, int(i)) = action_data[index][i];
        }
#if 0
        std::cout  << "[" << getCurrentTimeWithMicroseconds() << "] "
                   << "action_playback: index = " << index << " / " << action_data.size() << " "
                   << direct_output
                   << std::endl;
#endif
    }
}

void direct_control::move_joint_in_time(double start_time_tickt,
        double end_time_tickt,
        uint index,
        double current_time_tickt,
        double angle_start,
        double angle_end)
{
    if(current_time_tickt > start_time_tickt && current_time_tickt <= end_time_tickt)
    {
        double duration = end_time_tickt - start_time_tickt;
        double elapsed = current_time_tickt - start_time_tickt;
        double t = elapsed / duration;
        double eased_t = t * t * (3.0 - 2.0 * t);
        double angle_current = angle_start + (angle_end - angle_start) * eased_t;
        direct_output.at<float>(0, int(index))
                = float(angle_current / 180 * 3.141592654);
    }
}

void direct_control::move_to_posture(double current_time_tickt,
                                     double start_time_tickt,
                                     double end_time_tickt,
                                     std::vector<double> angle_start,
                                     std::vector<double> angle_end)
{
    for (uint index = 0; index < 23; index++)
    {
        move_joint_in_time( start_time_tickt, end_time_tickt, index, current_time_tickt, angle_start[index], angle_end[index]);
    }
}

void direct_control::read_posture(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["angle_posture"] >> angle_posture;
    fs["start_time_tickts"] >> start_time_tickts;
    fs["end_time_tickts"] >> end_time_tickts;
    fs.release();
}

double get_max(cv::Mat direct_output, cv::Mat joint_angles_clone)
{
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(direct_output - joint_angles_clone, &minVal, &maxVal, &minLoc, &maxLoc);
    return maxVal;
}

void direct_control::attention(std::string &action,
                               double &time_speed ,
                               double &current_time_tickt,
                               double &send_intervel,
                               int &attention_is_start,
                               cv::Mat &joint_angles_clone,
                               cv::Mat &start_posture,
                               cv::Mat &end_posture)
{
    if(!attention_is_start)
    {
        start_posture = joint_angles_clone.clone() * RAD;
        end_posture.setTo(0);
//        end_posture.at<float>(0, int(can_name_to_mat_index_map["LHipPitch"])) = 0.0f;            // -12.5          @init_z_offset = 10
//        end_posture.at<float>(0, int(can_name_to_mat_index_map["RHipPitch"])) = -12.2f;            // -12.5

        current_time_tickt = 0;
        std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] action == 立正" ;
        std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] start_posture:";
        print_mat_formatted(start_posture, 2);
        std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "]   end_posture:";
        print_mat_formatted(end_posture, 2);
        std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] direct_output:";
        print_mat_formatted(direct_output * RAD, 2);
        std::cout << std::endl << std::setw(10) << long(current_time_tickt);

        attention_is_start = 1;
        std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] direct_output:";
    }

    current_time_tickt = current_time_tickt + send_intervel * time_speed;
    move_to_posture(current_time_tickt, 0, 2000, start_posture, end_posture);          // 准备动作  start_time_tickt必须是0
#if 0
    print_mat_formatted(direct_output * RAD, 2);
#endif
    if(current_time_tickt > 2000)
    {
        attention_is_start = 0;
        action = "待命";
    }
}

void direct_control::action_walk(double &time_speed,
                                 int &ready_to_run,
                                 int &going_to_ready_target ,
                                 double &current_time_tickt,
                                 double &send_intervel,
                                 cv::Mat &joint_angles_clone,
                                 cv::Mat &start_posture,
                                 cv::Mat &end_posture,
                                 cv::Mat &Frame,
                                 Walk &walk)
{
    if(!ready_to_run)
    {
        if((!going_to_ready_target))
        {
            start_posture = joint_angles_clone.clone() * RAD;
            walk.walking_init();
            walk.set_m_Time(0);
            walk.start();
            end_posture = walk.GetFrameAtTimeMs().clone() * RAD;
            walk.walking.last_step = -1;                                                   // 为了让第二帧也能改变state的值
            going_to_ready_target = 1;

            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] 切换到指定姿势:" ;
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] start_posture:";
            print_mat_formatted(start_posture, 2);
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "]   end_posture:";
            print_mat_formatted(end_posture, 2);
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] direct_output:";
            print_mat_formatted(direct_output * RAD, 2);
            std::cout << std::endl << std::setw(10) << long(current_time_tickt);
        }
        current_time_tickt = current_time_tickt + send_intervel * time_speed;
        move_to_posture(current_time_tickt, 0, 1800, start_posture, end_posture);          // 准备动作
        if(current_time_tickt > 1800)
        {
            going_to_ready_target = 0;
            ready_to_run = 1;
            current_time_tickt = 0;
        }
    }
    if(ready_to_run)
    {
        Frame = walk.GetFrameAtTimeMs();
        direct_output = Frame;
        if(walk.get_state())
        {
            key_press.store(GLFW_KEY_F12);
        }
    }

#if 0
    //            print_mat_formatted(joint_angles_clone * RAD, 2);
    //            print_mat_formatted(start_posture, 2);
    //            print_mat_formatted(end_posture, 2);

    //            -3.01       1.97      35.74      13.12      -8.59     -24.62      -1.96      -1.15      -8.84      11.59     -11.94       2.74      -1.22       0.32      -0.02      16.98       0.00       0.00       0.15       0.00       0.00       0.00       0.00
    //           -12.00      -2.00       0.00      28.00     -15.00       5.00     -12.00       2.00       0.00      28.00     -15.00      -5.00       0.00       0.00      15.00       0.00       0.00       0.00       0.00     -15.00       0.00       0.00       0.00
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << std::setw(10) << long(current_time_tickt) << std::endl;
#endif
}

void direct_control::thread_walk()
{
    direct_output.setTo(0);
    double current_time_tickt = 0.0;
    std::vector<std::vector<float>> action_data = read_aciton_file("/home/lyenbot/modles/new_control.log");
    double send_intervel = 5.0;
    std::string action = "走路";
    double time_speed = 1;                                                                         // 倍速
    int action_run = 0;
    read_posture("/home/lyenbot/modles/squat.yml");
    std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] " << action  << "线程开始";

    cv::Mat joint_angles_clone;

    int go_to_0 = 0;

    cv::Mat Hi12_Imu_data_mat(1, 128, CV_32FC1, cv::Scalar(0.0));

    int going_to_ready_target = 0;
    int ready_to_run = 0;

    cv::Mat start_posture = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));
    cv::Mat end_posture = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));
    cv::Mat Frame = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));
    int attention_is_start = 0;

    int index = 0;

    Walk walk;
    walk.set_time_msec(4.5);
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(long(send_intervel) * 1000));
        if(key_press.load() == GLFW_KEY_F10)
        {
            current_time_tickt = 0;
            read_posture("/home/lyenbot/modles/squat.yml");
            walk.walking.ctrl_Running = false;                                             // 双脚并拢
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] " << action  << "开始";
        }

        if(key_press.load() == GLFW_KEY_F8)
        {
            action = "走路";
            ready_to_run = 0;
            going_to_ready_target = 0;
            current_time_tickt = 0;
            action_run = 1;
        }

        if(key_press.load() == GLFW_KEY_F11)                                                                    // F11 回零位
        {
            action = "立正";
            attention_is_start = 0;
            current_time_tickt = 0;
            key_press.store(0);
        }

        if(key_press.load() == GLFW_KEY_F12)
        {
            action_run = action_run ? 0 : 1;
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] "
                      << (action_run ? "继续" : "暂停")
                      << " walk.get_m_Time = " << std::setw(10) << std::fixed << float(walk.get_m_Time());
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] direct_output:";
            print_mat_formatted(direct_output * RAD, 2);
        }

        if(key_press.load() == GLFW_KEY_M)
        {
            index ++;
            index = index < 23 ? index : 0;
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] " << "修改 " << index << " " << mat_index_can_name_to_map[uint(index)];
        }

        if(key_press.load() == GLFW_KEY_N)
        {
            index --;
            index = index >= 0 ? index : 23;
            std::cout << std::endl << "[" << getCurrentTimeWithMicroseconds() << "] " << "修改 " << index << " " << mat_index_can_name_to_map[uint(index)];
        }

        if(key_press.load() == GLFW_KEY_UP)
        {
            direct_output.at<float>(0, index) += 2.0f * float(DEG);
            print_mat_formatted(direct_output * RAD, 3);
            public_info();
        }

        if(key_press.load() == GLFW_KEY_DOWN)
        {
            direct_output.at<float>(0, index) -= 2.0f * float(DEG);
            print_mat_formatted(direct_output * RAD, 3);
            public_info();
        }

        key_press.store(0);

        {
            std::unique_lock<std::mutex> lock(mutex["name_joint_angles"]);
            joint_angles_clone = joint_angles.clone();
        }

        wait_get("imu_direct_control", Hi12_Imu_data_mat);

        if(action == "立正")
        {
            attention (action,
                       time_speed ,
                       current_time_tickt,
                       send_intervel,
                       attention_is_start,
                       joint_angles_clone,
                       start_posture,
                       end_posture);
        }

        if(action == "摆动")
        {
            if(!action_run) { continue; }
            current_time_tickt = current_time_tickt + send_intervel * time_speed ;
            shake_all_joint(current_time_tickt);
        }

        if(action == "播放下蹲文件")
        {
            if(!action_run) { continue; }
            current_time_tickt = current_time_tickt + send_intervel * time_speed ;
            action_playback(uint(current_time_tickt/ send_intervel), action_data);
        }

        if(action == "下蹲")
        {
            if(go_to_0)
            {
                current_time_tickt = current_time_tickt + send_intervel * time_speed ;
                move_to_posture(current_time_tickt,  100,  10000, squat_angle_posture_current, angle_posture[0]);
                public_info();
                if(current_time_tickt > 10000)
                {
                    current_time_tickt = 0;
                    go_to_0 = 0;
                }
                continue;
            }

            if(!action_run) { continue; }
            current_time_tickt = current_time_tickt + send_intervel * time_speed;
            for (uint posture = 0; posture < start_time_tickts.size(); ++posture)
            {
                move_to_posture(current_time_tickt,
                                start_time_tickts[posture],
                                end_time_tickts[posture],
                                angle_posture[posture],
                                angle_posture[posture + 1]);
            }
        }
        if(action == "走路")
        {
            if(!action_run) { continue; }
            cv::Point3d accel = cv::Point3d(Hi12_Imu_data_mat.at<cv::Vec3f>(1));
            walk.walking.g_rl_gyro = accel.x;                            // 左右
            walk.walking.g_fb_gyro = accel.y;                            // 前后
            std::cout<< "[" << getCurrentTimeWithMicroseconds() << "] "
                     << std::fixed
                      << "g_fb_gyro: " << std::setw(10) << walk.walking.g_fb_gyro << " "
                      << "g_rl_gyro: " << std::setw(10) << walk.walking.g_rl_gyro << std::endl ;
            action_walk(time_speed,
                        ready_to_run,
                        going_to_ready_target ,
                        current_time_tickt,
                        send_intervel,
                        joint_angles_clone,
                        start_posture,
                        end_posture,
                        Frame,
                        walk);
        }
        public_info();
    }
}

void direct_control::thread_joint_angles()
{
    std::string name = names["can_data_mat_infos_direct_control"];
    while (running)
    {
        std::unique_lock<std::mutex> lock(mutex["thread_joint_angles"]);
        condition_variable[name].wait
                (lock, [=]() { return step[name].load() == lyn_step_copying; });
        if(std::holds_alternative<cv::Mat>(input_public[name][0]["joint_angles"]))
        {
            std::unique_lock<std::mutex> lock(mutex["name_joint_angles"]);
            joint_angles = std::get<cv::Mat>(input_public[name][0]["joint_angles"]).clone();
            step[name].store(lyn_step_copy_complete);
        } else { logger << "std::get: wrong index for variant, matrix" << std::endl; }
    }
}

int direct_control::run(int argc, char *argv[])
{
    logger.run("direct_control.log", false);
    Robot.run(argc, argv);
    thread["name_walk"] = std::thread(&direct_control::thread_walk, this);
    thread["name_joint_angles"] = std::thread(&direct_control::thread_joint_angles, this);
    return 0;
}

void direct_control::wait_get(const std::string &name, cv::Mat &Hi12_Imu_data_mat)
{
    if(step[name].load() == lyn_step_copying)
    {
        std::unique_lock<std::mutex> lock(mutex[name]);
        lyn_Infos infos = input_public[name];
        step[name].store(lyn_step_copy_complete);

        if(infos.size() && std::holds_alternative<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]))
        {
            Hi12_Imu_data_mat = std::move(std::get<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]));
        } else { logger << "std::get: wrong index for variant, Hi12_Imu_data_mat" << std::endl; }
    }
}

void direct_control::public_info()
{
    public_info("direct_output_axis_angle");
    public_info("direct_output_robot");
}

void direct_control::public_info(const std::string &name)
{
    lyn_Info Info;
    Info["joint_angles"] = direct_output.clone();
    if(step[name].load() == lyn_step_updating)
    {
        output_public[name].push_back(std::move(Info));
        step[name].store(lyn_step_update_complete);
    }
}

int direct_control::work(lyn_info &info)
{
    if(info.key_press > 0) { key_press.store(info.key_press); }

    info.load(names["direct_output_axis_angle"], step, output_public);
    info.load(names["direct_output_robot"], step, output_public);
    info.unload(names["can_data_mat_infos_direct_control"], step, input_public, condition_variable);
    info.unload(names["imu_direct_control"], step, input_public, condition_variable);

    Robot.work(info);
#if 0
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "lyn_gl::work: imu_direct_control = "
              << SetpString[info.steps["imu_direct_control"]] << std::endl;
#endif
    return 0;
}
