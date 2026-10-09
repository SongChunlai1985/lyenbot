#include <onnx_inference.h>

onnx_inference::onnx_inference()
{
}

void precise_sleep_5ms()
{
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
}

void onnx_inference::simulate_flat_ground(active_observation_268& obs, float base_height)
{
    for(int i = 0; i < 187; i++)
    {
        obs.height_scan[i] = base_height;
    }
}

void onnx_inference::simulate_joint_pos(active_observation_268& obs, cv::Mat onnx_output)
{
    for(int i = 0; i < 23; i++)
    {
        obs.joint_pos[i] = onnx_output.at<float>(0, i);
    }
}

void onnx_inference::simulate_joint_vel(active_observation_268& obs,
                                        cv::Mat onnx_output,
                                        cv::Mat onnx_output_last)
{
    for(int i = 0; i < 23; i++)
    {
        obs.joint_vel[i] = (onnx_output.at<float>(0, i)
                - onnx_output_last.at<float>(0, i)) * 0.05f * 0.0001f;
    }
}

void onnx_inference::simulate_joint_actions(active_observation_268& obs, cv::Mat onnx_output)
{
    std::normal_distribution<float> medium_noise(-5.31f, 5.31f);
    for(int i = 0; i < 23; i++)
    {
        obs.actions[i] = onnx_output.at<float>(0, i) + medium_noise(gen);
    }
}

void onnx_inference::set_input(cv::Mat Hi12_Imu_data_mat,
                               cv::Mat onnx_output,
                               cv::Mat onnx_output_last,
                               active_observation_268 &Active_observation_268)
{

    Active_observation_268.base_lin_vel[0] = Hi12_Imu_data_mat.at<float>(0, 0);
    Active_observation_268.base_lin_vel[1] = Hi12_Imu_data_mat.at<float>(0, 1);
    Active_observation_268.base_lin_vel[2] = Hi12_Imu_data_mat.at<float>(0, 2);

    Active_observation_268.base_ang_vel[0] = Hi12_Imu_data_mat.at<float>(0, 3);
    Active_observation_268.base_ang_vel[1] = Hi12_Imu_data_mat.at<float>(0, 4);
    Active_observation_268.base_ang_vel[2] = Hi12_Imu_data_mat.at<float>(0, 5);

    Active_observation_268.projected_gravity[0] = 0;
    Active_observation_268.projected_gravity[1] = 1;
    Active_observation_268.projected_gravity[2] = 0;

    Active_observation_268.velocity_commands[0] = 3.1f;
    Active_observation_268.velocity_commands[1] = 0;
    Active_observation_268.velocity_commands[2] = 0;

    simulate_joint_pos(Active_observation_268, onnx_output);
    simulate_joint_vel(Active_observation_268, onnx_output, onnx_output_last);
    simulate_joint_actions(Active_observation_268, onnx_output);
    simulate_flat_ground(Active_observation_268, 2.5f);
}

void onnx_inference::thread_walk()
{
    const cv::Size inputSize(268, 1);
    const double scaleFactor = 1.0 / 255.0;
    const cv::Scalar meanValues(0.485, 0.456, 0.406);
    const bool swapRB = true;
    cv::Mat blob;
    clock_t starttime_1, starttime_2, starttime_3, starttime_4;
    cv::Mat Hi12_Imu_data_mat(1, 128, CV_32FC1, cv::Scalar(0.0));
    cv::Mat joint_angles(1, 128, CV_32FC1, cv::Scalar(0.0));
    cv::Mat joint_angles_last(1, 128, CV_32FC1, cv::Scalar(0.0));
    input.setTo(0);
    while (true)
    {
        starttime_1 = clock();

        if(step[h12_imu_data].load() == lyn_step_copying_2)
        {
            mutex.lock();
            lyn_Infos infos = Hi12_Imu_data_mat_infos;
            mutex.unlock();
            step[h12_imu_data].store(lyn_step_copy_complete_2);

            if(infos.size() && std::holds_alternative<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]))
            {
                Hi12_Imu_data_mat = std::get<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]);
            } else
            {
                logger << "std::get: wrong index for variant, Hi12_Imu_data_mat" << std::endl;
            }
        }
#if 0
        logger << " Hi12_Imu_data_mat.cols: " << Hi12_Imu_data_mat.cols
               << " onnx_output.cols: " << onnx_output.cols
               << " onnx_output_last.cols: " << onnx_output_last.cols
               << std::endl;
#endif
        set_input(Hi12_Imu_data_mat, joint_angles, joint_angles_last, Active_observation_268);
        input = cv::Mat(1, 128, CV_32FC1, static_cast<void*>(&Active_observation_268)).clone();
        cv::Mat curr_input = input.clone();
        logger << "input: " << input << std::endl;
        cv::dnn::blobFromImage(curr_input, blob, scaleFactor, inputSize, meanValues, swapRB, false);
        starttime_2 = clock();
        net[cur_net].setInput(blob);
        starttime_3 = clock();
        joint_angles_last = joint_angles.clone();
        joint_angles = net[cur_net].forward();
        starttime_4 = clock();
        double pre_process_time = double(starttime_2 - starttime_1) / CLOCKS_PER_SEC * 1000;
        double process_time = double(starttime_3 - starttime_2) / CLOCKS_PER_SEC * 1000;
        double post_process_time = double(starttime_4 - starttime_3) / CLOCKS_PER_SEC * 1000;
#if 0
        logger << onnx_output
               << "\n [ONNX(CUDA)]: " << pre_process_time << "ms pre-process, "
               << process_time << "ms inference, "
               << post_process_time << "ms post-process." << onnx_output.size() << std::endl;
#endif
        logger << "onnx_output: " << joint_angles << std::endl;
        lyn_Info Info;

        Info["joint_angles"] = joint_angles.clone();
        if(step[joints_value].load() == lyn_step_updating)
        {
            onnx_output_public.push_back(std::move(Info));
            step[joints_value].store(lyn_step_update_complete);
        }
        precise_sleep_5ms();
    }
}

int onnx_inference::run(int argc, char *argv[])
{
    logger.run("onnx.log", false);
    input = cv::Mat(1, 128, CV_32FC1, static_cast<void*>(&Active_observation_268)).clone();
    for (int i = lyn_net_stop; i < lyn_net_run + 1; i++)
    {
        net[i] = cv::dnn::readNetFromONNX(modelPath[i]);
        net[i].setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net[i].setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        if (net[i].empty()) { logger << "Failed to load the ONNX model: " << modelPath << std::endl;}
    }
    cur_net = lyn_net_stop;
    thread = std::thread(&onnx_inference::thread_walk, this);
    std::ignore = argv;
    std::ignore = argc;
    return 0;
}

int onnx_inference::work(lyn_info &info)
{
// onnx_output
    // logger << SetpString[info.step["output_public"]] << std::endl;

    if(info.step["onnx_output_public"] == lyn_step_free ||
            info.step["onnx_output_public"] == lyn_step_copy_complete)
    {
        info.step["onnx_output_public"] = lyn_step_request;
        step[joints_value].store(lyn_step_request);
    }

    if(info.step["onnx_output_public"] == lyn_step_request)
    {
        step[joints_value].store(lyn_step_updating);
        info.step["onnx_output_public"] = lyn_step_updating;
    }

    if(step[joints_value].load() == lyn_step_update_complete)
    {
        info.infos["onnx_output_public"] = std::move(onnx_output_public);
        info.step["onnx_output_public"] = lyn_step_update_complete;
        step[joints_value].store(lyn_step_copying);
    }

//imu_onnx_inference
    if(info.step["imu_onnx_inference"] == lyn_step_copying)
    {
        step[h12_imu_data].store(lyn_step_copying);
        mutex.lock();
        Hi12_Imu_data_mat_infos = info.infos["imu_onnx_inference"];
        mutex.unlock();
    }
    // logger << SetpString[info.step["imu_onnx_inference"]] << std::endl;
    return 0;
}
