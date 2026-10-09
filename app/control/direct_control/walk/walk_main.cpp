#include <walk_main.h>
#include "hip_tilt_axis_IK.cpp"

Walk::Walk()
{
}

void Walk::WalkTimeInitialize(void)
{
   walking.WalkTime = 0;
}

void Walk::SetWalkingStepSize(float step,float stepHeight)                                         /* step: 步长 tepHeight：抬腿高度 */
{
    walking.init_x_Move_Amplitude = double(step);
    walking.init_z_Move_Amplitude  = double(stepHeight);
}

void Walk::set_time_msec(double time)
{
    walking.time_Msec = time;
}

void Walk::set_m_Time(double time)
{
    walking.m_Time = time;
}

double Walk::get_m_Time()
{
    return double(walking.m_Time);
}

int Walk::get_state()
{
    return walking.state;
}


void Walk::walking_process(uint8_t walk_line)
{
    walking.WalkLineSype = walk_line;
    walking.Process();
}

void Walk::walking_init()
{
    walking.Initialize();
    walking.Start();
}

void Walk::stop()
{
    walking.Stop();
}

void Walk::start()
{
    walking.Start();
}

cv::Mat Walk::GetFrameAtTimeMs()
{
    walking.Process();
    for (int i = 0; i < 16; ++i)
    {
        std::string name = ID_to_name_map[i];
        uint index = can_name_to_mat_index_map[name];
        Frame_RAD.at<float>(0, int(index)) =
                float(walking.WalkAngle[i]) * walk_motor_direction[name] * float(DEG);
    }
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LShoulderRoll"])) = float(15 * DEG);
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RShoulderRoll"])) = float(-15 * DEG);

#if 1
    PRY r_pry =
            fitTiltedAngles_PRY
            (double(Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipPitch"]))),
            double(Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipRoll"]))),
            double(-Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipYaw"]))));

    PRY l_pry =
            fitTiltedAngles_PRY
            (double(Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipPitch"]))),
            double(-Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipRoll"]))),
            double(-Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipYaw"]))));
#if 0
    std::cout << std::endl;
    std::cout << "r_pry_i: " << double(Frame.at<float>(0, int(can_name_to_mat_index_map["RHipPitch"]))) * RAD
            << ", " << double(Frame.at<float>(0, int(can_name_to_mat_index_map["RHipRoll"]))) * RAD
            << ", " << double( Frame.at<float>(0, int(can_name_to_mat_index_map["RHipYaw"]))) * RAD << std::endl;
    std::cout << "r_pry_o: " << r_pry.pitch * RAD << ", " << r_pry.roll * RAD << ", " << r_pry.yaw * RAD<< std::endl;

    std::cout << std::endl;
    std::cout << "l_pry_i: "
              << double(Frame.at<float>(0, int(can_name_to_mat_index_map["LHipPitch"]))) * RAD
            << ", " << double(Frame.at<float>(0, int(can_name_to_mat_index_map["LHipRoll"]))) * RAD
            << ", " << double( Frame.at<float>(0, int(can_name_to_mat_index_map["LHipYaw"]))) * RAD << std::endl;
    std::cout << "l_pry_o: " << l_pry.pitch * RAD << ", " << l_pry.roll * RAD << ", " << l_pry.yaw * RAD << std::endl;
#endif

    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipPitch"])) = float(r_pry.pitch);
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipRoll"])) = float(r_pry.roll);
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["RHipYaw"])) = float(-r_pry.yaw);

    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipPitch"])) = float(l_pry.pitch);
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipRoll"])) = float(-l_pry.roll);
    Frame_RAD.at<float>(0, int(can_name_to_mat_index_map["LHipYaw"])) = float(-l_pry.yaw);
#endif

#if 1
    print_mat_formatted(Frame_RAD * RAD);  std::cout << " m_Time = " << std::setw(10) << std::fixed << float(get_m_Time());
#endif
    return Frame_RAD;
}
