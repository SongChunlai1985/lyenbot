#include "real_sense.h"
real_sense::real_sense()
{

}

real_sense::~real_sense()
{
    running = false;
}

bool check_accel_gyro_are_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

bool check_combined_motion_is_supported()
{
    rs2::context ctx;

    for (auto dev : ctx.query_devices())
    {
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_MOTION)
                    return true;
            }
        }
    }

    return false;
}

int real_sense::run(int argc, char *argv[])
{
    logger.run("real_sense.log", false);
    return 0;
    worker = std::thread([=]()
    {
        rs2::pipeline pipe;
        rs2::config cfg;
        if (check_combined_motion_is_supported())
        {
            cfg.enable_stream(RS2_STREAM_MOTION, RS2_FORMAT_COMBINED_MOTION);
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            logger << "check_combined_motion_is_supported" << std::endl;
        }
        else if (check_accel_gyro_are_supported())
        {
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
            logger << "check_accel_gyro_are_supported" << std::endl;
        }
        else
        {
            logger << "Device supporting IMU not found";
            return EXIT_FAILURE;
        }
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);                                                                           // 运行要加 sudo
        rs2::pointcloud pc;                                                                        // 创建点云对象
        rs2::colorizer color_map;                                                                  // 创建颜色映射对象
        cv::Mat depth_image;
        cv::Mat color_image;
        while (true)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();                                     // 获取深度帧
            rs2::video_frame color = frames.get_color_frame();                                     // 获取颜色帧（如果有）
            if (color)                                                                             // 生成点云
            {
                pc.map_to(color);                                                                  // 告诉点云对象使用哪个颜色帧
            }
            rs2::points points = pc.calculate(depth);
            auto vertices = points.get_vertices();                                                 // 顶点坐标数组
            auto tex_coords = points.get_texture_coordinates();                                    // 纹理坐标数组
            depth_image = cv::Mat(cv::Size(640, 480), CV_8UC3,
                                (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            color_image = cv::Mat (cv::Size(640, 480), CV_8UC3,
                                (void*)depth.get_data(), cv::Mat::AUTO_STEP);

            logger << "depth_image: "
                   << depth_image.cols << ", "
                   << depth_image.rows << ", "
                   << depth_image.channels() << ", "
                   << depth_image.elemSize1() << ", "
                   << "color_image: "
                   << color_image.cols << ", "
                   << color_image.rows << ", "
                   << color_image.channels() << ", "
                   << color_image.elemSize1() << ", "
                   << "points: "
                   << points.size()
                   << std::endl;
        }
    });
    return 0;
}

int real_sense::stop()
{
    running = 0;
    return 0;
}

int real_sense::work(lyn_info &info)
{
    if(info.step["real_sense"] == lyn_step_request)
    {
        step.store(lyn_step_updating);
        info.step["real_sense"] = lyn_step_updating;
    }

    if(step.load() == lyn_step_update_complete)
    {
        info.infos["real_sense"] = std::move(RawMats_info);
        info.step["real_sense"] = lyn_step_update_complete;
        step.store(lyn_step_copying);
    }
    return 0;

}
