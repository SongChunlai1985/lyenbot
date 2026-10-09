#include "reconstruction.h"

static void thread_orbbec_point_cloud();
static void thread_output();
static void thread_mid360_point_cloud();
static void thread_h12_imu_data();

reconstruction::reconstruction()
{
    step["imu_reconstruction"].store(lyn_step_free);
    step["mid360_point_cloud"].store(lyn_step_free);
    step["orbbec_point_cloud"].store(lyn_step_free);
    step["realsense_point_cloud"].store(lyn_step_free);
    step["reconstruction_output"].store(lyn_step_free);
}

int reconstruction::run(int argc, char *argv[])
{
    std::ignore = argv;
    std::ignore = argc;
    logger.run("reconstruction.log", false);

    thread[h12_imu_data] = std::thread(&reconstruction::thread_h12_imu_data, this);
    thread[mid360_point_cloud] = std::thread(&reconstruction::thread_mid360_point_cloud, this);
    thread[orbbec_point_cloud] = std::thread(&reconstruction::thread_orbbec_point_cloud, this);
    thread[realsense_point_cloud] = std::thread(&reconstruction::thread_realsense_point_cloud, this);
    thread[output] = std::thread(&reconstruction::thread_output, this);
    return 0;
}

void reconstruction::thread_h12_imu_data()
{
    meta move_dot;

//    transform_testA.point_x = cv::Point3d(10, 0, 0);
//    transform_testA.point_y = cv::Point3d(0, 10, 0);
//    transform_testA.point_z = cv::Point3d(0, 0, 10);
//    transform_testA = transform_testA + cv::Point3d(2, 2, 2);

//    bornB.point_x = cv::Point3d(100, 0, 0);
//    bornB.point_y = cv::Point3d(0, 100, 0);
//    bornB.point_z = cv::Point3d(0, 0, 100);

//    bornB = bornB + cv::Point3d(0, 0, -110);
//    cv::Mat bornB_matrix = getRotate_rpy(cv::Point3d(1, 2, 3), bornB, Rotate_Sequence_XYZ);
//    bornB = bornB_matrix * bornB;

    long prev_timestamp;
    cv::Mat Hi12_Imu_data_mat;

    cv::Point3d total_accel_bias;
    cv::Point3d gyro_bias = cv::Point3d(0, 0, 0);                                                  // 陀螺仪累积偏置
    cv::Point3d total_gyro_bias(0, 0, 0);

    cv::Point3d prev_velocity(0, 0, 0);
    cv::Point3d prev_real_accel;

    cv::Point3d world_accel_bias;

    int sampling_times = 0;
    int n = 0;
    cv::Point3d prev_accel;
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "reconstruction thread_2 running ..." << std::endl;
    cv::Mat point_data_mat_xyzs = cv::Mat(1, 96, CV_32FC3);

    kalman_filter Kalman_filter_accel = kalman_filter(0.01, 0.1, cv::Point3d(0, 0, 0));
    kalman_filter Kalman_filter_gyro = kalman_filter(0.01, 0.1, cv::Point3d(0, 0, 0));
    std::list<cv::Point3d> filter;                                                                 // 平均值平滑过滤器
    for (int i = 0; i < 4; ++i)
    {
        filter.push_back(cv::Point3d(0, 0, 0));
    }
    cv::Point3d curr_accel_filtered;
    meta IMU_Hi12_copy_last;

    cv::Point3d bias_direction_last;
    cv::Point3d bias_direction_norm_last;

    imu_tools::ComplementaryFilter complementaryFilter;
    ImuFilter imu_filter;
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            condition_variable["imu_reconstruction"].wait(lock, [=]()
            { return step["imu_reconstruction"].load() == lyn_step_copying; });
        }

        lyn_Infos infos = std::move(input_public["imu_reconstruction"]);
        if(std::holds_alternative<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]))
        {
            Hi12_Imu_data_mat = std::move(std::get<cv::Mat>(infos[0]["Hi12_Imu_data_mat"]));
        } else { logger << "std::get: wrong index for variant, Hi12_Imu_data_mat" << std::endl; }
        step["imu_reconstruction"].store(lyn_step_copy_complete);
        cv::Point3d gyro = cv::Point3d(Hi12_Imu_data_mat.at<cv::Vec3f>(1) * double(LYN_PI) / 180);
        cv::Point3d curr_gyro = Kalman_filter_gyro.update(gyro);
        cv::Point3d accel = cv::Point3d(Hi12_Imu_data_mat.at<cv::Vec3f>(0));
        curr_accel = Kalman_filter_accel.update(accel);
        cv::Point3d mag = cv::Point3d(Hi12_Imu_data_mat.at<cv::Vec3f>(2));;
        rpy = cv::Point3d(Hi12_Imu_data_mat.at<cv::Vec3f>(3)) * double(LYN_PI) / 180;

        long curr_timestamp;
        if(std::holds_alternative<long>(infos[0]["Hi12_Imu_timestamp"]))
        {
            curr_timestamp = long(std::get<long>(infos[0]["Hi12_Imu_timestamp"]));
        } else { std::cerr << "std::get: wrong index for variant, Hi12_Imu_timestamp" << std::endl; }
        double delta_timestamp = double(curr_timestamp - prev_timestamp) / 1000 ;                  // 毫秒转换成秒
        prev_timestamp = curr_timestamp;
        if(delta_timestamp > 0.02) { logger << delta_timestamp << " > " << 0.02 << std::endl; continue;}                                                   // 过滤启动时第一帧的超长时间。 除了第一帧还有0.02也要消除
        if(delta_timestamp > 0.01) { logger << delta_timestamp << "-->" << 0.01 << std::endl; delta_timestamp = 0.01; }

        filter.pop_front();
        filter.push_back(curr_accel);
        cv::Point3d filter_total;
        for (auto it = filter.begin(); it != filter.end(); ++it)
        {
            filter_total += *it;
        }
        curr_accel_filtered = filter_total / int(filter.size());
        if(sampling_times < 400)                                                                   // 静止状态下的偏置
        {
            total_gyro_bias += curr_gyro;
            total_accel_bias += curr_accel;
            sampling_times ++;
            gyro_bias = total_gyro_bias / sampling_times;
            accel_bias = (total_accel_bias / sampling_times);                                      // 加速计加偏置平均值
            continue;
        };

        cv::Point3d angular_velocity = curr_gyro - gyro_bias;
        meta IMU_Hi12_copy;
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            IMU_Hi12_copy = IMU_Hi12.clone();
        }
#if 1
        cv::Point3d angular = angular_velocity * delta_timestamp;                                  // θ = ω（rad/s） × t（s）
        cv::Mat IMU_Hi12_Rotate = get_rotate_matrix(angular, IMU_Hi12_copy, Rotate_Sequence_XYZ);  // 计算绕自己的三个轴旋转变换的矩阵
        IMU_Hi12_copy = IMU_Hi12_Rotate * IMU_Hi12_copy;                                           // 更新姿态
#endif
        cv::Mat transform = IMU_Hi12_copy << meta();                                               // 计算局部到世界坐标系变换矩阵
        imu_accel = transform * curr_accel_filtered;                                               // 转到世界坐标系
        world_accel = imu_accel - IMU_Hi12_copy.position - accel_bias;                             // 移动到世界坐标系原点

        cv::Point3d real_accel = world_accel;
        cv::Point3d delta_real_accel = (prev_real_accel + real_accel) / 2;                         // Δa = (a0 + a) / 2
        cv::Point3d velocity = prev_velocity + delta_real_accel * delta_timestamp;                 // v = v0 + Δa * t
        cv::Point3d delta_s = (prev_velocity + velocity) / 2 * delta_timestamp;                    // Δs = (v0 + v) /2 * t 单位：米

        cv::Mat IMU_Hi12_Transform = get_translation(delta_s * 1000);                              // 单位：毫米
        IMU_Hi12_copy = IMU_Hi12_Transform * IMU_Hi12_copy;                                        // 更新位置
        prev_velocity = velocity;
        prev_real_accel = real_accel;

        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            IMU_Hi12 = IMU_Hi12_copy.clone();
#if 0
            MT = mt.clone();
            MT_madg = mt_madg.clone();
#endif
        }
        notify[h12_imu_data] ++ ;
        cv::Point3d bias_direction = IMU_Hi12_copy.position - IMU_Hi12_copy_last.position;
        cv::Point3d bias_direction_norm = bias_direction / cv::norm(bias_direction);

#if 1
        logger << " "
                 << "imu_accel: " << imu_accel << " "
               << "norm: " << cv::norm(bias_direction - bias_direction_last) << " "               // 飘移变化量
               << std::endl;
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            gyro_data.push_back(gyro);                                 if(gyro_data.size() > 200) { gyro_data.pop_front(); }
            gyro_filtered_data.push_back(curr_gyro);                   if(gyro_filtered_data.size() > 200) { gyro_filtered_data.pop_front(); }
            accel_data.push_back(world_accel);                           if(accel_data.size() > 200) { accel_data.pop_front(); }
            accel_filtered_data.push_back(curr_accel_filtered);        if(accel_filtered_data.size() > 200) { accel_filtered_data.pop_front(); }
        }
#endif
        bias_direction_last = bias_direction;
        bias_direction_norm_last = bias_direction_norm;
        IMU_Hi12_copy_last = IMU_Hi12_copy.clone();
    }
}

void reconstruction::thread_mid360_point_cloud()
{
    cv::Mat point_data_mat_xyz;
    point_data_mat_xyz = cv::Mat(1, 96, CV_32FC3);
    for (int i = 0; i < 500; ++i)
    {
        point_data_mat_xyzs.push_back(point_data_mat_xyz);
    }

    cv::Point3d prev_accel;
    cv::Point3d prev_real_gyro;
    double prev_timestamp;
    std::vector<long> Imu_timestamps;
    std::vector<int> Imu_data_mat_update;
    std::vector<cv::Mat> point_data_mat_xyz_vector;
    current_gravity.point_y = cv::Point3d(0, 9.81 , 0);

    std::vector<cv::Mat> Imu_data_mat;
    int sampling_times = 0;
    cv::Point3d gyro_bias(0, 0, 0);                                                                // 陀螺仪静态偏差
    cv::Point3d total_gyro_bias(0, 0, 0);
    cv::Point3d speed_vector(0, 0, 0);
    cv::Point3d accel_bias(0, 0, 0);                                                               // 加速度计静态偏差
    cv::Point3d total_accel_bias(0, 0, 0);

    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["mid360_point_cloud"]);
            condition_variable["mid360_point_cloud"].wait(lock, [=](){ return step["mid360_point_cloud"].load() == lyn_step_copying; });
        }
        lyn_Infos infos = std::move(input_public["mid360_point_cloud"]);
        Imu_data_mat_update.clear();
        Imu_data_mat.clear();
        Imu_timestamps.clear();
        for (uint i = 0; i < infos.size(); ++i)
        {
            if(std::holds_alternative<cv::Mat>(infos[i]["LivoxLidarp_point_data_mat_xyz"]))
            {
                point_data_mat_xyz_vector.push_back(std::move(std::get<cv::Mat>(infos[i]["LivoxLidarp_point_data_mat_xyz"])));
            } else { std::cerr << "std::get: wrong index for variant, LivoxLidarp_point_data_mat_xyz" << std::endl; }

            if(std::holds_alternative<cv::Mat>(infos[i]["LivoxLidarp_Imu_data_mat"]))
            {
                Imu_data_mat.push_back(std::get<cv::Mat>(std::move(infos[i]["LivoxLidarp_Imu_data_mat"])));
            } else { std::cerr << "std::get: wrong index for variant, LivoxLidarp_Imu_data_mat" << std::endl; }

            if(std::holds_alternative<int>(infos[i]["LivoxLidarp_Imu_data_mat_update"]))
            {
                Imu_data_mat_update.push_back(std::get<int>(infos[i]["LivoxLidarp_Imu_data_mat_update"]));
            } else { std::cerr << "std::get: wrong index for variant, LivoxLidarp_Imu_data_mat_update" << std::endl; }

            if(std::holds_alternative<long>(infos[i]["LivoxLidarp_Imu_timestamp"]))
            {
                Imu_timestamps.push_back(std::get<long>(infos[i]["LivoxLidarp_Imu_timestamp"]));
            } else { std::cerr << "std::get: wrong index for variant, LivoxLidarp_Imu_timestamp" << std::endl; }
#if 0
            std::ostringstream oss;
            oss << Imu_data_mat[i]<< std::endl;
            logger << oss.str();
#endif
        }
        step["mid360_point_cloud"].store(lyn_step_copy_complete);

        for (uint i = 0; i < point_data_mat_xyz_vector.size(); ++i)
        {
            point_data_mat_xyzs.push_back(point_data_mat_xyz_vector[i]);
            if(Imu_data_mat_update[i])
            {
                cv::Point3d curr_gyro(Imu_data_mat[i].at<cv::Vec3f>(0));
                cv::Point3d curr_accel(Imu_data_mat[i].at<cv::Vec3f>(1));
                double curr_timestamp = Imu_timestamps[i] / 1000000000.0;                          //纳秒转秒// cv::Point3d curr_gyro_biasx120(curr_gyro * 120);                           //放大120倍用于观察偏置
                if(sampling_times < 300)
                {
                    total_gyro_bias += curr_gyro;
                    total_accel_bias += curr_accel;
                    sampling_times ++;
                    gyro_bias = total_gyro_bias / sampling_times;
                    accel_bias = total_accel_bias / sampling_times;
                }
                cv::Point3d  curr_real_gyro =  curr_gyro - gyro_bias;
                cv::Point3d  delta_real_gyro = (curr_real_gyro + prev_real_gyro) / 2;
                prev_real_gyro = delta_real_gyro;
                double delta_timestamp = curr_timestamp - prev_timestamp;
                IMU = (get_rotate_matrix(delta_real_gyro * delta_timestamp, IMU, Rotate_Sequence_XYZ)) * IMU;      // IMU = (getRotate_rpy(-curr_gyro_biasx120, born)) * born;       // IMU = (getTransform(avg_accel * delta_timestamp) * getRotate_rpy(-avg_gyro, born)) * IMU;

                cv::Point3d avg_accel = (prev_accel + curr_accel) / 2;                             // 梯形法瞬时变换
                cv::Mat transform = IMU << meta();
                cv::Point3d imu_accel = transform * avg_accel;
                cv::Point3d world_accel = imu_accel - IMU.position;
                cv::Point3d real_accel = (world_accel - accel_bias) * 9.81;
                speed_vector = speed_vector + real_accel * delta_timestamp;                        // v = v0 + at
                // cv::Point3d moving_vector = speed_vector * delta_timestamp;                        // s = vt
                // IMU = getTransform(moving_vector) * IMU;                                         // real_accel误差 +-0.07m/s^2 无法继续。

                current_gravity = (get_translation(avg_accel * delta_timestamp) * get_rotate_matrix(delta_real_gyro, meta()))
                        * current_gravity;
                prev_accel = curr_accel;
                prev_timestamp = curr_timestamp;
#if 0
                std::ostringstream oss;
                oss << ""
                    << "IMU: " << IMU.position << "  "
                       //                            << "curr_accel: " << curr_accel << "  "
                       //                            << "total_accel_bias: " << total_accel_bias << "  "
                       //                            << "curr_gyro: " << curr_gyro << "  "
                       //                            << "avg_accel: " << avg_accel << "  "
                       //                            << "imu_accel: " << imu_accel << "  "
                       //                            << "world_accel: " << world_accel << "  "
                    << "real_accel: " << real_accel << "  "
                       //                            << "speed_vector: " << speed_vector << "  "
                       //                            << "accel_bias: " << accel_bias << "  "
                       //                            << "gyro_bias: " << gyro_bias << "  "
                       //                            << "moving_vector: " << moving_vector << "  "
                       //                            << "accel_norm: " << cv::norm(delta_accel) << "  "
                       //                            << "gyro_norm: " << cv::norm(delta_gyro) << "  "
                       //                            << "delta_timestamp: " << delta_timestamp << "  "
                    << std::endl;
                logger << oss.str();
#endif
            }
            if(point_data_mat_xyzs.rows > point_data_mat_xyzs_height)
            {
                cv::Mat point_data_mat_xyzs_roll = cv::Mat(point_data_mat_xyzs,
                                              cv::Rect(0, 1, point_data_mat_xyzs.cols, point_data_mat_xyzs_height - 1)); // 滚动裁剪
                point_data_mat_xyzs = std::move(point_data_mat_xyzs_roll);
            }
        }

        mutex_merge[mid360_point_cloud].lock();
        point_data_mat_xyzs_Copy = point_data_mat_xyzs.clone();
        if(point_data_mat_xyzs_Copy.rows > 0) {} else
        {
            logger << "error: point_data_mat_xyzs_Copy: rows" << point_data_mat_xyzs_Copy.rows
                   << " cols " << point_data_mat_xyzs_Copy.cols
                   << " channels " << point_data_mat_xyzs_Copy.channels() << std::endl;
        }

        mutex_merge[mid360_point_cloud].unlock();
        point_data_mat_xyz_vector.clear();

        notify[mid360_point_cloud] ++;

#if 0
        if(argc > 1 && (!point_data_mat_xyzs.empty()))
        {
            cv::imshow("point_data_mat_xyzs", point_data_mat_xyzs);
        }
#endif
#if 0
        std::cout << "point_data_mat_xyzs: "
                  << point_data_mat_xyzs[0].cols << ", " << point_data_mat_xyzs[0].rows
                  << " dot_num: " << dot_num << std::endl
                  << point_data_mat_xyzs[0] << std::endl
                     ;
#endif
    }
}

void reconstruction::thread_orbbec_point_cloud()
{
    cv::Mat pointcloud_MatD_Small;
    cv::Mat colorRawMatD_Small;
    cv::Mat colorRawMat;
    cv::Mat colorRawMatD;

    cv::Mat pointcloud_Mat;
    cv::Mat pointcloud_MatD;

    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["orbbec_point_cloud"]);
            condition_variable["orbbec_point_cloud"].wait(lock, [=](){ return step["orbbec_point_cloud"].load() == lyn_step_copying; });
        }
        colorRawMat = std::move(std::get<cv::Mat>(orbbec_RawMats_info.front()["colorRawMat"]));
        colorRawMatD = std::move(colorRawMat);
        pointcloud_Mat = std::get<cv::Mat>(orbbec_RawMats_info.front()["pointcloud_Mat"]);
        pointcloud_MatD = std::move(pointcloud_Mat);
        step["orbbec_point_cloud"].store(lyn_step_copy_complete);
        cv::resize(colorRawMatD, colorRawMatD_Small,
                   cv::Size(colorRawMatD.cols / ColorRawMatScale, colorRawMatD.rows / ColorRawMatScale) , 0, 0, cv::INTER_NEAREST);
        cv::resize(pointcloud_MatD, pointcloud_MatD_Small,
                   cv::Size(pointcloud_MatD.cols / ColorRawMatScale, pointcloud_MatD.rows / ColorRawMatScale) , 0, 0, cv::INTER_CUBIC);
        meta IMU_Hi12_copy;
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            IMU_Hi12_copy = IMU_Hi12.clone();
        }
        meta born_copy;
        cv::Mat transform = IMU_Hi12_copy.matrix;
//        cv_transform(pointcloud_MatD_Small, pointcloud_MatD_small,
//                     get_rotate_matrix(cv::Point3d(0, 0, double(LYN_PI)), IMU_Hi12_copy)
//                     * transform);
        cv_transform(pointcloud_MatD_Small, pointcloud_MatD_small,
                     get_rotate_matrix(cv::Point3d(0, 0, double(LYN_PI)), IMU_Hi12_copy) * transform);
        colorRawMatD_small = std::move(colorRawMatD_Small);
        notify[orbbec_point_cloud] ++;
    }
}

void reconstruction::thread_realsense_point_cloud()
{
    cv::Mat pointcloud_MatD_Small;
    cv::Mat colorRawMatD_Small;
    cv::Mat colorRawMat;
    cv::Mat colorRawMatD;

    cv::Mat pointcloud_Mat;
    cv::Mat pointcloud_MatD;
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["realsense_point_cloud"]);
            condition_variable["realsense_point_cloud"].wait(lock, [=](){ return step["realsense_point_cloud"].load() == lyn_step_copying; });
        }
        colorRawMat = std::move(std::get<cv::Mat>(orbbec_RawMats_info.front()["colorRawMat"]));
        colorRawMatD = std::move(colorRawMat);
        pointcloud_Mat = std::get<cv::Mat>(orbbec_RawMats_info.front()["pointcloud_Mat"]);
        pointcloud_MatD = std::move(pointcloud_Mat);
        step["realsense_point_cloud"].store(lyn_step_copy_complete);
        cv::resize(colorRawMatD, colorRawMatD_Small,
                   cv::Size(colorRawMatD.cols / ColorRawMatScale, colorRawMatD.rows / ColorRawMatScale) , 0, 0, cv::INTER_NEAREST);
        cv::resize(pointcloud_MatD, pointcloud_MatD_Small,
                   cv::Size(pointcloud_MatD.cols / ColorRawMatScale, pointcloud_MatD.rows / ColorRawMatScale) , 0, 0, cv::INTER_CUBIC);
        meta IMU_Hi12_copy;
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            IMU_Hi12_copy = IMU_Hi12.clone();
        }
        meta born_copy;
        cv::Mat transform = IMU_Hi12_copy.matrix;
//        cv_transform(pointcloud_MatD_Small, pointcloud_MatD_small,
//                     get_rotate_matrix(cv::Point3d(0, 0, double(LYN_PI)), IMU_Hi12_copy)
//                     * transform);
        cv_transform(pointcloud_MatD_Small, pointcloud_MatD_small,
                     get_rotate_matrix(cv::Point3d(0, 0, double(LYN_PI)), IMU_Hi12_copy) * transform);
        colorRawMatD_small = std::move(colorRawMatD_Small);
        notify[orbbec_point_cloud] ++;
    }
}

void reconstruction::output_h12_imu_data()
{
    cv::Mat IMU_Points_h12;
    draw_meta(meta(), IMU_Points_h12, 120);
    //                    draw_meta(bornB, IMU_Points_h12);
    //                    draw_meta(transform_testA, IMU_Points_h12);
    //                    draw_meta(transform_testB, IMU_Points_h12);
    meta IMU_Hi12_copy;
    meta mt;
    meta mt_madg;
    {
        std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
        IMU_Hi12_copy = IMU_Hi12.clone();
        mt = MT.clone();
        mt_madg = MT_madg.clone();
    }
    draw_meta(IMU_Hi12_copy, IMU_Points_h12, 80);
    draw_meta(meta() * 0.1 + IMU_Hi12_copy.position, IMU_Points_h12, 80);                          // 0
    draw_meta(IMU_Hi12_copy - IMU_Hi12_copy.position + cv::Point3d(1110, 0, 0), IMU_Points_h12, 80);// 1
    draw_meta(meta() + cv::Point3d(1110, 0, 0), IMU_Points_h12, 80);
    draw_meta(meta() + world_accel + cv::Point3d(1330, 0, 0), IMU_Points_h12, 80);                 // 2
    draw_meta(meta() + cv::Point3d(1330, 0, 0), IMU_Points_h12, 80);
    draw_meta(meta() + imu_accel - IMU_Hi12_copy.position + cv::Point3d(1220, 0, 0), IMU_Points_h12, 80);// 3
    draw_meta(meta() + cv::Point3d(1220, 0, 0), IMU_Points_h12, 80);
    draw_meta(meta() + accel_bias + cv::Point3d(1440, 0, 0), IMU_Points_h12, 80);                  // 4
    draw_meta(meta() + cv::Point3d(1440, 0, 0), IMU_Points_h12, 80);
    draw_meta(meta() + curr_accel + cv::Point3d(1550, 0, 0), IMU_Points_h12, 80);                  // 5
    draw_meta(meta() + cv::Point3d(1550, 0, 0), IMU_Points_h12, 80);
    draw_meta(mt + cv::Point3d(1660, 0, 0), IMU_Points_h12, 100);                                  // 6
    draw_meta(mt_madg + cv::Point3d(1770, 0, 0), IMU_Points_h12, 100);                             // 7
    draw_meta(get_rotate_matrix(rpy, meta()) * meta() + cv::Point3d(1880, 0, 0), IMU_Points_h12, 100);  // 8
    cv::Point3f color(0.16f, 0.24f, 0.32f);
    for (int i = -20; i <= 20; ++i)                                                                // draw gird
    {
        draw_line(cv::Point3f(i * 500.0f, -830.0f, -10000.0f),
                  cv::Point3f(i * 500.0f, -830.0f, 10000.0f),
                  color, color, IMU_Points_h12);
        draw_line(cv::Point3f(-10000.0f, -830.0f, i * 500.0f),
                  cv::Point3f(10000.0f, -830.0f, i * 500.0f),
                  color, color, IMU_Points_h12);
    }
    {
        std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
        draw_line_chart(gyro_data, IMU_Points_h12, meta() + cv::Point3d(500, 200, 0));
        draw_line( cv::Point3d(500, 200, 0),  cv::Point3d(2500, 200, 0),
                   cv::Point3f(1.0f, 0, 0), cv::Point3f(1.0f, 0, 0), IMU_Points_h12);
        draw_line_chart(gyro_filtered_data, IMU_Points_h12, meta() + cv::Point3d(500, 400, 0));
        draw_line( cv::Point3d(500, 400, 0),  cv::Point3d(2500, 400, 0),
                   cv::Point3f(1.0f, 0, 0), cv::Point3f(1.0f, 0, 0), IMU_Points_h12);
        draw_line_chart(accel_data, IMU_Points_h12, meta() + cv::Point3d(500, 600, 0));
        draw_line( cv::Point3d(500, 600, 0),  cv::Point3d(2500, 600, 0),
                   cv::Point3f(1.0f, 0, 0), cv::Point3f(1.0f, 0, 0), IMU_Points_h12);
        draw_line_chart(accel_filtered_data, IMU_Points_h12, meta() + cv::Point3d(500, 800, 0));
        draw_line( cv::Point3d(500, 800, 0),  cv::Point3d(2500, 800, 0),
                   cv::Point3f(1.0f, 0, 0), cv::Point3f(1.0f, 0, 0), IMU_Points_h12);
    }
    // draw_meta(born + world_gravity, IMU_Points_h12);
    lyn_Info output_info;
    output_info["IMU_Points_h12"] = std::move(IMU_Points_h12);
#if 0
    std::ostringstream oss;
    oss << ""
        << "IMU_Points_h12: " << std::endl << IMU_Points_h12 << std::endl << " "
           //                        << "IMU_Points_h12.rows: " << IMU_Points_h12.rows << " "
           //                        << ".cols: " << IMU_Points_h12.cols << " "
           //                        << ".channels(): " << IMU_Points_h12.channels() << " "
           //                        << ".elemSize1(): " << IMU_Points_h12.elemSize1() << " "
           ;
    oss << std::endl;
    logger << oss.str();
#endif
    output_info["name"] = thread_name_string[h12_imu_data];
    reconstruction_output_infos.push_back(output_info);
    notify[h12_imu_data] = 0;
}

int reconstruction::cv_transform(cv::Mat &src, cv::Mat &dst, cv::Mat m)
{
    std::vector<cv::Mat> src_split;
    cv::split(src, src_split);
    cv::Mat src04 = cv::Mat(src.rows, src.cols, CV_32FC1, cv::Scalar(1.0));
    src_split.push_back(std::move(src04));
    cv::Mat src4;
//    std::cout << src_split[0].cols << ", " << src_split[0].rows << ", " << src_split[0].channels() << "; ";
//    std::cout << src04.cols << ", " << src04.rows << ", " << src04.channels() << "; ";
//    dst = src.clone(); return 0;
    cv::merge(src_split, src4);
    cv::Mat dst4;
    std::vector<cv::Mat> dst_split;
    cv::transform(src4, dst4, m);
    cv::split(dst4, dst_split);
    dst_split.pop_back();
    cv::merge(dst_split, dst);
    return 0;
}

void reconstruction::output_orbbec_point_cloud()
{
    lyn_Info output_info;
    mutex_merge[orbbec_point_cloud].lock();
    pointcloud_MatD_small_copy = std::move(pointcloud_MatD_small);
    colorRawMatD_copy = std::move(colorRawMatD_small);
    mutex_merge[orbbec_point_cloud].unlock();
    output_info["pointcloud_MatD_small_copy"] = std::move(pointcloud_MatD_small_copy);
    output_info["colorRawMatD_copy"] = std::move(colorRawMatD_copy);                             // if(argc > 1)cv::imshow("colorRawMatD_copy", colorRawMatD_copy);
    output_info["name"] = thread_name_string[orbbec_point_cloud];
    reconstruction_output_infos.push_back(output_info);
    notify[orbbec_point_cloud] = 0;
}

void reconstruction::output_mid360_point_cloud()
{
    cv::Mat IMU_Points;
    //draw_meta(IMU, IMU_Points);
    draw_meta(meta(), IMU_Points, 110);
    // draw_meta(current_gravity, IMU_Points);
    lyn_Info output_info;
    mutex_merge[mid360_point_cloud].lock();
    cv::Mat point_data_mat_xyzs_copy = std::move(point_data_mat_xyzs_Copy);
    mutex_merge[mid360_point_cloud].unlock();
    if(point_data_mat_xyzs_copy.rows > 0)
    {
        cv::Mat point_data_mat_xyzs_copy_transformed;
        //    logger << " rows " <<point_data_mat_xyzs_copy.rows
        //           << " cols " << point_data_mat_xyzs_copy.cols
        //           << " channels " << point_data_mat_xyzs_copy.channels() << std::endl;
        meta IMU_Hi12_copy;
        {
            std::unique_lock<std::mutex> lock(mutex["h12_imu_data"]);
            IMU_Hi12_copy = IMU_Hi12.clone();
        }
        meta born_copy;
        cv::Mat transform = born_copy >> IMU_Hi12_copy;
        cv_transform(point_data_mat_xyzs_copy, point_data_mat_xyzs_copy_transformed, transform
                     * get_rotate_matrix(cv::Point3d(0, 0, double(LYN_PI)), IMU_Hi12_copy));       // 绕z轴旋转180度
        output_info["point_data_mat_xyzs_copy"] = std::move(point_data_mat_xyzs_copy_transformed);
        output_info["IMU_Points"] = std::move(IMU_Points);
        output_info["name"] = thread_name_string[mid360_point_cloud];
#if 0
        if(!Imu_data_mat.empty())
        {
            std::ostringstream oss;
            oss << ""
                << "Imu_data_mat: " << Imu_data_mat << std::endl;
            logger << oss.str();
        }
#endif
#if 0
        std::ostringstream oss;
        oss << ""
            << "IMU.point_position: " << IMU.point_position << " "
            << "x: " << IMU.point_x << " "
            << "y: " << IMU.point_y << " "
            << "z: " << IMU.point_z << " "  << std::endl;
        logger << oss.str();
#endif
        reconstruction_output_infos.push_back(output_info);
    } else
    {
        logger << "error: point_data_mat_xyzs_copy.empty() "
               << point_data_mat_xyzs_copy.empty() << ", "
               << notify[mid360_point_cloud] << std::endl;
    }
    notify[mid360_point_cloud] = 0;
}

void reconstruction::thread_output()
{
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["reconstruction_output"]);
            condition_variable["reconstruction_output"].wait(lock, [=]()
            { return step["reconstruction_output"].load() == lyn_step_updating; });
        }

        if(notify[h12_imu_data]) { output_h12_imu_data(); }
        if(notify[mid360_point_cloud]) { output_mid360_point_cloud(); }
        if(notify[orbbec_point_cloud]) { output_orbbec_point_cloud(); }

        if(reconstruction_output_infos.size())
        {
#if 0
            std::ostringstream oss;
            oss << ""
                << "reconstruction_output_infos: " << reconstruction_output_infos.size() << " ";
            for (uint i = 0; i < reconstruction_output_infos.size(); ++i)
            {
                oss << std::get<std::string>(reconstruction_output_infos[i]["name"]) << ", ";
            }
            oss << std::endl;
            logger << oss.str();
#endif
            output_public["reconstruction_output"] = reconstruction_output_infos;
            step["reconstruction_output"].store(lyn_step_update_complete);
            reconstruction_output_infos.clear();
        }
    }
}

int reconstruction::work(lyn_info &info)
{
    info.unload(names["imu_reconstruction"], step, input_public, condition_variable);
    info.unload(names["LivoxLidarp_point_data_mat_infos"], step, input_public, condition_variable);
    info.unload(names["orbbec"], step, input_public, condition_variable);
    info.load(names["reconstruction_output"], step, output_public);
    if(step["reconstruction_output"].load() == lyn_step_updating)
    {
        condition_variable["reconstruction_output"].notify_one();                                  // 等待对方消费完
    }
#if 0
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "reconstruction::work: "
              << SetpString[info.steps["reconstruction_output"]] << std::endl;
#endif
    return 0;
}
