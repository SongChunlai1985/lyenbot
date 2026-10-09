#ifndef LYN_KALMAN_FILTER_H
#define LYN_KALMAN_FILTER_H

#include <lyn_meta.h>
#include <lyn_info.h>

class kalman_filter
{
public:
    kalman_filter();
    kalman_filter(double processNoise = 0.01,                                                      // 过程噪声（加速度变化的不确定性）
                   double measureNoise = 0.1,                                                      // 测量噪声（传感器本身的噪声）
                   cv::Point3d initialEstimate = cv::Point3d(0, 0, 0));

    cv::Point3d update(cv::Point3d measurement);                                                   // 滤波更新：输入新的测量值，返回滤波后的值

private:
    cv::Point3d initial_estimate;                                                                  // 状态估计值（滤波后的值）
    double state_covariance;                                                                       // 状态协方差（估计值的不确定性）
    double process_noise;                                                                          // 过程噪声协方差
    double measure_noise;                                                                          // 测量噪声协方差
};
#endif // LYN_KALMAN_FILTER_H
