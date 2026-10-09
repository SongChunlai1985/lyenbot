#include <kalman_filter.h>

kalman_filter::kalman_filter()
{

}

kalman_filter::kalman_filter(double processNoise, double measureNoise, cv::Point3d initialEstimate)        // 初始估计值
    : initial_estimate(initialEstimate),
      state_covariance(1.0),
      process_noise(processNoise),
      measure_noise(measureNoise)
{

}

cv::Point3d kalman_filter::update(cv::Point3d measurement)                                         // 滤波更新：输入新的测量值，返回滤波后的值
{
    cv::Point3d initial_estimate_pred = initial_estimate;                                          // 加速度变化缓慢，预测值=上一估计值
    double state_covariance_pred = state_covariance + process_noise;                               // 协方差预测：上一协方差+过程噪声
    double kalman = state_covariance_pred / (state_covariance_pred + measure_noise);               // 卡尔曼增益（权重）
    initial_estimate = initial_estimate_pred + kalman * (measurement - initial_estimate_pred);     // 状态更新
    state_covariance = (1 - kalman) * state_covariance_pred;                                       // 协方差更新
    return initial_estimate;
}
