#ifndef LYN_FACE_RECOGNITION_H
#define LYN_FACE_RECOGNITION_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <numeric>

class face_recognition
{
private:
    cv::dnn::Net net;
    const cv::Size input_size = cv::Size(112, 112);                                                // ArcFace R100 的标准输入尺寸
    const float scale_factor = 1.0 / 255.0;                                                        // 归一化系数
    const cv::Scalar mean_values = cv::Scalar(0.5, 0.5, 0.5);                                      // 均值
    const cv::Scalar std_values = cv::Scalar(0.5, 0.5, 0.5);                                       // 标准差
public:
    face_recognition();
    int run(int argc, char *argv[]);
    lyn_log logger;
    int work(lyn_info &info);
    int running = 1;
    int i = 0;
    void ArcFaceRecognizer(const std::string& model_path);
    cv::Mat preprocess(const cv::Mat& face_image);
    std::vector<float> extractFeature(const cv::Mat& face_image);
    float cosineSimilarity(const std::vector<float>& feat1, const std::vector<float>& feat2);
    void normalizeFeatures(std::vector<float>& features);

    lyn_Infos RawMats_info;
    std::atomic<lyn_step> step = lyn_step_free;

    std::thread thread;
    std::condition_variable condition_variable;
    std::mutex mutex;
};
#endif // LYN_FACE_RECOGNITION_H
