#ifndef LYN_OBJECT_DETECTION_H
#define LYN_OBJECT_DETECTION_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class object_detection
{
private:

public:
    object_detection();
    int run(int argc, char *argv[]);
    lyn_log logger;
    int work(lyn_info &info);
    int running = 1;
    int i = 0;
    cv::Mat colorRawMat;
    std::thread thread;
    lyn_Infos RawMats_info;
    std::atomic<lyn_step> step = lyn_step_free;
    std::condition_variable condition_variable;
    std::mutex mutex;

    cv::dnn::Net net;
    cv::Size inputSize;
    float scaleFactor;
    cv::Scalar mean;
    bool swapRB;
    std::string modelPath = "fire.onnx";
    void object_detect(cv::Mat frame_undistort3d);
    cv::Mat preprocess(const cv::Mat& image);
    cv::Mat predict(const cv::Mat& image);
    std::vector<std::string> getOutputLayerNames();
};
#endif // LYN_OBJECT_DETECTION_H
