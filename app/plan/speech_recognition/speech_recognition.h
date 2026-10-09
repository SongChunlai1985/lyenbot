#ifndef LYN_SPEECH_RECOGNITION_H
#define LYN_SPEECH_RECOGNITION_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <thread>
#include <mutex>
#include <whisper.h>
#include <iostream>

class speech_recognition
{
private:

public:
    speech_recognition();
    int run(int argc, char *argv[]);
    lyn_log logger;
    int work(lyn_info &info);
    int running = 1;
    int i = 0;
    cv::Mat colorRawMat;
    std::thread thread;
    lyn_Infos RawMats_info;
    volatile lyn_step step = lyn_step_free;
    std::condition_variable condition_variable;
    std::mutex mutex;

    cv::dnn::Net net;
    cv::Size inputSize;
    float scaleFactor;
    cv::Scalar mean;
    bool swapRB;
    std::string modelPath = "fire.onnx";
    int recognition();
};
#endif // LYN_SPEECH_RECOGNITION_H
