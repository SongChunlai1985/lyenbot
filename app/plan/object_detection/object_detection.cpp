#include <object_detection.h>

object_detection::object_detection()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);                                         // 设置后端（可选）
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

cv::Mat object_detection::preprocess(const cv::Mat& image)                                         // 预处理图像
{
    cv::Mat blob;                                                                                  // 创建 blob
    cv::dnn::blobFromImage(image, blob, scaleFactor, inputSize, mean, swapRB, false);
    return blob;
}

cv::Mat object_detection::predict(const cv::Mat& image)                                            // 执行推理
{
    cv::Mat blob = preprocess(image);                                                              // 预处理
    net.setInput(blob);                                                                            // 设置输入
    cv::Mat output = net.forward();                                                                // 推理
    return output;
}

std::vector<std::string> object_detection::getOutputLayerNames()                                   // 获取输出层名称
{
    return net.getUnconnectedOutLayersNames();
}

void object_detection::object_detect(cv::Mat frame_undistort3d)
{
#if 0
    detector->detect(frame_undistort3d, detections);

    cv::Mat result = frame_undistort3d.clone();
    cv::Mat result2 = frame_undistort3d.clone();
    result.setTo(0);

    for (ulong i = 0; i < detections.size(); ++i) {
        //cv::rectangle(frame_undistort, detections[i].box, cv::Scalar(0, 255, 0, 255));                                               //框出检测物
        threadname = std::to_string(instance_id) + " save musk Image" + std::to_string(GetCurrentTimeMsec());
        threadmap[threadname] = std::thread(cv_imwrite, "jpg/" + std::to_string(GetCurrentTimeMsec()) + "uddmusk.jpg", detections[i].mask, threadname);
        cv::Mat det(frame_undistort3d, detections[i].box);
        cv::Mat mask(detections[i].mask, detections[i].box);
        DRTM_INFO("musk.type():%d", mask.type());
        cv::Mat mask8U;
        mask.convertTo(mask8U, CV_8UC1);
        DRTM_INFO("musk8U.type():%d", mask8U.type());
        det.copyTo(result(detections[i].box), mask8U);                                                                                 //剪出检测物
        putTextZH(result, std::to_string(detections[i].label), detections[i].box.tl() - cv::Point(0, 4), 10, cv::Scalar(0, 255, 0, 255));
        threadname = std::to_string(instance_id) + " save detections shap Image" + std::to_string(GetCurrentTimeMsec());
        threadmap[threadname] = std::thread(cv_imwrite, "jpg/" + std::to_string(GetCurrentTimeMsec()) + "result.jpg", result, threadname);

        cv::Mat maskbin;
        cv::threshold(mask8U, maskbin, 0, 255, cv::THRESH_TOZERO);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(maskbin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (int j = 0; j < int(contours.size()); ++j)
        {
            cv::drawContours(result2, contours, j, cv::Scalar(120, 255, 120), 2, cv::LINE_8, cv::noArray(), INT_MAX, detections[i].box.tl());
        }
    }
    threadname = std::to_string(instance_id) + " save drawContours Image" + std::to_string(GetCurrentTimeMsec());
    threadmap[threadname] = std::thread(cv_imwrite, "jpg/" + std::to_string(GetCurrentTimeMsec()) + "result2.jpg", result2, threadname);                                             //框出多边形区域
    threadname = std::to_string(instance_id) + " save detections Image" + std::to_string(GetCurrentTimeMsec());
#endif
}

int object_detection::run(int argc, char *argv[])
{
    logger.run("avoiding.log", false);
#if 0
    if(argc > 1)
    {
        cv::namedWindow("colorrawmat");
        cv::setMouseCallback("colorrawmat", [](int event, int x, int y, int flags, void* userdata)
        {
            //std::cout << "x= " << x << ", y= " << y << std::endl;
            mousex = x;
            mousey = y;
        }, nullptr);
    }
#endif
    thread = std::thread([=]()
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "avoiding running ..." << std::endl;
        while (running)
        {
            {
                std::unique_lock<std::mutex> lock(mutex);
                condition_variable.wait(lock, [=](){ return step.load() == lyn_step_copying; });          // 大括号里的一定要加，否则会停在lyn_step_copying状态
            }
            colorRawMat = std::get<cv::Mat>(RawMats_info.front()["colorRawMat"]);
        }
    });
    return 0;
}

int object_detection::work(lyn_info &info)
{
    // logger << SetpString[info.step["orbbec"]] << std::endl;
    if(step.load() == lyn_step_free)
    {
        info.step["orbbec"] = lyn_step_request;
        step.store(lyn_step_request);
    }

    if(info.step["orbbec"] == lyn_step_update_complete)
    {
        RawMats_info = info.infos["orbbec"];
        info.step["orbbec"] = lyn_step_copying;
        {
            std::unique_lock<std::mutex> lock(mutex);
            step.store(lyn_step_copying);
        }
        condition_variable.notify_one();
    }

    if(step.load() == lyn_step_copy_complete)
    {
        info.step["orbbec"] = lyn_step_copy_complete;
        step.store(lyn_step_copying_2);
    }

    if(info.step["orbbec"] == lyn_step_copy_complete_2)
    {
        step.store(lyn_step_free);
    }
    return 0;
}
