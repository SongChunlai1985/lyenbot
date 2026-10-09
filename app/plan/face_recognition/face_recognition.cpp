#include <face_recognition.h>

face_recognition::face_recognition()
{
}

int face_recognition::run(int argc, char *argv[])
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

            logger << SetpString[step] << std::endl;
        }
    });
    return 0;
}

void face_recognition::ArcFaceRecognizer(const std::string &model_path)
{
    try
    {
        net = cv::dnn::readNetFromONNX(model_path);                                                // 尝试使用 CUDA（如果可用）
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);                                    // 如果安装了 OpenCV with CUDA 支持，可以取消下面的注释
        // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        std::cout << "成功加载 ArcFace 模型: " << model_path << std::endl;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "加载模型失败: " << e.what() << std::endl;
        throw;
    }
}

cv::Mat face_recognition::preprocess(const cv::Mat& face_image)
{
    cv::Mat processed;
    cv::resize(face_image, processed, input_size);                                                 // 1. 调整尺寸到 112x112
    processed.convertTo(processed, CV_32FC3, scale_factor);                                        // 2. 转换为 float32 并归一化到 [0,1]
    processed -= mean_values;                                                                      // 3. 标准化: (x - mean) / std
    processed /= std_values;
    cv::cvtColor(processed, processed, cv::COLOR_BGR2RGB);                                         // 4. 转换颜色通道顺序 BGR -> RGB（某些模型需要）
    return processed;
}

std::vector<float> face_recognition::extractFeature(const cv::Mat& face_image)
{
    cv::Mat input_blob = preprocess(face_image);                                                   // 预处理图像
    cv::Mat blob = cv::dnn::blobFromImage(input_blob);                                             // 将 HWC 转换为 NCHW 格式 [1, 3, 112, 112] OpenCV 的 blobFromImage 会自动完成这个转换
    net.setInput(blob);                                                                            // 设置输入
    cv::Mat output = net.forward();                                                                // 前向传播推理
    std::vector<float> features;                                                                   // 将输出转换为特征向量
    if (output.isContinuous())
    {
        features.assign(output.ptr<float>(), output.ptr<float>() + output.total());
    } else {
        for (int i = 0; i < output.rows; i++)
        {
            const float* row = output.ptr<float>(i);
            features.insert(features.end(), row, row + output.cols);
        }
    }
    normalizeFeatures(features);                                                                   // 可选：对特征向量进行 L2 归一化
    return features;
}

float face_recognition::cosineSimilarity(const std::vector<float>& feat1, const std::vector<float>& feat2)    // 计算余弦相似度
{
    if (feat1.size() != feat2.size())
    {
        throw std::invalid_argument("特征向量维度不匹配");
    }
    float dot_product = 0.0f;
    float norm1 = 0.0f;
    float norm2 = 0.0f;
    for (size_t i = 0; i < feat1.size(); ++i)
    {
        dot_product += feat1[i] * feat2[i];
        norm1 += feat1[i] * feat1[i];
        norm2 += feat2[i] * feat2[i];
    }
    if (norm1 < 1e-8f || norm2 < 1e-8f)
    {
        return 0.0f;
    }
    return dot_product / (std::sqrt(norm1) * std::sqrt(norm2));
}

void face_recognition::normalizeFeatures(std::vector<float>& features)
{
    float norm = 0.0f;
    for (float val : features)
    {
        norm += val * val;
    }
    if (norm > 1e-8f)
    {
        norm = std::sqrt(norm);
        for (float& val : features)
        {
            val /= norm;
        }
    }
}

int face_recognition::work(lyn_info &info)
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
