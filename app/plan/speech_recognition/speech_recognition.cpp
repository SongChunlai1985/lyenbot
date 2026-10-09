#include <speech_recognition.h>

speech_recognition::speech_recognition()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);                                         // 设置后端（可选）
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

int speech_recognition::recognition()                                                              // 1. 克隆仓库并编译// git clone https://github.com/ggerganov/whisper.cpp.git  // cd whisper.cpp && make // 2. 下载预训练模型（如 base 模型，轻量快速）// bash ./models/download-ggml-model.sh base  // 3. C++ 调用示例（简化版）
{
    struct whisper_context* ctx = whisper_init("models/ggml-base.bin");                            // 初始化上下文
    if (!ctx)
    {
        std::cerr << "Failed to initialize whisper context" << std::endl;
        return 1;
    }
    whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);             // 配置识别参数
    params.language = "zh";                                                                        // 指定语言（en/zh/jp 等）
    params.print_progress = false;
    const char* audio_path = "test.wav";                                                           // 加载音频文件（需 16kHz、单通道、16 位 PCM 格式）
    std::vector<float> pcm;
    if (whisper_load_wav(audio_path, pcm) != 0) {
        std::cerr << "Failed to load WAV file" << std::endl;
        whisper_free(ctx);
        return 1;
    }
    if (whisper_full(ctx, params, pcm.data(), pcm.size()) != 0)                                    // 执行识别
    {
        std::cerr << "Whisper recognition failed" << std::endl;
        whisper_free(ctx);
        return 1;
    }
    std::cout << "识别结果：" << std::endl;                                                             // 输出识别结果
    for (int i = 0; i < whisper_full_n_segments(ctx); ++i)
    {
        const char* text = whisper_full_get_segment_text(ctx, i);
        std::cout << text << std::endl;
    }
    whisper_free(ctx);                                                                             // 释放资源
    return 0;
}

int speech_recognition::work(lyn_info &info)
{
    // logger << SetpString[info.step["orbbec"]] << std::endl;
    if(step == lyn_step_free)
    {
        info.step["orbbec"] = lyn_step_request;
        step = lyn_step_request;
    }

    if(info.step["orbbec"] == lyn_step_update_complete)
    {
        RawMats_info = info.infos["orbbec"];
        info.step["orbbec"] = lyn_step_copying;
        {
            std::unique_lock<std::mutex> lock(mutex);
            step = lyn_step_copying;
        }
        condition_variable.notify_one();
    }

    if(step == lyn_step_copy_complete)
    {
        info.step["orbbec"] = lyn_step_copy_complete;
        step = lyn_step_copying_2;
    }

    if(info.step["orbbec"] == lyn_step_copy_complete_2)
    {
        step = lyn_step_free;
    }
    return 0;
}
