#include <avoiding.h>

avoiding::avoiding()
{
}

static int mousex;
static int mousey;

int avoiding::run(int argc, char *argv[])
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

    std::string name =  names["orbbec_avoiding"];
    thread = std::thread([=]()
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "avoiding running ..." << std::endl;
        while (running)
        {
            wait_get(name, colorRawMat, pointcloud_Mat);
#if 0
            logger << SetpString[step[name]] << std::endl;
#endif
            if((!pointcloud_MatD.empty()) && (!colorRawMatD.empty()))
            {
                std::vector<cv::Mat> m;
                cv::split(pointcloud_MatD, m);

                cv::Mat mask = (m[2] == 0);                                                        // 创建一个掩码，标识所有为零的位置
                m[2].setTo(100000, mask);                                                          // 使用 setTo 函数将零值替换为大数
                cv::Mat dst;
                cv::threshold(m[2], dst, 1200.0, 0.0, cv::THRESH_TOZERO_INV);                      // 当像素值大于阈值时，设置为0，否则保持不变
                std::vector<cv::Mat> BGRm;

                cv::split(colorRawMatD, BGRm);
                cv::Mat dst3;
                dst.convertTo(dst3, CV_8UC1);
                BGRm[2] = BGRm[2] + dst3;
                cv::merge(BGRm, colorRawMatD);
                cv::putText(colorRawMatD, std::to_string(i++), cv::Point(100, 100), 1, 1, cv::Scalar(255, 0, 0));
                if(argc > 1)
                {
#if 0
                    cv::imshow("colorrawmat", colorRawMatD);
                    cv::waitKey(10);
                    logger << cv::Point3d(mousex, mousey, 0) << ", 距离: " << m[2].at<float>(mousey, mousex) << std::endl;
#endif
                }
            }
        }
    });
    return 0;
}

void avoiding::wait_get(std::string name, cv::Mat &colorRawMat, cv::Mat &pointcloud_Mat)
{
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition_variable[name].wait(lock, [=](){ return step[name].load() == lyn_step_copying; });          // 大括号里的一定要加，否则会停在lyn_step_copying状态
    }
    colorRawMat = std::get<cv::Mat>(input_public[name].front()["colorRawMat"]);
    colorRawMatD = colorRawMat.clone();

    pointcloud_Mat = std::get<cv::Mat>(input_public[name].front()["pointcloud_Mat"]);
    pointcloud_MatD = pointcloud_Mat.clone();
    step[name].store(lyn_step_copy_complete);
}

int avoiding::work(lyn_info &info)
{
    info.unload(names["orbbec_avoiding"], step, input_public, condition_variable);               // 这里需要开发重新连接
    return 0;
}
