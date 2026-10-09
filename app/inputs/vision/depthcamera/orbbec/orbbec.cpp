#include "orbbec.h"
orbbec::orbbec()
{

}

orbbec::~orbbec()
{
    running = false;
}

int orbbec::run(int argc, char *argv[])
{

#ifdef HAVE_PIPELINE
    logger.run("orbbec.log", false);
    worker = std::thread([=]()
    {
        std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>();
        std::shared_ptr<ob::Device> dev;
        std::shared_ptr<ob::FrameSet> frameSet;
        std::shared_ptr<ob::DepthFrame> depthFrame;
        ob::PointCloudFilter pointCloud;
        OBCameraParam cameraParam;
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();                       // Configure which streams to enable or disable for the Pipeline by creating a Config
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;                            // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
        try                                                                                        // Get all stream profiles of the color camera, including stream resolution, frame rate, and frame format
        {
            auto colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            if(colorProfiles) {
                auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
                colorProfile = profile->as<ob::VideoStreamProfile>();
            }
            config->enableStream(colorProfile);
        }
        catch(ob::Error &e) {
            config->setAlignMode(ALIGN_DISABLE);
            logger << "Current device is not support color sensor!" << std::endl;
        }

        std::shared_ptr<ob::StreamProfileList> depthProfileList;                                   // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
        OBAlignMode                            alignMode = ALIGN_DISABLE;
        if(colorProfile)                                                                           // Try find supported depth to color align hardware mode profile
        {
            depthProfileList = pipeline->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if(depthProfileList->count() > 0)
            {
                alignMode = ALIGN_D2C_HW_MODE;
            }
            else                                                                                   // Try find supported depth to color align software mode profile
            {
                depthProfileList = pipeline->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if(depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_SW_MODE;
                }
            }

            try                                                                                    // Enable frame synchronization
            {
                pipeline->enableFrameSync();
            }
            catch(ob::Error &e)
            {
                logger << "Current device is not support frame sync!" << std::endl;
            }
        }
        else
        {
            depthProfileList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        }

        if(depthProfileList->count() > 0)
        {
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try                                                                                    // Select the profile with the same frame rate as color.
            {
                if(colorProfile)
                {
                    depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, int(colorProfile->fps()));
                }
            }
            catch(...)
            {
                depthProfile = nullptr;
            }

            if(!depthProfile)
            {
                depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);                   // If no matching profile is found, select the default profile.
            }
            config->enableStream(depthProfile);
        }
        config->setAlignMode(alignMode);
        config->enableVideoStream(OB_STREAM_COLOR);                                                // start pipeline with config

        ob::Context ctx;
        std::shared_ptr<ob::DeviceList> devList = ctx.queryDeviceList();

        if(devList->deviceCount() == 0)
        {
            logger << "Device not found!" << std::endl;
            return -1;
        }
        dev = devList->getDevice(0);
        std::shared_ptr<ob::Sensor> gyroSensor  = nullptr;
        std::shared_ptr<ob::Sensor> accelSensor = nullptr;
        try {

            gyroSensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);                          // Get Gyroscope Sensor
            if(gyroSensor)                                                                         // Get configuration list
            {
                auto profiles = gyroSensor->getStreamProfileList();                                // Select the first profile to open stream
                auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
                gyroSensor->start(profile, [=](std::shared_ptr<ob::Frame> frame) {
                    std::unique_lock<std::mutex> lk(printerMutex);
                    gyro.                         timeStamp = frame->timeStamp();
                    gyro.                         index     = frame->index();
                    auto                          gyroFrame = frame->as<ob::GyroFrame>();
                    gyro.temperature = gyroFrame->temperature();
                    auto value = gyroFrame->value();
                    gyro.gyro = cv::Point3f(value.x, value.y, value.z);
#if false
                    if(gyroFrame != nullptr && (gyro.index % 5) == 2) {  //( timeStamp % 500 ) < 2: Reduce printing frequency

                        logger    << "Gyro Frame: \n\r{\n\r"
                                  << "  tsp = " << gyro.timeStamp << "\n\r"
                                  << "  temperature = " << gyro.temperature << "\n\r"
                                  << "  gyro.x = " << gyro.gyro.x << " rad/s"
                                  << "\n\r"
                                  << "  gyro.y = " << gyro.gyro.y << " rad/s"
                                  << "\n\r"
                                  << "  gyro.z = " << gyro.gyro.z << " rad/s"
                                  << "\n\r"
                                  << "}\n\r" << std::endl;
                    }
#endif
                });
            }
            else {
                logger << "get gyro Sensor failed ! " << std::endl;
            }
        }

        catch(ob::Error &e) {
            logger << "current device is not support imu!" << std::endl;
            return 1;
        }

        accelSensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);
        if(accelSensor)                                                                            // Get configuration list
        {
            auto profiles = accelSensor->getStreamProfileList();                                   // Select the first profile to open stream
            auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
            accelSensor->start(profile, [=](std::shared_ptr<ob::Frame> frame) {
                std::unique_lock<std::mutex> lk(printerMutex);
                accel.                         timeStamp  = frame->timeStamp();
                accel.                         index      = frame->index();
                auto                         accelFrame = frame->as<ob::AccelFrame>();
                accel.temperature = accelFrame->temperature();
                auto value = accelFrame->value();
                accel.accel = cv::Point3f(value.x, value.y, value.z);
#if 1
                if(accelFrame != nullptr && (accel.index % 5) == 0) {
                    logger    << "Accel Frame: {"
                              << "  tsp = " << accel.timeStamp << " "
                              << "  temperature = " << accel.temperature << " "
                              << "  accel.x = " << accel.accel.x << " m/s^2  "
                              << "  accel.y = " << accel.accel.y << " m/s^2  "
                              << "  accel.z = " << accel.accel.z << " m/s^2  "
                              << "} " << std::endl;
                }
#endif
            });
        }
        else {
            std::cout << "get Accel Sensor failed ! " << std::endl;
        }

        try {
            pipeline->start(config);
        }
        catch(ob::Error &e) {
            std::cerr << "pipeline start failed!" << std::endl;
            return 1;
        };
        cameraParam = pipeline->getCameraParam();
        pointCloud.setCameraParam(cameraParam);

        cv::Mat colorRawMat;
        cv::Mat pointcloud_Mat;
        while(running)
        {
            frameSet = pipeline->waitForFrames(100);
            if(frameSet == nullptr) {
                continue;
            }
            std::shared_ptr<ob::ColorFrame> colorFrame = frameSet->colorFrame();
            if(colorFrame == nullptr) {
                continue;
            }

            depthFrame = frameSet->depthFrame();
            if( depthFrame == nullptr )
            {
                continue;
            }
            auto depthValueScale = depthFrame->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);
            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
            frame = pointCloud.process(frameSet);
            cv::Mat rawMat(1, int(colorFrame->dataSize()), CV_8UC1, colorFrame->data());
            colorRawMat = cv::imdecode(rawMat, 1);
            pointcloud_Mat = cv::Mat (720, 1280, CV_32FC3, frame->data());

            lyn_Info RawMats;
            if(step.load() == lyn_step_updating)
            {
                colorRawMatD = std::move(colorRawMat);
                RawMats["colorRawMat"] = std::move(colorRawMatD);
                pointcloud_MatD = std::move(pointcloud_Mat);
                RawMats["pointcloud_Mat"] = std::move(pointcloud_MatD);
                RawMats_info.push_back(std::move(RawMats));
                step.store(lyn_step_update_complete);
            }

#if 0
            logger    << " colorFrame.data: " << colorFrame->data()
                      << " colorFrame.type: " << colorFrame->type()
                      << " colorFrame.format: " << colorFrame->format()
                      << " colorFrame.dataSize: " << colorFrame->dataSize()
                      << " colorFrame.index: " << colorFrame->index()
                      << " colorFrame.width: " << colorFrame->width()
                      << " colorFrame.height: " << colorFrame->height()
                      << std::endl;
#endif
#if false
            logger    << " depthFrame.data: " << depthFrame->data()
                      << " depthFrame.type: " << depthFrame->type()
                      << " depthFrame.format: " << depthFrame->format()
                      << " depthFrame.dataSize: " << depthFrame->dataSize()
                      << " depthFrame.index: " << depthFrame->index()
                      << " depthFrame.width: " << depthFrame->width()
                      << " depthFrame.height: " << depthFrame->height()
                      << std::endl;
#endif
        }

        pipeline->stop();

        return 0;
    });
#endif
    return 0;
}

int orbbec::stop()
{
    running = 0;
    return 0;
}

std::vector<cv::Point3d> orbbec::getPosition(std::vector<cv::Point3d> p)
{
    OBColorPoint* point = nullptr;
    std::vector<cv::Point3d> points;
    if(frame == nullptr)
    {
        std::cout << "frame == nullptr";
    }
    else
    {
        for (uint i = 0; i < p.size(); ++i)
        {
            point = (OBColorPoint*)frame->data() + int(p[i].y) * 1280 + int(p[i].x);                                     //宽0->1279，高0-719
            points.push_back(cv::Point3d(double(point->x), double(point->y), double(point->z)));
        }
    }
    return points;
}

int orbbec::work(lyn_info &info)
{
    if(info.step["orbbec"] == lyn_step_request)
    {
        step.store(lyn_step_updating);
        info.step["orbbec"] = lyn_step_updating;
    }

    if(step.load() == lyn_step_update_complete)
    {
        info.infos["orbbec"] = std::move(RawMats_info);
        info.step["orbbec"] = lyn_step_update_complete;
        step.store(lyn_step_copying);
    }
    return 0;

}
