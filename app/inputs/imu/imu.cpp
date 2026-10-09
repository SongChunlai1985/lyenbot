#include <imu.h>

imu::imu()
{
    step["Hi12_Imu_data_mat_infos"].store(lyn_step_free);
    step["Hi12_Imu_data_mat_infos_direct_control"].store(lyn_step_free);
}

std::string convertToHex(const std::vector<char>& data) {
    std::string result;
    const char hex_digits[] = "0123456789ABCDEF";

    for (char byte : data) {
        result += hex_digits[byte >> 4];
        result += hex_digits[byte & 0x0F];
        result += ' ';
    }

    return result;
}

typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        main_status;    /* reserved */
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
} hi91_t;

#define HIPNUC_MAX_RAW_SIZE     (256)   /* Maximum size of raw message buffer */

int imu::run(int argc, char *argv[])
{
    logger.run("imu.log", false);

    thread = std::thread([=]()
    {
        usbtty Usbtty;
        std::string deviceName = "/dev/ttyUSB0";
//        system("echo \"05270701\"| sudo -S chown lyenbot:lyenbot /dev/ttyUSB0");
        system("echo \"jokio.369*\"| sudo -S chown song:song /dev/ttyUSB0");
        int result = Usbtty.openDevice(deviceName.c_str(), 921600);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));                            // 避免设备异常
        std::cout << "Usbtty.writeString(\"AT+EOUT=1\\r\\n\"): " << result << std::endl;
        int writed = Usbtty.writeString("LOG HI91 ONTIME 0.005\r\n");
        std::cout << "[imu]: openDevice " << deviceName << " result: " << result << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        writed = Usbtty.writeString("AT+EOUT=1\r\n");
        std::cout << "Usbtty.writeString(LOG HI91 ONTIME 0.005): " << writed << std::endl;

        std::vector<char> buffer;
        std::vector<char> buffer_frame;

        buffer.resize(64);
        buffer_frame.resize(82);

        int is_data_5aa5 = 0;

        cv::Mat Hi12_Imu_data_mat = cv::Mat(1, 6, CV_32FC3);
        Hi12_Imu_data_mat.setTo(0);

        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "imu running ..." << std::endl;
        while (running)
        {
            buffer.resize(64);
            int buffersize = Usbtty.readChars(buffer.data());
            if (buffersize > 0)
            {
                // std::cout << std::endl << "[imu]: received " << buffersize << " bytes buffer: " << convertToHex(buffer);
                if((buffer[1] == char(0xA5) && buffer[0] == char(0x5A)))
                {
                    buffer_frame = buffer;
                    is_data_5aa5 = 1;
                }
                else
                {
                    if(buffersize == 18 && is_data_5aa5 == 1)
                    {
                        buffer_frame.resize(64 + ulong(buffersize));
                        memcpy(buffer_frame.data() + 64, buffer.data(), size_t(buffersize));
                        std::ostringstream oss;
#if 0
                        oss << "buffer_frame: " << convertToHex(buffer_frame) << " sizeof(hi91_t)" << sizeof(hi91_t) << std::endl;
                        logger << oss.str();
#endif

#if 0
                        hi91_t hi91;
                        std::memcpy(&hi91, buffer_frame.data() + 6, sizeof(hi91_t));
                        oss << "temp: " << int(hi91.temp) << " "
                            << "system_time: " << int(hi91.system_time) << " "
                            << "acc: " << hi91.acc[0] << ", " << hi91.acc[1] << ", " << hi91.acc[2] << "  "
                            << "gyr: " << hi91.gyr[0] << ", " << hi91.gyr[1] << ", " << hi91.gyr[2] << "  "
                            << "mag: " << hi91.mag[0] << "  " << hi91.mag[1] << ", " << hi91.mag[2] << "  "
                            << "rpy: " << hi91.roll << ", " << hi91.pitch << ", " << hi91.yaw << " "
                            << std::endl
                               ;
                        logger << oss.str();
#endif

                        memcpy(Hi12_Imu_data_mat.data, buffer_frame.data() + 18, 16 * sizeof(float));
                        uint32_t Hi12_Imu_timestamp = 0;

                        memcpy(&Hi12_Imu_timestamp, buffer_frame.data() + 14, sizeof(uint32_t));
#if 1
                        logger << " "
                            << "Hi12_Imu_data_mat: " << std::fixed << std::setprecision(8) << Hi12_Imu_data_mat
                            << " Hi12_Imu_timestamp: " << Hi12_Imu_timestamp << std::endl;

#endif
                        public_info("imu_reconstruction", Hi12_Imu_data_mat, Hi12_Imu_timestamp);
                        public_info("imu_direct_control", Hi12_Imu_data_mat, Hi12_Imu_timestamp);
                        public_info("imu_onnx_inference", Hi12_Imu_data_mat, Hi12_Imu_timestamp);
                        buffer.resize(ulong(buffersize));
                    }
                    else
                    {
                        //std::cout << " <--";
                    }
                    is_data_5aa5 = 0;
                }
                //std::cout << std::endl;
            } else
            {
                //std::cout << "." ;
                usleep(1000);
            }
        }
    });

    return 0;
}

void imu::public_info(const std::string &name, cv::Mat Hi12_Imu_data_mat, uint32_t Hi12_Imu_timestamp)
{
    lyn_Info info;
    info["Hi12_Imu_data_mat"] = Hi12_Imu_data_mat.clone();
    info["Hi12_Imu_timestamp"] = long(Hi12_Imu_timestamp);

    lyn_Infos Hi12_Imu_data_mat_infos;
    Hi12_Imu_data_mat_infos.push_back(info);

    if(step[name].load() == lyn_step_updating)
    {
        output_public[name] = Hi12_Imu_data_mat_infos;
        Hi12_Imu_data_mat_infos.clear();
        step[name].store(lyn_step_update_complete);
    }
}

int imu::work(lyn_info &info)
{
    info.load(name["imu_reconstruction"], step, output_public);
    info.load(name["imu_direct_control"], step, output_public);
    info.load(name["imu_onnx_inference"], step, output_public);
#if 0
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
              << SetpString[info.step["imu_direct_control"]] << std::endl;
#endif
    return 0;
}
