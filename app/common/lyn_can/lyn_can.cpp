#include "lyn_can.h"

static void thread_can_read();

can::can()
{
}

int can::run(int argc, char *argv[])
{
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)                                               // 创建 socket
    {
        perror("socket");
        return -1;
    }
    strcpy(ifr.ifr_name, name.c_str());                                                            // 设置接口
    int err = ioctl(s, SIOCGIFINDEX, &ifr);
    std::cout << "name: " << name  << " s: " << s << " err: " << err << std::endl;
    if(err)
    {
        perror("socket");
        return -1;                                                                                 // 不返回会收到其它接口的数据
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)                                       // 绑定
    {
        perror("bind");
        return -1;
    }
    thread = std::thread(&can::thread_can_read, this);
    return 0;
}

void can::set_callback(Callback cb)
{
    callback_ = cb;
}

int can::send_whith_id(unsigned int can_id, unsigned char can_dlc, unsigned char* data)
{
    frame.can_id = can_id;
    frame.can_dlc = can_dlc;
    std::memcpy(frame.data, data, sizeof(data));
    return write(s, &frame, sizeof(struct can_frame));
}

void can::thread_can_read()
{
    while (running)
    {
        nbytes = read(s, &frame, sizeof(struct can_frame));
        if (callback_)
        {
            callback_(name, frame.can_id, frame.can_dlc, frame.data);
        };
    }
}

unsigned char* can::make_data(std::string cmd,
                              std::string can_address_name,
                              std::string data_type,
                              double value, double value2)
{
    unsigned char* frame_data = new unsigned char[8];
    memset(frame_data, 0, 8);

    if (can_cmd.find(cmd) == can_cmd.end() ||
            can_address.find(can_address_name) == can_address.end())
    {
        delete[] frame_data;
        throw std::invalid_argument("Invalid command or address name");
    }

    frame_data[0] = can_cmd[cmd];
    frame_data[1] = can_address[can_address_name];

    if (data_type == "float")
    {
        float float_value = static_cast<float>(value);
        unsigned char* float_bytes = reinterpret_cast<unsigned char*>(&float_value);
        memcpy(&frame_data[2], float_bytes, 4);
    }
    else if (data_type == "uint8")
    {
        uint8_t uint8_value = static_cast<uint8_t>(value);
        frame_data[2] = uint8_value;
    }
    else if (data_type == "int8")
    {
        int8_t int8_value = static_cast<int8_t>(value);
        memcpy(&frame_data[2], &int8_value, 1);
    }
    else if (data_type == "uint16")
    {
        uint16_t uint16_value = static_cast<uint16_t>(value);
        memcpy(&frame_data[2], &uint16_value, 2);
    }
    else if (data_type == "int16")
    {
        int16_t int16_value = static_cast<int16_t>(value);
        memcpy(&frame_data[2], &int16_value, 2);
    }
    else if (data_type == "uint32")
    {
        uint32_t uint32_value = static_cast<uint32_t>(value);
        memcpy(&frame_data[2], &uint32_value, 4);
    }
    else if (data_type == "int32")
    {
        int32_t int32_value = static_cast<int32_t>(value);
        memcpy(&frame_data[2], &int32_value, 4);
    }
    else if (data_type == "uint32_uint16")
    {
        int32_t int32_value = static_cast<int32_t>(value);
        memcpy(&frame_data[2], &int32_value, 4);
        int16_t int16_value = static_cast<int16_t>(value2);
        memcpy(&frame_data[6], &int16_value, 2);
    }
    else
    {
        delete[] frame_data;
        throw std::invalid_argument("Unsupported data type");
    }

    return frame_data;
}

int can::send(std::string joint_name, std::string cmd, std::string cmd2, double value, double value2)
{
    std::string data_type = can_data_type[can_address[cmd2]];
    if(cmd == "写入命令" || cmd == "快写命令" )
    {
//        std::cout << joint_name << ", " << cmd << ", " << cmd2 << std::endl;
        return send_whith_id(can_name2id[joint_name],
                     2 + can_data_type_len[data_type],
                     make_data(cmd, cmd2, data_type, value, value2));
    }
    if(cmd == "读取命令")
    {
        unsigned char* frame_data = new unsigned char[8];
        memset(frame_data, 0, 8);
        frame_data[0] = can_cmd[cmd];
        frame_data[1] = can_address[cmd2];
        // std::cout << joint_name << ", " << cmd << ", " << cmd2 << std::endl;
        return send_whith_id(can_name2id[joint_name],
                     2,
                     frame_data);
    }
    if(can_address[cmd2] == 0)
    {
        throw std::invalid_argument("can_address[cmd2] == 0");
    }
    return 0;
}
