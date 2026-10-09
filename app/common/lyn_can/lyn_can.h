#ifndef LYN_CAN_H
#define LYN_CAN_H

#include <thread>
#include <functional>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <fstream>

static std::unordered_map<std::string, unsigned char> can_name2interface =   // 注意：如果 joint_name 不存在，会插入默认值
{
    {"RHipPitch", 1},   //
    {"RHipYaw", 1},
    {"RHipRoll", 1},
    {"RKneePitch", 1},
    {"RAnklePitch", 1},
    {"RAnkleRoll", 1},   //

    {"LHipPitch", 3},
    {"LHipYaw", 3},
    {"LHipRoll", 3},      //
    {"LKneePitch", 3},
    {"LAnklePitch", 3},
    {"LAnkleRoll", 3},

    {"HeadYaw", 2},
    {"HeadPitch", 2},
    {"AbdomeRoll", 2},
    {"AbdomeYaw", 1},
    {"AbdomePitch", 2},

    {"RShoulderPitch", 1},//
    {"RShoulderRoll", 1},
    {"RElbowYaw", 1},
    {"RElbowRoll", 1},
    {"RElbowPitch", 1},
    {"RWristYaw", 1},

    {"LShoulderPitch", 0},
    {"LShoulderRoll", 0},
    {"LElbowYaw", 0},
    {"LElbowPitch", 0},
    {"LElbowRoll", 0},
    {"LWristYaw", 0}
};

static std::unordered_map<std::string, uint> can_name_to_mat_index_map =
{
    {"RHipPitch", 6},
    {"RHipYaw", 8},
    {"RHipRoll", 7},
    {"RKneePitch", 9},
    {"RAnklePitch", 10},
    {"RAnkleRoll", 11},

    {"LHipPitch", 0},      // 0
    {"LHipYaw", 2},
    {"LHipRoll", 1},
    {"LKneePitch", 3},
    {"LAnklePitch", 4},
    {"LAnkleRoll", 5},

    {"HeadYaw", 30},
    {"HeadPitch", 31},
    {"AbdomeRoll", 32},
    {"AbdomeYaw", 12},
    {"AbdomePitch", 33},

    {"RShoulderPitch", 18},
    {"RShoulderRoll", 19},
    {"RElbowYaw", 20},
    {"RElbowPitch", 21},
    {"RElbowRoll", 22},
    {"RWristYaw", 34},

    {"LShoulderPitch", 13},
    {"LShoulderRoll", 14},
    {"LElbowYaw", 15},
    {"LElbowPitch", 16},
    {"LElbowRoll", 17},
    {"LWristYaw", 35}
};

static std::unordered_map<uint, std::string> mat_index_can_name_to_map =
{
    {6, "RHipPitch"},
    {8, "RHipYaw"},
    {7, "RHipRoll"},
    {9, "RKneePitch"},
    {10, "RAnklePitch"},
    {11, "RAnkleRoll"},

    {0, "LHipPitch"},      // 0
    {2, "LHipYaw"},
    {1, "LHipRoll"},
    {3, "LKneePitch"},
    {4, "LAnklePitch"},
    {5, "LAnkleRoll"},

//    {30, "HeadYaw"},
//    {31, "HeadPitch"},
//    {32, "AbdomeRoll"},
    {12, "AbdomeYaw"},
//    {33, "AbdomePitch"},

//    {18, "RShoulderPitch"},
//    {19, "RShoulderRoll"},
//    {20, "RElbowYaw"},
//    {21, "RElbowPitch"},
//    {22, "RElbowRoll"},
//    {34, "RWristYaw"},

//    {13, "LShoulderPitch"},
//    {14, "LShoulderRoll"},
//    {15, "LElbowYaw"},
//    {16, "LElbowPitch"},
//    {17, "LElbowRoll"},
//    {35, "LWristYaw"}
};

static std::unordered_map<std::string, float> motor_direction =
{
    {"RHipPitch", 1},
    {"RHipRoll", 1},
    {"RHipYaw", -1},
    {"RKneePitch", -1},                      //
    {"RAnklePitch", -1},
    {"RAnkleRoll", -1},

    {"LHipPitch", -1},
    {"LHipRoll", 1},
    {"LHipYaw", -1},
    {"LKneePitch", 1},
    {"LAnklePitch", -1},
    {"LAnkleRoll", -1},

    {"HeadYaw", 1},
    {"HeadPitch", 1},
    {"AbdomeRoll", 1},
    {"AbdomeYaw", 1},
    {"AbdomePitch", 1},

    {"RShoulderPitch", -1},
    {"RShoulderRoll", 1},                   //
    {"RElbowYaw", -1},
    {"RElbowRoll", 1},
    {"RElbowPitch", -1},
    {"RWristYaw", 1},

    {"LShoulderPitch", 1},
    {"LShoulderRoll", 1},
    {"LElbowYaw", -1},
    {"LElbowRoll", 1},                      //
    {"LElbowPitch", 1},
    {"LWristYaw", 1}
};

class can
{
public:
    can();
    int run(int argc, char *argv[]);
    int running = 1;
    std::string name = "can0";
    using Callback = std::function<void(std::string, unsigned int&, unsigned char&, unsigned char*)>;
    void set_callback(Callback cb);
    int send_whith_id(unsigned int can_id, unsigned char can_dlc, unsigned char* data);
    int send(std::string joint_name, std::string cmd, std::string cmd2, double value, double value2 = 0);
    void thread_can_read();
    std::unordered_map<std::string, unsigned char> can_name2id =
    {
        {"广播", 0x00},

        {"RHipPitch", 0x01},
        {"RHipRoll", 0x02},
        {"RHipYaw", 0x03},
        {"RKneePitch", 0x04},
        {"RAnklePitch", 0x05},
        {"RAnkleRoll", 0x06},

        {"LHipPitch", 0x21},
        {"LHipRoll", 0x22},
        {"LHipYaw", 0x23},
        {"LKneePitch", 0x24},
        {"LAnklePitch", 0x25},
        {"LAnkleRoll", 0x26},

        {"HeadYaw", 0x41},
        {"HeadPitch", 0x42},
        {"AbdomeRoll", 0x43},
        {"AbdomeYaw", 0x44},
        {"AbdomePitch", 0x45},

        {"RShoulderPitch", 0x61},
        {"RShoulderRoll", 0x62},
        {"RElbowYaw", 0x63},
        {"RElbowPitch", 0x64},
        {"RElbowRoll", 0x65},
        {"RWristYaw", 0x66},

        {"LShoulderPitch", 0x81},
        {"LShoulderRoll", 0x82},
        {"LElbowYaw", 0x83},
        {"LElbowPitch", 0x84},
        {"LElbowRoll", 0x85},
        {"LWristYaw", 0x86},
    };

    std::unordered_map<unsigned int, std::string> can_id2name = {
        {0x00, "广播"},

        {0x01, "RHipPitch"},
        {0x02, "RHipRoll"},
        {0x03, "RHipYaw"},
        {0x04, "RKneePitch"},
        {0x05, "RAnklePitch"},
        {0x06, "RAnkleRoll"},

        {0x21, "LHipPitch"},
        {0x22, "LHipRoll"},
        {0x23, "LHipYaw"},
        {0x24, "LKneePitch"},
        {0x25, "LAnklePitch"},
        {0x26, "LAnkleRoll"},

        {0x41, "HeadYaw"},
        {0x42, "HeadPitch"},
        {0x43, "AbdomeRoll"},
        {0x44, "AbdomeYaw"},
        {0x45, "AbdomePitch"},

        {0x61, "RShoulderPitch"},
        {0x62, "RShoulderRoll"},
        {0x63, "RElbowYaw"},
        {0x64, "RElbowPitch"},
        {0x65, "RElbowRoll"},
        {0x66, "RWristYaw"},

        {0x81, "LShoulderPitch"},
        {0x82, "LShoulderRoll"},
        {0x83, "LElbowYaw"},
        {0x84, "LElbowPitch"},
        {0x85, "LElbowRoll"},
        {0x86, "LWristYaw"},
        {0xFF, "主机网络诊断"},
    };

    std::unordered_map<std::string, std::string> can_name_white =
    {
        {"广播", "          "},

        {"RHipPitch", "     "},
        {"RHipYaw", "       "},
        {"RHipRoll", "      "},
        {"RKneePitch", "    "},
        {"RAnklePitch", "   "},
        {"RAnkleRoll", "    "},

        {"LHipPitch", "     "},
        {"LHipYaw", "       "},
        {"LHipRoll", "      "},
        {"LKneePitch", "    "},
        {"LAnklePitch", "   "},
        {"LAnkleRoll", "    "},

        {"HeadYaw", "       "},
        {"HeadPitch", "     "},
        {"AbdomeRoll", "    "},
        {"AbdomeYaw", "     "},
        {"AbdomePitch", "   "},

        {"RShoulderPitch", ""},
        {"RShoulderRoll", " "},
        {"RElbowYaw", "     "},
        {"RElbowRoll", "    "},
        {"RElbowPitch", "   "},
        {"RWristYaw", "     "},

        {"LShoulderPitch", ""},
        {"LShoulderRoll", " "},
        {"LElbowYaw", "     "},
        {"LElbowRoll", "    "},
        {"LElbowPitch", "   "},
        {"LWristYaw", "     "},
    };

    enum can_data
    {
        CMD,
        CMD2_Address,
        data0,
        data1,
        data2,
        data3,
        data4,
        data5
    };

    std::unordered_map<unsigned char, std::string> can_cmd_name =
    {
        {0x00, "网络管理"},
        {0x01, "写入命令"},
        {0x02, "写入命令返回"},
        {0x03, "读取命令"},
        {0x04, "读取命令返回"},
        {0x05, "快写命令"},
        {0x06, "升级命令"},
        {0x07, "升级命令返回"}
    };

    std::unordered_map<std::string, unsigned char> can_cmd =
    {
        {"网络管理", 0x00},
        {"写入命令", 0x01},
        {"写入命令返回", 0x02},
        {"读取命令", 0x03},
        {"读取命令返回", 0x04},
        {"快写命令", 0x05},
        {"升级命令", 0x06},
        {"升级命令返回", 0x07}
    };

    std::unordered_map<unsigned char, std::string> can_address_name =
    {
        {0x01, "设备型号"},
        {0x02, "设备序列号"},
        {0x03, "UID"},
        {0x04, "硬件版本号"},
        {0x05, "软件版本号"},
        {0x06, "电机型号"},
        {0x07, "bootloader软件版本号"},

        {0x10, "执行器输出轴当前绝对位置、速度、力矩"},
        {0x11, "错误码"},
        {0x12, "关节当前母线电压&母线电流值"},
        {0x13, "电机当前id"},
        {0x14, "电机当前iq"},
        {0x15, "关节当前温度值"},
        {0x16, "电机当前电角度"},
        {0x17, "高速编码器"},
        {0x18, "低速编码器"},
        {0x19, "输出轴位置"},
        {0x1A, "软件限位"},

        {0x20, "减速比"},
        {0x21, "输出轴方向"},
        {0x22, "最大位置"},
        {0x23, "最小位置"},
        {0x24, "电机最大相电流(峰值)"},
        {0x25, "电机最大扭矩"},
        {0x26, "电机最大转速"},
        {0x27, "最大加速度"},

        {0x30, "使能/失能状态"},
        {0x31, "设置工作模式"},
        {0x32, "设置电机目标id"},
        {0x33, "设置电机目标iq"},
        {0x34, "设置关节目标扭矩"},
        {0x35, "设置关节目标位置"},
        {0x36, "设置关节最大加速度"},
        {0x37, "设置关节最大减速度"},
        {0x38, "设置关节目标速度"},
        {0x39, "设置角度模式"},

        {0x40, "电流环id kp"},
        {0x41, "电流环id ki"},
        {0x42, "电流环iq kp"},
        {0x43, "电流环iq ki"},
        {0x44, "转速环kp"},
        {0x45, "转速环ki"},
        {0x46, "位置环kp"},
        {0x47, "位置环ki"},
        {0x48, "PD模式kp"},
        {0x49, "PD模式kd"},

        {0x50, "关节过压保护值"},
        {0x51, "关节欠压保护值"},
        {0x52, "电机过流保护值(峰值)"},
        {0x53, "MOS温度保护值"},
        {0x54, "电机温度保护值"},

        {0xE0, "调试用参数1"},
        {0xE1, "调试用参数2"},
        {0xE2, "调试用参数3"},

        {0xF0, "重启设备"},
        {0xF1, "故障清除"},
        {0xF2, "修改 CAN 波特率"},
        {0xF3, "修改设备的CAN ID"},
        {0xF4, "数据保存"},
        {0xF5, "电角度零位校准"},
        {0xF6, "型号配置指令"},
        {0xF7, "输出轴设置零位"},
        {0xF8, "使能软件限位"},
        {0xF9, "修改金刚编码器波特率"},
    };

    std::unordered_map<std::string, unsigned char> can_address =
    {
        {"设备型号", 0x01},
        {"设备序列号", 0x02},
        {"UID", 0x03},
        {"硬件版本号", 0x04},
        {"软件版本号", 0x05},
        {"电机型号", 0x06},
        {"bootloader软件版本号", 0x07},

        {"执行器输出轴当前绝对位置、速度、力矩", 0x10},
        {"错误码", 0x11},
        {"关节当前母线电压&母线电流值", 0x12},
        {"电机当前id", 0x13},
        {"电机当前iq", 0x14},
        {"关节当前温度值", 0x15},
        {"电机当前电角度", 0x16},
        {"高速编码器", 0x17},
        {"低速编码器", 0x18},
        {"输出轴位置", 0x19},
        {"软件限位", 0x1A},

        {"减速比", 0x20},
        {"输出轴方向", 0x21},
        {"最大位置", 0x22},
        {"最小位置", 0x23},
        {"电机最大相电流(峰值)", 0x24},
        {"电机最大扭矩", 0x25},
        {"电机最大转速", 0x26},
        {"最大加速度", 0x27},

        {"使能/失能状态", 0x30},
        {"设置工作模式", 0x31},
        {"设置电机目标id", 0x32},
        {"设置电机目标iq", 0x33},
        {"设置关节目标扭矩", 0x34},
        {"设置关节目标位置", 0x35},
        {"设置关节最大加速度", 0x36},
        {"设置关节最大减速度", 0x37},
        {"设置关节目标速度", 0x38},
        {"设置角度模式", 0x39},

        {"电流环id kp", 0x40},
        {"电流环id ki", 0x41},
        {"电流环iq kp", 0x42},
        {"电流环iq ki", 0x43},
        {"转速环kp", 0x44},
        {"转速环ki", 0x45},
        {"位置环kp", 0x46},
        {"位置环ki", 0x47},
        {"PD模式kp", 0x48},
        {"PD模式kd", 0x49},

        {"关节过压保护值", 0x50},
        {"关节欠压保护值", 0x51},
        {"电机过流保护值(峰值)", 0x52},
        {"MOS温度保护值", 0x53},
        {"电机温度保护值", 0x54},

        {"调试用参数1", 0xE0},
        {"调试用参数2", 0xE1},
        {"调试用参数3", 0xE2},

        {"重启设备", 0xF0},
        {"故障清除", 0xF1},
        {"修改 CAN 波特率", 0xF2},
        {"修改设备的CAN ID", 0xF3},
        {"数据保存", 0xF4},
        {"电角度零位校准", 0xF5},
        {"型号配置指令", 0xF6},
        {"输出轴设置零位", 0xF7},
        {"使能软件限位", 0xF8},
        {"修改金刚编码器波特率", 0xF9},
    };

    std::unordered_map<unsigned char, std::string> can_data_type =
    {
        {0x01, "uint32"},
        {0x02, "uint32"},
        {0x03, "uint32"},
        {0x04, "uint32"},
        {0x05, "uint32"},
        {0x06, "uint16"},
        {0x07, "uint32"},

        {0x10, "int16_int16_int16"},
        {0x11, "uint32"},
        {0x12, "int16_int16"},
        {0x13, "float"},
        {0x14, "float"},
        {0x15, "int16_int16"},
        {0x16, "float"},
        {0x17, "uint32"},
        {0x18, "uint32"},
        {0x19, "float"},
        {0x1A, "float"},

        {0x20, "float"},
        {0x21, "uint8"},
        {0x22, "float"},
        {0x23, "float"},
        {0x24, "float"},
        {0x25, "float"},
        {0x26, "float"},
        {0x27, "float"},

        {0x30, "uint8"},
        {0x31, "uint8"},
        {0x32, "float"},
        {0x33, "float"},
        {0x34, "float"},
        {0x35, "float"},
        {0x36, "float"},
        {0x37, "float"},
        {0x38, "float"},
        {0x39, "uint8"},

        {0x40, "float"},
        {0x41, "float"},
        {0x42, "float"},
        {0x43, "float"},
        {0x44, "float"},
        {0x45, "float"},
        {0x46, "float"},
        {0x47, "float"},
        {0x48, "float"},
        {0x49, "float"},

        {0x50, "float"},
        {0x51, "float"},
        {0x52, "float"},
        {0x53, "float"},
        {0x54, "float"},

        {0xE0, "float"},
        {0xE1, "float"},
        {0xE2, "float"},

        {0xF0, "uint16"},
        {0xF1, "uint16"},
        {0xF2, "uint16"},
        {0xF3, "uint16"},
        {0xF4, "uint16"},
        {0xF5, "uint16"},
        {0xF6, "uint16"},
        {0xF7, "uint16"},
        {0xF8, "uint16"},
        {0xF9, "uint32"},
    };

    std::unordered_map<std::string, unsigned char> can_data_type_len =
    {
        {"float", 4},
        {"uint8", 1},
        {"int8", 1},
        {"uint16", 2},
        {"int16", 2},
        {"uint32", 4},
        {"int32", 4},
        {"uint32_uint16", 6},
    };

private:
    std::thread thread;

    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int nbytes;
    Callback callback_;
    unsigned char *make_data(std::string CMD,
                             std::string can_address,
                             std::string data_type,
                             double value,
                             double value2 = 0);
};

#endif // LYN_CAN_H
