#include <can_in.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

static lyn_log logger_can_in;
static lyn_log logger_can_heartbeat;
static std::shared_mutex render_mutex_can_in;
static can CAN;
static std::unordered_map<std::string, lyn_Infos> output_public;
static cv::Mat axis_absolute_position;
static cv::Mat output_shaft_position;
static cv::Mat motor_UIDs;

static std::unordered_map<std::string, std::condition_variable> condition_variable;
static std::unordered_map<std::string, std::atomic<lyn_step>> step;

static std::unordered_map<std::string, lyn_Infos> input_public;

can_in::can_in()
{
    step["can_data_mat_infos"].store(lyn_step_free);
    axis_absolute_position = cv::Mat(1, 128, CV_32FC1);
    axis_absolute_position.setTo(0);
    output_shaft_position = cv::Mat(1, 128, CV_32FC1);
    output_shaft_position.setTo(0);
    motor_UIDs = cv::Mat(1, 128, CV_32FC1);
    motor_UIDs.setTo(0);
    logger_can_in.run("can_in.log", false);
}

std::string charToHexString2(uchar c)
{
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%02X", c);
    return std::string(buffer);
}

void log_data(unsigned char& can_dlc, unsigned char* data)
{
    for (int i = 0; i < can_dlc; ++i)
    {
        logger_can_in << charToHexString2(data[i]) << " ";
    }
    logger_can_in << " ";
}

std::vector<double> parse_data(std::string data_type, unsigned char* dat)
{
    std::vector<double> re;
    if (data_type == "float")
    {
        float float_value = 0.0f;
        memcpy(&float_value, &dat[2], 4);
        re.push_back(double(float_value));
        return re;
    }

    if (data_type == "uint8")
    {
        re.push_back(uint8_t(dat[2]));
        return re;
    }

    if (data_type == "int8")
    {
        re.push_back(int8_t(dat[2]));
        return re;
    }

    if (data_type == "uint16")
    {
        uint16_t uint16_value = 0;
        memcpy( &uint16_value, &dat[2], 2);
        re.push_back(double(uint16_value));
        return re;
    }

    if (data_type == "int16")
    {
        int16_t int16_value = 0;
        memcpy(&int16_value, &dat[2], 2);
        re.push_back(double(int16_value));
        return re;
    }

    if (data_type == "int16_int16")
    {
        int16_t int16_value = 0;
        memcpy(&int16_value, &dat[2], 2);
        re.push_back(int16_value);
        memcpy(&int16_value, &dat[4], 2);
        re.push_back(int16_value);
        return re;
    }

    if (data_type == "int16_int16_int16")
    {
        int16_t int16_value = 0;
        memcpy(&int16_value, &dat[2], 2);
        re.push_back(int16_value);
        memcpy(&int16_value, &dat[4], 2);
        re.push_back(int16_value);
        memcpy(&int16_value, &dat[6], 2);
        re.push_back(int16_value);
        return re;
    }

    if (data_type == "uint32")
    {
        uint32_t uint32_value = 0;
        memcpy(&uint32_value, &dat[2], 4);
        re.push_back(uint32_value);
        return re;
    }

    if (data_type == "int32")
    {
        int32_t int32_value = 0.0;
        memcpy(&int32_value, &dat[2], 4);
        re.push_back(int32_value);
        return re;
    }
    return re;
}

static inline float deg2rad(float d) { return d * float(M_PI) / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / float(M_PI); }

struct Vec3
{
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
};

static inline float norm3(const Vec3& v)
{
    return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static inline float clamp(float v, float lo, float hi)
{
    return std::max(lo, std::min(v, hi));
}

static inline float normalize_angle(float a)
{
    // normalize to [-pi, pi]
    float two_pi = 2.0f * float(M_PI);
    a = std::fmod(a + float(M_PI), two_pi);
    if (a < 0) a += two_pi;
    return a - float(M_PI);
}

static float select_valid_angle(float angle1, float angle2)
{
    float a1 = normalize_angle(angle1);
    float a2 = normalize_angle(angle2);

    bool v1 = (a1 >= -float(M_PI)/2 && a1 <= float(M_PI)/2);
    bool v2 = (a2 >= -float(M_PI)/2 && a2 <= float(M_PI)/2);

    if (v1 && v2) return (std::fabs(a1) <= std::fabs(a2)) ? a1 : a2;
    if (v1) return a1;
    if (v2) return a2;

    // fallback
    float a = normalize_angle(a1);
    if (a > float(M_PI)/2) a -= float(M_PI);
    if (a < -float(M_PI)/2) a += float(M_PI);
    return a;
}

// -----------------------------
// model parameters (copy from MATLAB)
struct AnkleParams {
    float l_axis   = 0.03685f;
    float l_axis_C = 0.033f;
    float l_bar    = 0.0225f;
    float l_bar_c  = 0.0185f;
    float l_rod1   = 0.2105f;
    float l_rod2   = 0.1455f;
    float l_2      = 0.0f;
    float lz       = 0.01f;

    Vec3 C1_0 = Vec3(-l_axis_C,  l_bar_c, -lz);
    Vec3 C2_0 = Vec3(-l_axis_C, -l_bar_c, -lz);
    Vec3 A1_0 = Vec3(-l_axis, 0.0f, 0.200f);
    Vec3 A2_0 = Vec3(-l_axis, 0.0f, 0.135f);
};
// -----------------------------
// compute_rotated_C() same as MATLAB: T_total = T_trans2*T_pitch*T_trans*T_roll
static Vec3 compute_rotated_C(float roll, float pitch, const Vec3& C0, float l_2) {
    // roll about X
    float cr = std::cos(roll), sr = std::sin(roll);
    // pitch about Y
    float cp = std::cos(pitch), sp = std::sin(pitch);

    // Apply roll first around X
    Vec3 p = C0;
    Vec3 p_roll(
        p.x,
        cr * p.y - sr * p.z,
        sr * p.y + cr * p.z
    );

    // T_trans: z -= l_2
    Vec3 p_t1(p_roll.x, p_roll.y, p_roll.z - l_2);

    // pitch about Y
    Vec3 p_pitch(
        cp * p_t1.x + sp * p_t1.z,
        p_t1.y,
        -sp * p_t1.x + cp * p_t1.z
    );

    // T_trans2: z += l_2
    Vec3 p_out(p_pitch.x, p_pitch.y, p_pitch.z + l_2);
    return p_out;
}

// -----------------------------
// compute_theta() same as MATLAB
static void compute_theta(const Vec3& C1, const Vec3& C2,
                          const Vec3& A1, const Vec3& A2,
                          float l_rod1, float l_rod2, float l_bar,
                          float& theta1, float& theta2) {
    float dy1 = C1.y - A1.y;
    float dz1 = C1.z - A1.z;
    float dy2 = C2.y - A2.y;
    float dz2 = C2.z - A2.z;

    float norm_CA1 = norm3(C1 - A1);
    float norm_CA2 = norm3(C2 - A2);

    float c1 = (l_rod1*l_rod1 - l_bar*l_bar - norm_CA1*norm_CA1) / (-2.0f * l_bar);
    float c2 = (l_rod2*l_rod2 - l_bar*l_bar - norm_CA2*norm_CA2) / ( 2.0f * l_bar);

    float denom1 = std::sqrt(dy1*dy1 + dz1*dz1);
    float denom2 = std::sqrt(dy2*dy2 + dz2*dz2);

    // avoid zero
    denom1 = std::max(denom1, 1e-8f);
    denom2 = std::max(denom2, 1e-8f);

    float arg1 = clamp(c1 / denom1, -1.0f, 1.0f);
    float arg2 = clamp(c2 / denom2, -1.0f, 1.0f);

    float phi1 = std::atan2(dy1, dz1);
    float phi2 = std::atan2(dy2, dz2);

    float asin1 = std::asin(arg1);
    float asin2 = std::asin(arg2);

    float t11 = asin1 - phi1;
    float t12 = (float(M_PI) - asin1) - phi1;

    float t21 = asin2 - phi2;
    float t22 = (float(M_PI) - asin2) - phi2;

    theta1 = select_valid_angle(t11, t12);
    theta2 = select_valid_angle(t21, t22);
}

// -----------------------------
// FK solver: given theta1/theta2 -> solve roll/pitch
static bool solve_roll_pitch_from_motor(float theta1_target, float theta2_target,
                                        float& roll, float& pitch,
                                        const AnkleParams& P, int L)
{
    // initial guess (good default)
    roll  = 0.0f;
    pitch = 0.0f;

    const int max_iter = 40;
    const float eps = 1e-6f;
    const float h = 1e-4f;     // numeric diff step
    const float lambda = 0.6f; // damping

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Vec3 C1 = compute_rotated_C(roll, pitch, P.C1_0, P.l_2);
        Vec3 C2 = compute_rotated_C(roll, pitch, P.C2_0, P.l_2);

        float t1, t2;
        compute_theta(C1, C2, P.A1_0, P.A2_0, P.l_rod1, P.l_rod2, P.l_bar, t1, t2);

        float f1 = t1 - theta1_target;
        float f2 = t2 - theta2_target;

        float err = std::sqrt(f1*f1 + f2*f2);
        if (err < 1e-4f) return true;

        // numeric Jacobian J = dF/d[roll,pitch]
        // roll + h
        Vec3 C1_r = compute_rotated_C(roll + h, pitch, P.C1_0, P.l_2);
        Vec3 C2_r = compute_rotated_C(roll + h, pitch, P.C2_0, P.l_2);
        float t1_r, t2_r;
        compute_theta(C1_r, C2_r, P.A1_0, P.A2_0, P.l_rod1, P.l_rod2, P.l_bar, t1_r, t2_r);

        // pitch + h
        Vec3 C1_p = compute_rotated_C(roll, pitch + h, P.C1_0, P.l_2);
        Vec3 C2_p = compute_rotated_C(roll, pitch + h, P.C2_0, P.l_2);
        float t1_p, t2_p;
        compute_theta(C1_p, C2_p, P.A1_0, P.A2_0, P.l_rod1, P.l_rod2, P.l_bar, t1_p, t2_p);

        float J11 = (t1_r - t1) / h; // df1/droll
        float J21 = (t2_r - t2) / h; // df2/droll
        float J12 = (t1_p - t1) / h; // df1/dpitch
        float J22 = (t2_p - t2) / h; // df2/dpitch

        // Solve linear system: J * delta = -F
        // 2x2 inverse
        float det = J11*J22 - J12*J21;
        if (std::fabs(det) < 1e-8f) return false;

        float inv11 =  J22 / det;
        float inv12 = -J12 / det;
        float inv21 = -J21 / det;
        float inv22 =  J11 / det;

        float droll  = -(inv11*f1 + inv12*f2);
        float dpitch = -(inv21*f1 + inv22*f2);

        // damping
        roll  += lambda * droll;
        pitch += lambda * dpitch;

        // optional clamp to reasonable range
        roll  = clamp(roll,  deg2rad(-45.0f), deg2rad(45.0f));
        pitch = clamp(pitch, deg2rad(-45.0f), deg2rad(45.0f));
    }
    return false;
}

std::vector<float> calculate_ankle_axis_angle(float motor_angle_1, float motor_angle_2, int L)
{
    (void)L; // if you want later: use L to select different mechanism parameter set

    AnkleParams P; // matches your MATLAB parameters
    if(L)
    {
        P. l_axis   = 0.03685f;
        P. l_axis_C = 0.033f;
        P. l_bar    = 0.0225f;
        P. l_bar_c  = 0.0185f;
        P. l_rod1   = 0.2105f;
        P. l_rod2   = 0.1455f;
        P. l_2      = 0.0f;
        P. lz       = 0.01f;

        P. C1_0 = Vec3(-P.l_axis_C,  P.l_bar_c, -P.lz);
        P. C2_0 = Vec3(-P.l_axis_C, -P.l_bar_c, -P.lz);
        P. A1_0 = Vec3(-P.l_axis, 0.0f, 0.200f);
        P. A2_0 = Vec3(-P.l_axis, 0.0f, 0.135f);
    }
    else
    {
        P. l_axis   = 0.03685f;
        P. l_axis_C = 0.033f;
        P. l_bar    = 0.0225f;
        P. l_bar_c  = 0.0185f;
        P. l_rod1   = 0.1455f;
        P. l_rod2   = 0.2105f;
        P. l_2      = 0.0f;
        P. lz       = 0.01f;

        P. C1_0 = Vec3(-P.l_axis_C,  P.l_bar_c, -P.lz);
        P. C2_0 = Vec3(-P.l_axis_C, -P.l_bar_c, -P.lz);
        P. A1_0 = Vec3(-P.l_axis, 0.0f, 0.135f);
        P. A2_0 = Vec3(-P.l_axis, 0.0f, 0.200f);
    }

    // theta targets in rad
    float theta1_target = L ? motor_angle_1 : motor_angle_2;
    float theta2_target = L ? motor_angle_2 : motor_angle_1;

    float roll = 0.0f, pitch = 0.0f;
    bool ok = solve_roll_pitch_from_motor(theta1_target, theta2_target, roll, pitch, P, L);

    float AnkleRoll  = ok ? roll  : std::numeric_limits<float>::quiet_NaN();
    float AnklePitch = ok ? pitch : std::numeric_limits<float>::quiet_NaN();
#if 1
    {
        logger_can_in << "calculate_ankle_axis_angle: " << (L ? "L " : "R ")
                  << std::setw(10) << AnklePitch * 180.0f / 3.1415926f << " "
                  << std::setw(10) << AnkleRoll * 180.0f / 3.1415926f << " "
                  << std::setw(10) << double(motor_angle_1) * 180.0 / 3.1415926 << " "
                  << std::setw(10) << double(motor_angle_2) * 180.0 / 3.1415926 << " "
                  << std::endl;
    }

#endif
    // 你想要输出 pitch/roll 顺序我就按你函数声明的变量来
    return {AnklePitch, AnkleRoll};
}

double maxDifference(const cv::Mat& img1, const cv::Mat& img2)
{
    cv::Mat diff;
    cv::absdiff(img1, img2, diff);
    double min_val, max_val;
    cv::Point min_loc, max_loc;
    for (int mat_index = 0; mat_index < 23; ++mat_index)
    {
        if(diff.at<float>(0, mat_index) > float(1.0 * DEG))
        {
            std::cout << std::endl << "轴位置异常："
                      << mat_index_can_name_to_map[uint(mat_index)] << " "
                      << diff.at<float>(0, mat_index) << "弧度 （"
                      << diff.at<float>(0, mat_index) * (180.0f / 3.1415926f)<< "度)";
        }
        if (img1.at<float>(0, mat_index) == 0.0f && img2.at<float>(0, mat_index) == 0.0f)
        {
            std::cout << std::endl << "轴位置读数异常："
                      << mat_index_can_name_to_map[uint(mat_index)] << " = 0 ";
        }
    }
    cv::minMaxLoc(diff, &min_val, &max_val, &min_loc, &max_loc);
    return max_val;
}

cv::Mat can_in::motor_angle2axis_angle(cv::Mat &joint_angles)
{
    cv::Mat new_joint_angles = joint_angles.clone();
    std::vector<float> ankle_axis_angleL = calculate_ankle_axis_angle
            (joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnklePitch"])),
            joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnkleRoll"])), 1);

    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnklePitch"])) = ankle_axis_angleL[0];
    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["LAnkleRoll"])) = ankle_axis_angleL[1];

    std::vector<float> ankle_axis_angleR = calculate_ankle_axis_angle
            (joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnklePitch"])),
            joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnkleRoll"])), 0);

    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnklePitch"])) = ankle_axis_angleR[0];
    new_joint_angles.at<float>(0, int(can_name_to_mat_index_map["RAnkleRoll"])) = ankle_axis_angleR[1];

    return new_joint_angles;
}

static std::unordered_map<std::string, int> motor_ready;
static std::unordered_map<std::string, int> motor_ready2;

static int joint_lost_axis_absolute = 0;
static int joint_lost_shaft_position = 0;
static int check_pass = 0;
static int num_of_motor = 13;

std::string unsignedCharArrayToHex(const unsigned char* data, size_t len) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i) {
        ss << std::setw(2) << static_cast<int>(data[i]);
        if (i < len - 1) ss << " "; // 添加空格分隔
    }
    return ss.str();
}

void can_in::can_call_back(std::string name,
                           unsigned int& can_id,
                           unsigned char& can_dlc,
                           unsigned char* dat)
{
    std::unique_lock<std::shared_mutex> exclusive_lock(render_mutex_can_in);                       //保证数据打印整齐
    std::string motor_name = CAN.can_id2name[can_id];
    std::string motor_name_white = CAN.can_name_white[motor_name];

    if((dat[can::CMD] == CAN.can_cmd["网络管理"]) && (dat[can::CMD2_Address] == 0x81))
    {
        long ID = 0;
        memcpy(&ID, &dat[2], 4);
        log_data(can_dlc, dat);
        logger_can_in << name.c_str() << " "
                      << motor_name << motor_name_white << " "
                      << "ID" << " " << ID << " "
                      << charToHexString2(dat[6]) << " "
                                      << std::endl;
    }

    if((dat[can::CMD] == CAN.can_cmd["网络管理"]) && (dat[can::CMD2_Address] == 0x21))
    {
        long UID = 0;
        memcpy(&UID, &dat[2], 4);
        logger_can_in << name.c_str() << " "
                      << motor_name << motor_name_white  << " "
                      << "关节心跳" << " "
                      << "UID" << " " << UID << " "
                      << "heartbeat " << int(dat[6]) << " "
                                      << std::endl;
    }

    if(dat[can::CMD] == CAN.can_cmd["读取命令返回"])
    {
        if(dat[can::CMD2_Address] == 0x02 && can_dlc == 2)
        {
            logger_can_in << name.c_str() << " "
                          << motor_name << motor_name_white  << " ";
            logger_can_in << int(can_dlc) << " " "失败"
                          << std::endl;
        }
        else
        {
            logger_can_in << unsignedCharArrayToHex(dat, 8) << " "
                          << name.c_str() << " "
                          << motor_name << motor_name_white  << " "
                          << " [" << (can_id > 9 ? "" : "0") << int(can_id) << "] "
                          << int(can_dlc) << " "
                          << CAN.can_cmd_name[dat[can::CMD]] << " "
                          << CAN.can_address_name[dat[can::CMD2_Address]] << " ";
            std::vector<double> re = parse_data(CAN.can_data_type[dat[can::CMD2_Address]], dat);
            for (uint i = 0; i < re.size(); ++i) { logger_can_in << std::fixed << re[i] << ", " ; }
            if(dat[can::CMD2_Address] == CAN.can_address["执行器输出轴当前绝对位置、速度、力矩"])
            {
                int index = int(can_name_to_mat_index_map[motor_name]);
                float value = float(re[0] / 32768.0 * 2 * 3.1415926535897932384) * motor_direction[motor_name];
                axis_absolute_position.at<float> (0, index) = value;
#if 0
                if (motor_name == "RAnklePitch" || motor_name == "LAnklePitch")
                {
                    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " <<
                              motor_name << " " << value * float(RAD) << std::endl;
                }
                print_mat_formatted(axis_absolute_position * RAD, 2);
#endif
                motor_ready[motor_name] = 1;
                int count_ready = 0;
                for (const auto& pair : motor_ready)
                {
                    if (pair.second == 1.0)
                    {
                        count_ready++;
                    }
                }
                if(count_ready == num_of_motor)
                {
#if 0
                    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                              << cv::Mat(motor_angle2axis_angle(axis_absolute_position), cv::Rect(0, 0, 23, 1)) << std::endl;
#endif
                    update_info("can_data_mat_infos", "joint_angles",
                                             motor_angle2axis_angle(axis_absolute_position).clone());
                    update_info("can_data_mat_infos_direct_control", "joint_angles",
                                             motor_angle2axis_angle(axis_absolute_position).clone());
                    if(!check_pass && maxDifference(output_shaft_position, axis_absolute_position) > 1.0 * DEG)         // 误差大于1度则报异常
                    {
                        logger_can_in << "[" << getCurrentTimeWithMicroseconds() << "] "
                                      << std::endl << "轴位置异常!" << std::endl;
                        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                                  << std::endl << "轴位置异常!" << std::endl;
                    }
                    joint_lost_axis_absolute = 0;
                    if(!check_pass)
                    {
                        check_pass = 1;
                        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                                  << std::endl << "电机数量正常" << std::endl;
                    }
                    motor_ready.clear();
                } else
                {
                    joint_lost_axis_absolute ++;
                    if(joint_lost_axis_absolute > 180)
                    {
                        for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
                        {
                             std::string joint_name = mat_index_can_name_to_map[index];
#if 0
                            if(motor_ready[joint_name] != 1 && joint_name != "")
                            {
                                std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                                           << "关节失联: " << joint_name << std::endl;
                            }
#endif
                        }
#if 0
                        std::cout << std::endl;
#endif
                        joint_lost_axis_absolute = 0;
                    }
                }
            }

            if(dat[can::CMD2_Address] == CAN.can_address["输出轴位置"])
            {
                output_shaft_position.at<float>
                        (0, int(can_name_to_mat_index_map[motor_name])) =
                        float(re[0]) *
                        motor_direction[motor_name];
                motor_ready2[motor_name] = 1;
                int count_ready = 0;
                for (const auto& pair : motor_ready2)
                {
                    if (pair.second == 1.0f)
                    {
                        count_ready++;
                    }
                }

                if(count_ready == num_of_motor)   // ==23才能看到界面
                {
#if 0
                    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                              << output_shaft_position << std::endl;
#endif
                    update_info("shaft_position", "joint_angles",
                                             motor_angle2axis_angle(output_shaft_position).clone());
                    joint_lost_shaft_position = 0;
                    motor_ready2.clear();
                } else
                {
                    joint_lost_shaft_position ++;
                    if(joint_lost_shaft_position > 80)
                    {
                        for (uint index = 0; index < mat_index_can_name_to_map.size(); index++)
                        {
                            std::string joint_name = mat_index_can_name_to_map[index];
                            if(motor_ready2[joint_name] != 1)
                            {
                                std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                                          << "关节失去连接: " << joint_name << std::endl;
                                logger_can_in << "[" << getCurrentTimeWithMicroseconds() << "] "
                                              << "关节失去连接: " << joint_name << std::endl;
                            }
                        }
                        std::cout << std::endl;
                        joint_lost_shaft_position = 0;
                    }
                }
            }
            if(dat[can::CMD2_Address] == CAN.can_address["UID"])
            {
                motor_UIDs.at<float>(0, int(can_name_to_mat_index_map[motor_name])) = uint(re[0]);
                update_info("motor_UIDs", "motor_UIDs", motor_UIDs.clone());
            }
            logger_can_in << std::endl;
        }
    }

    if(dat[can::CMD] == CAN.can_cmd["写入命令返回"])
    {
        log_data(can_dlc, dat);
        logger_can_in << name.c_str() << " "
                      << motor_name
                      << motor_name_white << " ";
        logger_can_in << int(can_dlc) << " ";
        logger_can_in << CAN.can_cmd_name[dat[can::CMD]] << " ";
        logger_can_in << ((dat[can::CMD2_Address] == 0x01) ? "成功" : "失败") << " ";
        logger_can_in << std::endl;
    }

    if(dat[can::CMD] == CAN.can_cmd["升级命令返回"])
    {
        log_data(can_dlc, dat);
        logger_can_in << name.c_str() << " "
                      << motor_name << " ";
        logger_can_in << int(can_dlc) << " ";
        logger_can_in << CAN.can_cmd_name[dat[can::CMD]] << " ";
        logger_can_in << CAN.can_address_name[dat[can::CMD2_Address]] << " ";
        logger_can_in << std::endl;
    }
}

int can_in::run(int argc, char *argv[])
{
    for (int i = 0; i < 4; i++)
    {
        Can[i].name = "can" + std::to_string(i);
        Can[i].set_callback(can_call_back);
        Can[i].run(argc, argv);
    }

    return 0;
}

void can_in::update_info(std::string name, std::string Info_name, cv::Mat mat)
{
    if(step[name].load() == lyn_step_updating)
    {
        lyn_Info Info;
        Info[Info_name] = mat ;
        output_public[name].clear();
        output_public[name].push_back(Info);
        step[name].store(lyn_step_update_complete);
    }
}

int can_in::work(lyn_info &info)
{
    info.load(names["can_data_mat_infos"], step, output_public);
    info.load(names["can_data_mat_infos_direct_control"], step, output_public);
    info.load(names["motor_UIDs"], step, output_public);
    return 0;
}
