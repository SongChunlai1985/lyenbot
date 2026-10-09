#include <lyn_log.h>
#include <lyn_meta.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>

static inline double normalize_angle(double a)
{
    a = std::fmod(a + CV_PI, 2.0 * CV_PI);
    if (a < 0) a += 2.0 * CV_PI;
    return a - CV_PI;
}

static inline double select_valid_angle(double a1, double a2)
{
    double n1 = normalize_angle(a1);
    double n2 = normalize_angle(a2);

    bool v1 = (n1 >= -CV_PI/2.0) && (n1 <=  CV_PI/2.0);
    bool v2 = (n2 >= -CV_PI/2.0) && (n2 <=  CV_PI/2.0);

    if (v1 && v2) return (std::abs(n1) <= std::abs(n2)) ? n1 : n2;
    if (v1) return n1;
    if (v2) return n2;

    double s = normalize_angle(n1);
    if (s >  CV_PI/2.0) s -= CV_PI;
    if (s < -CV_PI/2.0) s += CV_PI;
    return s;
}

static inline cv::Mat make_T_roll_x(double roll)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(1,1) =  std::cos(roll);
    T.at<double>(1,2) = -std::sin(roll);
    T.at<double>(2,1) =  std::sin(roll);
    T.at<double>(2,2) =  std::cos(roll);
    return T;
}

static inline cv::Mat make_T_pitch_y(double pitch)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(0,0) =  std::cos(pitch);
    T.at<double>(0,2) =  std::sin(pitch);
    T.at<double>(2,0) = -std::sin(pitch);
    T.at<double>(2,2) =  std::cos(pitch);
    return T;
}

static inline cv::Mat make_T_trans_z(double dz)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(2,3) = dz;
    return T;
}

static inline cv::Point3d apply_T(const cv::Mat& T, const cv::Point3d& p)
{
    cv::Mat p4 = (cv::Mat_<double>(4,1) << p.x, p.y, p.z, 1.0);
    cv::Mat out = T * p4;  // 4x4 * 4x1
    return cv::Point3d(out.at<double>(0,0), out.at<double>(1,0), out.at<double>(2,0));
}

static inline cv::Point3d compute_rotated_C(double roll, double pitch,
                                            const cv::Point3d& C0, double l_2)
{
    cv::Mat T_roll  = make_T_roll_x(roll);
    cv::Mat T_trans = make_T_trans_z(-l_2);
    cv::Mat T_pitch = make_T_pitch_y(pitch);
    cv::Mat T_trans2= make_T_trans_z(+l_2);

    cv::Mat T_total = T_trans2 * T_pitch * T_trans * T_roll;
    return apply_T(T_total, C0);
}

static inline void compute_theta(const cv::Point3d& C1, const cv::Point3d& C2,
                                 const cv::Point3d& A1, const cv::Point3d& A2,
                                 double l_rod1, double l_rod2, double l_bar,
                                 double& theta1, double& theta2)
{
    double dy1 = C1.y - A1.y;
    double dz1 = C1.z - A1.z;
    double dy2 = C2.y - A2.y;
    double dz2 = C2.z - A2.z;

    double norm_CA1 = cv::norm(C1 - A1);
    double norm_CA2 = cv::norm(C2 - A2);

    double c1 = (l_rod1*l_rod1 - l_bar*l_bar - norm_CA1*norm_CA1) / (-2.0 * l_bar);
    double c2 = (l_rod2*l_rod2 - l_bar*l_bar - norm_CA2*norm_CA2) / ( 2.0 * l_bar);

    double denom1 = std::sqrt(dy1*dy1 + dz1*dz1);
    double denom2 = std::sqrt(dy2*dy2 + dz2*dz2);
    if (denom1 == 0.0) throw std::runtime_error("denom1 == 0");
    if (denom2 == 0.0) throw std::runtime_error("denom2 == 0");

    double arg1 = c1 / denom1;
    double arg2 = c2 / denom2;

    arg1 = std::max(-1.0, std::min(1.0, arg1));
    arg2 = std::max(-1.0, std::min(1.0, arg2));

    double phi1 = std::atan2(dy1, dz1);
    double phi2 = std::atan2(dy2, dz2);

    double asin1 = std::asin(arg1);
    double asin2 = std::asin(arg2);

    double t1a = asin1 - phi1;
    double t1b = (CV_PI - asin1) - phi1;

    double t2a = asin2 - phi2;
    double t2b = (CV_PI - asin2) - phi2;

    theta1 = select_valid_angle(t1a, t1b);
    theta2 = select_valid_angle(t2a, t2b);
}
std::string getCurrentTimeWithMicroseconds();
std::vector<float> calculate_ankle_motor_joint(float AnklePitch, float AnkleRoll, int L)
{
    const double l_axis   = 0.03685;             // A点轴长
    const double l_axis_C = 0.033;               // C点轴长
    const double l_bar    = 0.0225;              // 杆长
    const double l_bar_c  = 0.0185;              // 杆长
    const double l_rod1   = L ? 0.2105 : 0.1455;              // 推杆1长度
    const double l_rod2   = L ? 0.1455 : 0.2105;              // 推杆2长度
    const double l_2      = 0.0;                 // 踝关节长度
    const double lz       = 0.01;                // 控制面和十字中心的高度

    const cv::Point3d C1_0(-l_axis_C,  l_bar_c, -lz);
    const cv::Point3d C2_0(-l_axis_C, -l_bar_c, -lz);

    const cv::Point3d A1_0(-l_axis, 0.0, L ? 0.200 : 0.135);
    const cv::Point3d A2_0(-l_axis, 0.0, L ? 0.135 : 0.200);

    const double roll  = (static_cast<double>(L ? AnkleRoll : AnkleRoll));
    const double pitch = (static_cast<double>(L ? AnklePitch : AnklePitch));

    const cv::Point3d C1 = compute_rotated_C(roll, pitch, C1_0, l_2);
    const cv::Point3d C2 = compute_rotated_C(roll, pitch, C2_0, l_2);

    double theta1 = 0.0, theta2 = 0.0;
    compute_theta(C1, C2, A1_0, A2_0, l_rod1, l_rod2, l_bar, theta1, theta2);
#if 1
    {
        std::cout << "[" << getCurrentTimeWithMicroseconds() << "] "
                  << "calculate_ankle_motor_joint: " << (L ? "L " : "R ")
                  << std::setw(10) << AnklePitch * 180.0f / 3.1415926f << " "
                  << std::setw(10) << AnkleRoll * 180.0f / 3.1415926f << " "
                  << std::setw(10) << theta1 * 180.0 / 3.1415926 << " "
                  << std::setw(10) << theta2 * 180.0 / 3.1415926 << " "
                  << std::endl;
    }

#endif
    if(L)
    {
        return { static_cast<float>(theta1),
                 static_cast<float>(theta2) };
    } else
    {
        return { static_cast<float>(theta2),
                 static_cast<float>(theta1) };
    }
}
