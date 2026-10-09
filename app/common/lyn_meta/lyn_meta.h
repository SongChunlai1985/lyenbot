#ifndef LYN_META_H
#define LYN_META_H
#include <opencv2/opencv.hpp>
#include <Mathematics/GteConvertCoordinates.h>
#define LYN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L
#define ANGLE * double(LYN_PI) / 180.0
class meta                                                                                         // 一个最小单元，含位置和姿态角，也可以是4x4矩阵，在三维空间中具有唯一性，重叠元需要增加维度
{
public:
    meta();
    void run();

    cv::Point3d position;
    cv::Point3d attitude_angle;

    cv::Mat matrix;                                                                                // cv::Mat 本身不具备线程安全性

    cv::Point3d point_x;
    cv::Point3d point_y;
    cv::Point3d point_z;

    cv::Point3d speed;

    meta operator + (cv::Point3d scalar);
    meta operator - (cv::Point3d scalar) ;

    int around_line(cv::Point3d pointa, cv::Point3d pointb, double angle_rad);                      // 用于驾驶摄像机
    cv::Mat get_transformation_with_attitude_angle();                                               // 存在attitude_angle的情况下，只能获取旋转平移矩阵
    void expand();
    cv::Mat get_scale_matrix(double scale);
    meta clone();
};

enum Rotate_Sequence
{
    Rotate_Sequence_XYZ,
    Rotate_Sequence_XZY,
    Rotate_Sequence_YXZ,
    Rotate_Sequence_YZX,
    Rotate_Sequence_ZXY,
    Rotate_Sequence_ZYX,
};

cv::Point3d unit_vector(cv::Point3d Initial_Point, cv::Point3d Terminal_Point);
cv::Mat to_Mat4X1(cv::Point3d point);
cv::Point3d from_Mat4x1(cv::Mat Mat);
cv::Mat get_matrix_rotate_line(cv::Point3d pointa,
                               cv::Point3d pointb,
                               double angle_rad);
cv::Point3d point_around_line(cv::Point3d point_to_rotate,
                            cv::Point3d pointa,
                            cv::Point3d pointb,
                            double angle_rad);
cv::Mat get_rotate_matrix(cv::Point3d angular,
                          meta coordinate_system,
                          Rotate_Sequence rotate_sequence = Rotate_Sequence_XYZ);
cv::Mat get_translation( cv::Point3d translation);

gte::Matrix<4, 4, double> get_gte_matrix(double m00, double m10, double m20, double m30,
                                         double m01, double m11, double m21, double m31,
                                         double m02, double m12, double m22, double m32,
                                         double m03, double m13, double m23, double m33);

gte::Matrix<4, 4, double> get_gte_matrix(cv::Point3d X, cv::Point3d Y, cv::Point3d Z, cv::Point3d M,
                                         double m03 = 0.0, double m13 = 0.0, double m23 = 0.0, double m33 = 1.0);

gte::ConvertCoordinates<4, double> make_convert(cv::Point3d a0, cv::Point3d b0, cv::Point3d c0, cv::Point3d m0,
                                                cv::Point3d a1, cv::Point3d b1, cv::Point3d c1, cv::Point3d m1,
                                                bool vectorOnRightU = false, bool vectorOnRightV = false);
cv::Mat operator << (meta src, meta dst);
cv::Mat operator >> (meta src, meta dst);
meta operator * (cv::Mat mat4x4, meta Meta);
meta operator * (meta Meta_parent, meta Meta);                                                     // 元的矩阵形式
cv::Point3d operator *(cv::Mat mat4x4, cv::Point3d point);
meta operator * (meta Meta, double scale);

cv::Mat createReflectionMatrix();
cv::Mat quaternion2mat(double q0, double q1, double q2, double q3);
cv::Mat matrix_rotate_line(cv::Point3d line_direction_normalize, double angle_rad);
cv::Mat matrix_rotate_line(cv::Point3d pointa, cv::Point3d pointb, double angle_rad);

int draw_line(cv::Point3f pointA, cv::Point3f pointB,
              cv::Point3f colorA, cv::Point3f colorB, cv::Mat &Points);

int draw_meta(meta Meta, cv::Mat &Points, double length = 1.0);
int draw_line_chart(std::deque<cv::Point3d> &data, cv::Mat &Points, meta pose);
#endif // LYN_META_H
