#include "lyn_meta.h"

meta::meta()
{
    position = cv::Point3d(0, 0, 0);
    point_x = cv::Point3d(1, 0, 0);
    point_y = cv::Point3d(0, 1, 0);
    point_z = cv::Point3d(0, 0, 1);
    attitude_angle = cv::Point3d(0, 0, 0);
    matrix = cv::Mat::eye(4, 4, CV_64FC1);
}

void meta::run()
{
}

meta meta::operator + (cv::Point3d scalar)
{
    meta result;
    result.position = position + scalar;
    result.point_x = point_x + scalar;
    result.point_y = point_y + scalar;
    result.point_z = point_z + scalar;
    result.matrix = get_translation(scalar) * matrix;
    return result;
}

meta meta::operator - (cv::Point3d scalar)
{
    meta result;
    result.position = position - scalar;
    result.point_x = point_x - scalar;
    result.point_y = point_y - scalar;
    result.point_z = point_z - scalar;
    result.matrix = get_translation(-scalar) * matrix;
    return result;
}

int meta::around_line(cv::Point3d pointa, cv::Point3d pointb, double angle_rad)
{
    position = point_around_line(position, pointa, pointb, angle_rad);
    point_x = point_around_line(point_x, pointa, pointb, angle_rad);
    point_y = point_around_line(point_y, pointa, pointb, angle_rad);
    point_z = point_around_line(point_z, pointa, pointb, angle_rad);
#if 0
    std::cout << "runrotatePointAroundLine: " << point_position << ", "
              << point_x << ", "
              << point_y << ", "
              << point_z << ", "
              << point_on_line << ", "
              << line_direction << ", "
              << angle_rad << ", "
              << std::endl;
#endif
    return 0;
}

cv::Mat meta::get_transformation_with_attitude_angle()
{
    meta m;
    m.position = position;
    m.point_x = point_x;
    m.point_y = point_y;
    m.point_z = point_z;
    m.attitude_angle = attitude_angle;
    cv::Mat mat = get_rotate_matrix(attitude_angle, m);
#if 0
    std::cout << __FUNCTION__ << " position " << position << std::endl
              << " point_x " << point_x << std::endl
              << " point_y " << point_y << std::endl
              << " point_z " << point_z << std::endl
              << " attitude_angle " << attitude_angle << std::endl
              << " matrix \n" << mat << std::endl;
#endif
    matrix = get_translation(position) * mat;
    return matrix;
}

void meta::expand()
{
    meta m;
    m.position = position;
    m.point_x = point_x;
    m.point_y = point_y;
    m.point_z = point_z;
    m.attitude_angle = attitude_angle;
    matrix = get_rotate_matrix(attitude_angle, m);
    meta n = matrix * m;
    position = n.position;
    point_x = n.point_x;
    point_y = n.point_y;
    point_z = n.point_z;

}

meta meta::clone()
{
    meta m;
    m.position = position;
    m.point_x = point_x;
    m.point_y = point_y;
    m.point_z = point_z;
    m.attitude_angle = attitude_angle;
    m.matrix = matrix.clone();
    return m;
}

cv::Point3d unit_vector(cv::Point3d Initial_Point, cv::Point3d Terminal_Point)
{
    cv::Point3d vector = Terminal_Point - Initial_Point;
    double lenth = cv::norm(vector);
    return vector / lenth;
}

cv::Mat matrix_rotate_line(cv::Point3d line_direction_normalize, double angle_rad)
{
    double
            x = line_direction_normalize.x,                                                            //必须是单位向量
            y = line_direction_normalize.y,
            z = line_direction_normalize.z,
            r = angle_rad,
            c = cos(r),
            sd = sin(r),
            i = 1 - c,
            xi = x *  i, yi = y *  i, zi = z *  i,
            xx = x * xi, yy = y * yi, zz = z * zi,
            xy = y * xi, yz = z * yi, zx = x * zi,
            xs = x * sd, ys = y * sd, zs = z * sd;
    cv::Mat Matrix = cv::Mat::eye(4, 4, CV_64FC1);
    Matrix.at<double>(0, 0) = xx +  c; Matrix.at<double>(0, 1) = xy - zs; Matrix.at<double>(0, 2) = zx + ys; Matrix.at<double>(0, 3) = 0.0;
    Matrix.at<double>(1, 0) = xy + zs; Matrix.at<double>(1, 1) = yy +  c; Matrix.at<double>(1, 2) = yz - xs; Matrix.at<double>(1, 3) = 0.0;
    Matrix.at<double>(2, 0) = zx - ys; Matrix.at<double>(2, 1) = yz + xs; Matrix.at<double>(2, 2) = zz +  c; Matrix.at<double>(2, 3) = 0.0;
    Matrix.at<double>(3, 0) =     0.0; Matrix.at<double>(3, 1) =     0.0; Matrix.at<double>(3, 2) =     0.0; Matrix.at<double>(3, 3) = 1.0;
    return Matrix;
}


cv::Mat matrix_rotate_line(cv::Point3d pointa, cv::Point3d pointb, double angle_rad)
{
    cv::Point3d line = pointb - pointa;
    cv::Point3d line_direction_normalize = line / cv::norm(line);
    return matrix_rotate_line(line_direction_normalize, angle_rad);
}

cv::Mat get_translation(cv::Point3d translation)
{
    cv::Mat Matrix = cv::Mat::eye(4, 4, CV_64FC1);
    Matrix.at<double>(0, 0) = 1.0; Matrix.at<double>(0, 1) = 0.0; Matrix.at<double>(0, 2) = 0.0; Matrix.at<double>(0, 3) = translation.x;
    Matrix.at<double>(1, 0) = 0.0; Matrix.at<double>(1, 1) = 1.0; Matrix.at<double>(1, 2) = 0.0; Matrix.at<double>(1, 3) = translation.y;
    Matrix.at<double>(2, 0) = 0.0; Matrix.at<double>(2, 1) = 0.0; Matrix.at<double>(2, 2) = 1.0; Matrix.at<double>(2, 3) = translation.z;
    Matrix.at<double>(3, 0) = 0.0; Matrix.at<double>(3, 1) = 0.0; Matrix.at<double>(3, 2) = 0.0; Matrix.at<double>(3, 3) = 1.0;
    return Matrix;
}

cv::Mat get_matrix_rotate_line(cv::Point3d pointa, cv::Point3d pointb, double angle_rad)
{
    return get_translation(pointa) * matrix_rotate_line(pointa, pointb, angle_rad) * get_translation(-pointa);
}

cv::Mat get_rotate_matrix(cv::Point3d angular, meta coordinate_system, Rotate_Sequence sequence)
{
    cv::Mat translation = get_translation(coordinate_system.position);
    cv::Mat rotate_x = matrix_rotate_line(coordinate_system.position, coordinate_system.point_x, angular.x);
    cv::Mat rotate_y = matrix_rotate_line(coordinate_system.position, coordinate_system.point_y, angular.y);
    cv::Mat rotate_z = matrix_rotate_line(coordinate_system.position, coordinate_system.point_z, angular.z);
    cv::Mat translation_r = get_translation(-coordinate_system.position);
    switch (sequence)
    {
    case Rotate_Sequence_XYZ:
    {
        cv::Mat rotate(rotate_z * rotate_y * rotate_x);                                            // 显式操作避免矩阵乘法内存泄漏
        return  translation * rotate * translation_r;
    }
    case Rotate_Sequence_XZY:
    {
        cv::Mat rotate(rotate_y * rotate_z * rotate_x);
        return  translation * rotate * translation_r;
    }
    case Rotate_Sequence_YXZ:
    {
        cv::Mat rotate = rotate_z * rotate_x * rotate_y;
        return  translation * rotate * translation_r;
    }
    case Rotate_Sequence_YZX:
    {
        cv::Mat rotate = rotate_x * rotate_z * rotate_y;
        return  translation * rotate * translation_r;
    }
    case Rotate_Sequence_ZXY:
    {
        cv::Mat rotate = rotate_y * rotate_x * rotate_z;
        return  translation * rotate * translation_r;
    }
    case Rotate_Sequence_ZYX:
    {
        cv::Mat rotate = rotate_x * rotate_y * rotate_z;
        return  translation * rotate * translation_r;
    }
    }
    return cv::Mat::eye(4, 4, CV_64FC1);
}

cv::Mat to_Mat4X1(cv::Point3d point)
{
    cv::Mat Mat4x1(4, 1, CV_64FC1, cv::Scalar(0));
    Mat4x1.at<double>(0, 0) = point.x;
    Mat4x1.at<double>(1, 0) = point.y;
    Mat4x1.at<double>(2, 0) = point.z;
    Mat4x1.at<double>(3, 0) = 1.0;
    return Mat4x1;
}

cv::Point3d from_Mat4x1(cv::Mat Mat)
{
    return cv::Point3d( Mat.at<double>(0, 0), Mat.at<double>(1, 0), Mat.at<double>(2, 0));
}

cv::Point3d operator *(cv::Mat mat4x4, cv::Point3d point)
{
    return from_Mat4x1(mat4x4 * to_Mat4X1(point));
}

cv::Point3d point_around_line(cv::Point3d point_to_rotate, cv::Point3d pointa, cv::Point3d pointb, double angle_rad)
{
    return get_matrix_rotate_line(pointa, pointb, angle_rad) * point_to_rotate;
}

meta operator *(cv::Mat mat4x4, meta Meta)
{
    meta result;
    result.position = mat4x4 * Meta.position;
    result.point_x  = mat4x4 * Meta.point_x;
    result.point_y  = mat4x4 * Meta.point_y;
    result.point_z = mat4x4 * Meta.point_z;
    result.matrix = mat4x4 * Meta.matrix;
    return result;
}

meta operator *(meta Meta_parent, meta Meta)
{
    return Meta_parent.get_transformation_with_attitude_angle() * Meta;
}


cv::Mat get_scale_matrix(double scale)
{
    cv::Mat Matrix = cv::Mat(4, 4, CV_64FC1, cv::Scalar(0));
    Matrix.at<double>(0, 0) = scale; Matrix.at<double>(0, 1) =   0.0; Matrix.at<double>(0, 2) =   0.0; Matrix.at<double>(0, 3) = 0.0;     //(col, row) (y, x)
    Matrix.at<double>(1, 0) =   0.0; Matrix.at<double>(1, 1) = scale; Matrix.at<double>(1, 2) =   0.0; Matrix.at<double>(1, 3) = 0.0;
    Matrix.at<double>(2, 0) =   0.0; Matrix.at<double>(2, 1) =   0.0; Matrix.at<double>(2, 2) = scale; Matrix.at<double>(2, 3) = 0.0;
    Matrix.at<double>(3, 0) =   0.0; Matrix.at<double>(3, 1) =   0.0; Matrix.at<double>(3, 2) =   0.0; Matrix.at<double>(3, 3) = 1.0;
    return Matrix;
}

meta operator * (meta Meta, double scale)
{
    return  get_scale_matrix(scale) * Meta;
}

gte::Matrix<4, 4, double> get_gte_matrix(double m00, double m10, double m20, double m30,
                                         double m01, double m11, double m21, double m31,
                                         double m02, double m12, double m22, double m32,
                                         double m03, double m13, double m23, double m33)
{
    gte::Matrix<4, 4, double> U;
    U.SetRow(0, gte::Vector<4, double>{m00, m10, m20, m30});
    U.SetRow(1, gte::Vector<4, double>{m01, m11, m21, m31});
    U.SetRow(2, gte::Vector<4, double>{m02, m12, m22, m32});
    U.SetRow(3, gte::Vector<4, double>{m03, m13, m23, m33});
    return U;
}

gte::Matrix<4, 4, double> get_gte_matrix(cv::Point3d X, cv::Point3d Y, cv::Point3d Z, cv::Point3d M,
                                         double m03, double m13, double m23, double m33)
{
    return (get_gte_matrix(X.x, Y.x, Z.x, M.x,
                           X.y, Y.y, Z.y, M.y,
                           X.z, Y.z, Z.z, M.z,
                           m03, m13, m23, m33));
}

gte::ConvertCoordinates<4, double> make_convert(cv::Point3d a0, cv::Point3d b0, cv::Point3d c0, cv::Point3d m0,
                                                cv::Point3d a1, cv::Point3d b1, cv::Point3d c1, cv::Point3d m1,
                                                bool vectorOnRightU, bool vectorOnRightV)
{
    gte::Matrix<4, 4, double> U, V;
    gte::ConvertCoordinates<4, double> convert;
    U = get_gte_matrix(a1, b1, c1, m1);
    V = get_gte_matrix(a0, b0, c0, m0);
    convert(U, vectorOnRightU, V, vectorOnRightV);
    return convert;
}

cv::Mat operator >> (meta src, meta dst)                                                           // 坐标左乘变回原坐标系
{
    gte::ConvertCoordinates<4, double> convert = make_convert(src.point_x - src.position,
                                                              src.point_y - src.position,
                                                              src.point_z - src.position,
                                                              src.position - src.position,
                                                              dst.point_x - dst.position,
                                                              dst.point_y - dst.position,
                                                              dst.point_z - dst.position,
                                                              dst.position - dst.position);
    gte::Matrix<4, 4, double> gte_matrix = get_gte_matrix(cv::Point3d(1, 0, 0), cv::Point3d(0, 1, 0), cv::Point3d(0, 0, 1),
                                                       src.position - dst.position);
    gte::Matrix<4, 4, double> gteMat = convert.GetC() * gte_matrix;

    cv::Mat ret = cv::Mat(4, 4, CV_64F, (void*)&gteMat(0, 0)).clone();                             // clone深拷贝避免析构异常
    return ret;
}

cv::Mat operator << (meta src, meta dst)                                                           // 坐标左乘变至目标坐标系
{
    gte::ConvertCoordinates<4, double> convert = make_convert(src.point_x - src.position,
                                                              src.point_y - src.position,
                                                              src.point_z - src.position,
                                                              src.position - src.position,
                                                              dst.point_x - dst.position,
                                                              dst.point_y - dst.position,
                                                              dst.point_z - dst.position,
                                                              dst.position - dst.position);
    gte::Matrix<4, 4, double> gte_matrix = get_gte_matrix(cv::Point3d(1, 0, 0), cv::Point3d(0, 1, 0), cv::Point3d(0, 0, 1),
                                                      src.position - dst.position);
    gte::Matrix<4, 4, double> gteMat = gte_matrix * convert.GetC();
    cv::Mat ret = cv::Mat(4, 4, CV_64F, (void*)&gteMat(0, 0)).clone();
    return ret;
}

cv::Mat createReflectionMatrix()                                                                   // 变换到左手坐标系
{
    cv::Mat matrix = cv::Mat::eye(4, 4, CV_64F);                                                   // 创建4x4单位矩阵
    matrix.at<double>(0, 0) = -1.0;                                                                // 修改X轴缩放因子
    return matrix;
}

cv::Mat quaternion2mat(double q0, double q1, double q2, double q3)                                 // 这个矩阵无法使用
{
    cv::Mat result(4, 4, CV_64F);

    double q0q0 = q0 * q0;
    double q1q1 = q1 * q1;
    double q2q2 = q2 * q2;
    double q3q3 = q3 * q3;

    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q3 = q2 * q3;

    result.at<double >(0, 0) = q0q0 + q1q1 - q2q2 - q3q3;
    result.at<double >(1, 0) = 2 * (q1q2 - q0q3);
    result.at<double >(2, 0) = 2 * (q1q3 + q0q2);
    result.at<double >(3, 0) = 0;

    result.at<double >(0, 1) = 2 * (q1q3 - q0q2);
    result.at<double >(1, 1) = 2 * (q2q3 + q0q1);
    result.at<double >(2, 1) = q0q0 - q1q1 - q2q2 + q3q3;
    result.at<double >(3, 1) = 0;

    result.at<double >(0, 2) = 2 * (q1q2 + q0q3);
    result.at<double >(1, 2) = q0q0 - q1q1 + q2q2 - q3q3;
    result.at<double >(2, 2) = 2 * (q2q3 - q0q1);
    result.at<double >(3, 2) = 0;

    result.at<double >(0, 3) = 0;
    result.at<double >(1, 3) = 0;
    result.at<double >(2, 3) = 0;
    result.at<double >(3, 3) = 1;
    return  result;
}

int draw_line(cv::Point3f pointA, cv::Point3f pointB,
              cv::Point3f colorA, cv::Point3f colorB,
              cv::Mat &Points)
{
    Points.push_back(pointA);
    Points.push_back(colorA);
    Points.push_back(pointB);
    Points.push_back(colorB);
    return 0;
}

int draw_meta(meta Meta, cv::Mat &Points, double length)
{
    draw_line(Meta.position, Meta.position + (Meta.point_x - Meta.position) * length,
              cv::Point3f(1.0f, 0.0f, 0.0f), cv::Point3f(1.0f, 0.0f, 0.0f), Points);
    draw_line(Meta.position, Meta.position + (Meta.point_y - Meta.position) * length,
              cv::Point3f(0.0f, 1.0f, 0.0f), cv::Point3f(0.0f, 1.0f, 0.0f), Points);
    draw_line(Meta.position, Meta.position + (Meta.point_z - Meta.position) * length,
              cv::Point3f(0.0f, 0.0f, 1.0f), cv::Point3f(0.0f, 0.0f, 1.0f), Points);
    return 0;
}

int draw_line_chart(std::deque<cv::Point3d> &data, cv::Mat &Points, meta pose)
{
    cv::Point3d last_point = data[0] * 100 + pose.position;
    for (uint i = 1; i < data.size(); ++i)
    {
        cv::Point3d curr_point = data[i] * 100 + pose.position + cv::Point3d(i * 10, 0, 0);
        draw_line(last_point, curr_point,
                  cv::Point3f(0.0f, 1.0f, 0.0f), cv::Point3f(0.0f, 0.5f, 0.0f), Points);
        last_point = curr_point;
    }
    return 0;
}
