#ifndef LYN_URDF_PARSER_H
#define LYN_URDF_PARSER_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>
#include <mesh_parser.h>
#include <components.h>

static std::map<int, std::string> jointTypeString =
{
    {urdf::Joint::REVOLUTE, "旋转关节 (Revolute)"},
    {urdf::Joint::CONTINUOUS, "连续关节 (Continuous)"},
    {urdf::Joint::PRISMATIC, "移动关节 (Prismatic)"},
    {urdf::Joint::FIXED, "固定关节 (Fixed)"},
    {urdf::Joint::FLOATING, "浮动关节 (Floating)"},
    {urdf::Joint::PLANAR, "平面关节 (Planar)"},
    {urdf::Joint::PLANAR, "未知类型 (Unknown)"},
};

class lyn_urdf_parser
{
private:
    std::string jointTypeToString(int type);
    cv::Mat loadGeometry(const std::string &type, const urdf::GeometrySharedPtr &geometry,
                                    cv::Point3f visual_material_color);
public:
    lyn_urdf_parser();
    components parser(std::string urdf_file);
    int work(lyn_info &info);
    std::shared_ptr<urdf::ModelInterface> robot_model;
    std::thread thread;
    lyn_log logger;
    mesh_parser Mesh_parser;
    std::string get_base_link(const std::shared_ptr<urdf::ModelInterface> &robot_model);

};
#endif // LYN_URDF_PARSER_H
