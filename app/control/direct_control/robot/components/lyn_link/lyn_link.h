#ifndef LYN_LINK_H
#define LYN_LINK_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <lyn_meta.h>
class lyn_link
{
public:
    lyn_link();
    std::string name;
    meta Meta;
    meta visual_Meta;
    cv::Mat visual_vertexData;
    meta collision_Meta;
    cv::Mat collision_vertexData;

    double mass = 0.0;

    double ixx = 0.0;
    double ixy = 0.0;
    double ixz = 0.0;
    double iyy = 0.0;
    double iyz = 0.0;
    double izz = 0.0;

    cv::Point3f visual_material_color;
};
#endif // LYN_LINK_H
