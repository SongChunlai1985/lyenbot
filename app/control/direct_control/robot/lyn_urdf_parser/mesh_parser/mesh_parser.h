#ifndef LYN_MESH_PARSER_H
#define LYN_MESH_PARSER_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <lyn_meta.h>

class mesh_parser
{
private:
    lyn_log logger;
public:
    mesh_parser();
    int run(std::string filename);
    int work(lyn_info &info);
    cv::Mat loadBinarySTL(std::string &filename, cv::Point3f visual_material_color);
    cv::Mat load_BinarySTL_box(std::string &filename, cv::Point3f visual_material_color);
};
#endif // LYN_MESH_PARSER_H
