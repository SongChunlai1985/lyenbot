#include <mesh_parser.h>

mesh_parser::mesh_parser()
{
    logger.run("mesh_parser.log", false);
}

void replaceAll(std::string& str, const std::string& oldVal, const std::string& newVal)
{
    size_t pos = 0;
    while ((pos = str.find(oldVal, pos)) != std::string::npos)
    {
        str.replace(pos, oldVal.length(), newVal);
        pos += newVal.length();                                                                    // 跳过新替换的内容，避免重复替换
    }
}

cv::Mat mesh_parser::loadBinarySTL(std::string& filename, cv::Point3f visual_material_color)
{
    cv::Mat vertices;
    replaceAll(filename, "package://", "");
    std::ifstream file("/home/lyenbot/config/" + filename, std::ios::binary);

    if (!file)
    {
        vertices.push_back(cv::Point3f(0, 0, 0));
        std::cerr << "file " << filename << " not found." << std::endl;
        return vertices;
    }
    file.seekg(80);
    uint32_t numTriangles;
    file.read(reinterpret_cast<char*>(&numTriangles), 4);

    vertices.reserve(numTriangles * 9);
    cv::Point3f color(visual_material_color.x, visual_material_color.y, visual_material_color.z);
    for (uint32_t i = 0; i < numTriangles && file; ++i)
    {
        file.seekg(12, std::ios::cur);                                                             // Skip normal
//        float triangle[9];
//        file.read(reinterpret_cast<char*>(triangle), 36);                                          // Read vertices
//        vertices.insert(vertices.end(), triangle, triangle + 9);
        float triangle[3];
        for (int j = 0; j < 3; ++j)
        {
            file.read(reinterpret_cast<char*>(triangle), 12);                                      // Read verticeA
            vertices.push_back(cv::Point3f(triangle[0], triangle[1], triangle[2]) * 1000);         // 单位由米转毫米
            vertices.push_back(color);
        }
        file.seekg(2, std::ios::cur);                                                              // Skip attributes
    }

    return vertices;
}

cv::Mat mesh_parser::load_BinarySTL_box(std::string &filename, cv::Point3f visual_material_color)
{
    cv::Mat vertices;
    replaceAll(filename, "package://", "");
    std::ifstream file(filename, std::ios::binary);

    if (!file)
    {
        vertices.push_back(cv::Point3f(0, 0, 0));
        std::cerr << "file " << filename << " not found." << std::endl;
        return vertices;
    }
    file.seekg(80);
    uint32_t numTriangles;
    file.read(reinterpret_cast<char*>(&numTriangles), 4);

    vertices.reserve(numTriangles * 9);
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
     cv::Point3f color(visual_material_color.x, visual_material_color.y, visual_material_color.z);
    for (uint32_t i = 0; i < numTriangles && file; ++i)
    {
        file.seekg(12, std::ios::cur);                                                             // Skip normal
//        float triangle[9];
//        file.read(reinterpret_cast<char*>(triangle), 36);                                          // Read vertices
//        vertices.insert(vertices.end(), triangle, triangle + 9);
        cv::Point3f vertice;
        float triangle[3] = {0.0f};
        for (int j = 0; j < 3; ++j)
        {
            file.read(reinterpret_cast<char*>(triangle), 12);                                      // Read verticeA
            minX = std::min(minX, triangle[0] * 1000);                                             // 米转毫米
            maxX = std::max(maxX, triangle[0] * 1000);
            minY = std::min(minY, triangle[1] * 1000);
            maxY = std::max(maxY, triangle[1] * 1000);
            minZ = std::min(minZ, triangle[2] * 1000);
            maxZ = std::max(maxZ, triangle[2] * 1000);
        }
        file.seekg(2, std::ios::cur);                                                              // Skip attributes
    }
    cv::Point3f vertices_box[2][2][2];
    vertices_box[0][0][0] = cv::Point3f(minX, minY, minZ);
    vertices_box[0][0][1] = cv::Point3f(minX, minY, maxZ);
    vertices_box[0][1][0] = cv::Point3f(minX, maxY, minZ);
    vertices_box[0][1][1] = cv::Point3f(minX, maxY, maxZ);
    vertices_box[1][0][0] = cv::Point3f(maxX, minY, minZ);
    vertices_box[1][0][1] = cv::Point3f(maxX, minY, maxZ);
    vertices_box[1][1][0] = cv::Point3f(maxX, maxY, minZ);
    vertices_box[1][1][1] = cv::Point3f(maxX, maxY, maxZ);

    cv::Mat triangels;
#if 0                                                                          // 立方体
    // 前面 (X最小面) - 2个三角形
    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color); // 左上前
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color); // 左上后

    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color); // 左上后
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color); // 左下后


    // 后面 (X最大面) - 2个三角形
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color); // 右下前
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color); // 右下后
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后

    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color); // 右下前
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color); // 右上前


    // 左面 (Y最小面) - 2个三角形
    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color); // 左下后
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color); // 右下后

    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color); // 右下后
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color); // 右下前


    // 右面 (Y最大面) - 2个三角形
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color); // 左上前
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color); // 右上前
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后

    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color); // 左上前
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color); // 左上后


    // 底面面 (Z最小面) - 2个三角形
    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color); // 右下前
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color); // 右上前

    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color); // 左下前
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color); // 右上前
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color); // 左上前


    // 顶面 (Z最大面) - 2个三角形
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color); // 左下后
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color); // 左上后
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后

    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color); // 左下后
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color); // 右上后
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color); // 右下后
#else                                                                          // 立方体12条梭
    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color);

    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color);

    triangels.push_back(vertices_box[0][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][0]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][0][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[0][1][1]);    triangels.push_back(color);
    triangels.push_back(vertices_box[1][1][1]);    triangels.push_back(color);
#endif
    logger << " color: " << color << " triangels: " << triangels << std::endl;
    return triangels;
}
int mesh_parser::run(std::string filename)
{
    logger << " mesh_parser: " << filename << std::endl;
    return 0;
}

int mesh_parser::work(lyn_info &info)
{
    return 0;
}


