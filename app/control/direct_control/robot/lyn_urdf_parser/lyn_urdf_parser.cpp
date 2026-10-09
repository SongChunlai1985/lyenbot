#include <lyn_urdf_parser.h>

lyn_urdf_parser::lyn_urdf_parser()
{
}

cv::Mat lyn_urdf_parser::loadGeometry(const std::string& type,
                                      const urdf::GeometrySharedPtr& geometry,
                                      cv::Point3f visual_material_color)                           // 打印几何模型信息（碰撞或可视化）
{
    cv::Mat vertexData;
    if (!geometry)
    {
        logger << "  " << type << "几何: 未定义" << std::endl;
        return vertexData;
    }

    logger << "  " << type << "几何:" << std::endl;
    switch (geometry->type)
    {
    case urdf::Geometry::BOX:
    {
        auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
        logger << "    类型: 长方体 (Box)" << std::endl;
        logger << "    尺寸 (xyz): "
               << box->dim.x << ", " << box->dim.y << ", " << box->dim.z << " m" << std::endl;
        break;
    }
    case urdf::Geometry::SPHERE:
    {
        auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
        logger << "    类型: 球体 (Sphere)" << std::endl;
        logger << "    半径: " << sphere->radius << " m" << std::endl;
        break;
    }
    case urdf::Geometry::CYLINDER:
    {
        auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
        logger << "    类型: 圆柱体 (Cylinder)" << std::endl;
        logger << "    半径: " << cylinder->radius << " m" << std::endl;
        logger << "    长度: " << cylinder->length << " m" << std::endl;
        break;
    }
    case urdf::Geometry::MESH:
    {
        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
        logger << "    类型: 网格模型 (Mesh)" << std::endl;
        logger << "    文件路径: " << mesh->filename << std::endl;
        logger << "    缩放因子 (xyz): "
               << mesh->scale.x << ", " << mesh->scale.y << ", " << mesh->scale.z << std::endl;
#define ROBOT_LOAD_TRIANGLES
#ifdef ROBOT_LOAD_TRIANGLES
        // std::string test = "package://lyn03/meshes/left_shoulder_yaw_link.STL";
        // vertexData = Mesh_parser.loadBinarySTL(test, visual_material_color);
        vertexData = Mesh_parser.loadBinarySTL(mesh->filename, visual_material_color);
#else
        vertexData = Mesh_parser.load_BinarySTL_box(mesh->filename, visual_material_color);
#endif
        break;
    }
    default:
        logger << "    类型: 未知" << std::endl;
    }
    return vertexData;
}

std::string lyn_urdf_parser::get_base_link(const std::shared_ptr<urdf::ModelInterface>& robot_model)
{
    if (!robot_model)
    {
        std::cerr << "无效的URDF模型" << std::endl;
        return "";
    }

    std::set<std::string> child_links;                                                             // 1. 收集所有作为"子连杆"的连杆（即有父连杆的连杆）
    for (const auto& joint_pair : robot_model->joints_)
    {
        const auto& joint = joint_pair.second;
        child_links.insert(joint->child_link_name);                                                // 子连杆必然有父连杆
    }

    for (const auto& link_pair : robot_model->links_)                                              // 2. 遍历所有连杆，找到不在child_links中的连杆（即无父连杆的根）
    {
        const std::string& link_name = link_pair.first;                                            // 若连杆不是任何关节的子连杆，则它是根
        if (child_links.find(link_name) == child_links.end()) { return link_name; }
    }
    std::cerr << "未找到根连杆（可能存在循环依赖）" << std::endl;                                                  // 3. 特殊情况：所有连杆都有父（可能存在循环，但URDF不允许）
    return "";
}

void findAllChildLinks(
    const std::string& current_link,
    const std::map<std::string, std::vector<std::string>>& parent_map,
    std::set<std::string>& child_links)                                                            // 递归查找所有子连杆（包括直接和间接子连杆）
{
    auto it = parent_map.find(current_link);                                                       // 查找当前连杆的直接子连杆
    if (it == parent_map.end()) {
        return;                                                                                    // 没有子连杆
    }
    for (const std::string& child : it->second)                                                    // 遍历所有直接子连杆
    {
        child_links.insert(child);                                                                 // 添加直接子连杆
    }
}

std::map<std::string, std::vector<std::string>> buildParentToChildrenMap(
    const std::shared_ptr<urdf::ModelInterface>& robot_model)                                      // 构建父到子的映射表
{
    std::map<std::string, std::vector<std::string>> parent_map;
    for (const auto& joint_pair : robot_model->joints_)
    {
        const auto& joint = joint_pair.second;
        std::string parent = joint->parent_link_name;
        std::string child = joint->child_link_name;
        parent_map[parent].push_back(child);
    }
    return parent_map;
}

std::set<std::string> getChildLinks(
    std::map<std::string, std::vector<std::string>> parent_map,
    const std::string& target_link)                                                                // 查找指定连杆的所有子连杆（返回包含所有子连杆名称的集合）
{
    std::set<std::string> child_links;                                                             // 递归查找所有子连杆
    findAllChildLinks(target_link, parent_map, child_links);
    return child_links;
}

void buildComponentHierarchy( std::map<std::string,
                              std::vector<std::string>> &parent_map,
                              std::map<std::string, components> &all_links,
                              components& current_component)                                       // 当前组件（递归的起点或父组件）
{
    std::string current_link_name = current_component.name;                                        // 1. 设置当前组件的名称（如果需要）
    if(parent_map.count(current_link_name) == 0) { return; }                                       // 没有子连杆，递归终止
    std::vector<std::string> child_link_names = parent_map[current_link_name];                     // 2. 查找当前连杆的直接子连杆
    for (std::string& child_link_name : child_link_names)                                          // 3. 为每个直接子连杆创建组件，并递归处理其子组件
    {
        if(all_links.count(child_link_name) == 0) { continue; }                                    // 跳过不存在的连杆
        components link_it = all_links[child_link_name];                                           // 检查子连杆是否存在于所有连杆数据中
        current_component.Components[child_link_name] = link_it;                                   // 将子连杆添加到当前组件的 Components 中
        buildComponentHierarchy(parent_map, all_links,                                             // 递归处理子连杆的子组件（深度优先）
                    current_component.Components[child_link_name]);                                // 传入子组件作为当前组件
    }
}

components lyn_urdf_parser::parser(std::string urdf_file)
{
    logger.run("urdf_parser.log", false);
    robot_model = urdf::parseURDFFile(urdf_file);

    logger << "My name: " << robot_model->getName() << std::endl;                                  // 获取所有关节（存储在 joints_ 成员中，是 std::map 类型）

    std::map<std::string, urdf::LinkSharedPtr> links = robot_model->links_;                        // 获取所有连杆（存储在 links_ 成员中，std::map 类型）

    std::map<std::string, components> all_links;
    for (std::pair<std::string, urdf::LinkSharedPtr> link_pair : links)                            // 枚举所有连杆并输出详细信息
    {
        components one_link;
        std::string link_name = link_pair.first;
        one_link.name = link_name;
        one_link.link.name = link_name;
        urdf::LinkSharedPtr link = link_pair.second;

        if (link->inertial)
        {
            one_link.link.ixx = link->inertial->ixx;
            one_link.link.iyy = link->inertial->iyy;
            one_link.link.izz = link->inertial->izz;
            one_link.link.ixy = link->inertial->ixy;
            one_link.link.ixz = link->inertial->ixz;
            one_link.link.iyz = link->inertial->iyz;

            meta Meta;
            Meta = meta() + cv::Point3d(link->inertial->origin.position.x,
                                        link->inertial->origin.position.y,
                                        link->inertial->origin.position.z) ;

            Meta.attitude_angle = cv::Point3d(link->inertial->origin.rotation.x,
                                              link->inertial->origin.rotation.y,
                                              link->inertial->origin.rotation.z) ;
            Meta.expand();
            one_link.link.Meta = Meta;
        }

        if (link->collision)                                                                       // 输出碰撞模型
        {
            one_link.link.collision_vertexData = loadGeometry("碰撞", link->collision->geometry,
                                                              cv::Point3f(0, 0, 1));
            meta collision_Meta;
            collision_Meta = meta() + cv::Point3d(link->collision->origin.position.x,
                                                  link->collision->origin.position.y,
                                                  link->collision->origin.position.z) ;

            collision_Meta.attitude_angle = cv::Point3d(link->collision->origin.rotation.x,
                                                        link->collision->origin.rotation.y,
                                                        link->collision->origin.rotation.z) ;
            collision_Meta.expand();
            one_link.link.collision_Meta = collision_Meta;
        }
        else { logger << "  碰撞几何: 未定义" << std::endl; }

        if (link->visual)                                                                          // 输出可视化模型
        {
            urdf::Color color = link->visual->material->color;                                     // 输出可视化颜色（若有）
            one_link.link.visual_material_color = cv::Point3f(color.r, color.g, color.b);
            one_link.link.visual_vertexData = loadGeometry("可视化", link->visual->geometry,
                                                           one_link.link.visual_material_color);
            meta visual_Meta;
            visual_Meta = meta() + cv::Point3d(link->visual->origin.position.x,
                                               link->visual->origin.position.y,
                                               link->visual->origin.position.z) ;

            visual_Meta.attitude_angle = cv::Point3d(link->visual->origin.rotation.x,
                                                     link->visual->origin.rotation.y,
                                                     link->visual->origin.rotation.z) ;
            visual_Meta.expand();
            one_link.link.visual_Meta = visual_Meta;
        } else { logger << "  可视化几何: 未定义" << std::endl; }

        all_links[link_name] = one_link;
    }

    const std::map<std::string, urdf::JointSharedPtr>& joints = robot_model->joints_;
#if 1
    logger << "===== 机器人连杆信息 =====" << std::endl
           << "连杆总数: " << links.size() << std::endl
           << "关节总数: " << joints.size() << std::endl
           << "-------------------------" << std::endl;
#endif
    for (std::pair<std::string, urdf::JointSharedPtr> joint_pair : joints)                         // 枚举所有关节
    {
        std::string joint_name = joint_pair.first;
        urdf::JointSharedPtr joint = joint_pair.second;

        lyn_joint Lyn_joint;
        Lyn_joint.name = joint_name;
        Lyn_joint.parent_link_name = joint->parent_link_name;
        Lyn_joint.child_link_name = joint->child_link_name;
        Lyn_joint.type = joint->type;
#if 0
        logger << "-------------------------" << std::endl
               << "关节名称: " << joint_name << std::endl
               << "父连杆: " << joint->parent_link_name << std::endl
               << "子连杆: " << joint->child_link_name << std::endl
               << "-------------------------" << std::endl;
#endif
        if(joint->dynamics)
        {
            Lyn_joint.dynamics_damping = joint->dynamics->damping;
            Lyn_joint.dynamics_friction = joint->dynamics->friction;
        }

        if (joint->limits)                                                                         // 输出关节限位（旋转/移动关节有此信息，固定关节无）
        {
            Lyn_joint.limits_lower = joint->limits->lower;
            Lyn_joint.limits_upper = joint->limits->upper;
            Lyn_joint.limits_effort = joint->limits->effort;
            Lyn_joint.limits_velocity = joint->limits->velocity;
        } else { logger << "限位信息: 无（固定关节或未定义）" << std::endl; }

        if (joint->safety)
        {
            Lyn_joint.soft_upper_limit = joint->safety->soft_upper_limit;
            Lyn_joint.soft_lower_limit = joint->safety->soft_lower_limit;
            Lyn_joint.k_position = joint->safety->k_position;
            Lyn_joint.k_velocity = joint->safety->k_velocity;
        }

        urdf::Pose origin = joint->parent_to_joint_origin_transform;                               // 获取关节原点坐标（相对于父连杆）

        meta link_joint = (meta()
                + cv::Point3d(origin.position.x * 1000, origin.position.y * 1000, origin.position.z * 1000));   //轴长100，位置转mm
        link_joint.attitude_angle = cv::Point3d(origin.rotation.x, origin.rotation.y, origin.rotation.z);
        link_joint.expand();

        Lyn_joint.Meta[env_components] = link_joint;
        Lyn_joint.Meta[env_components_axis].point_z
                = cv::Point3d(joint->axis.x, joint->axis.y, joint->axis.z);
        all_links[joint->child_link_name].joint = Lyn_joint;                                       // 固定基座时被驱动的连杆
    }

    std::string root_link_name = get_base_link(robot_model);
    std::map<std::string, std::vector<std::string>> parent_map = buildParentToChildrenMap(robot_model);
    components Components;
    Components.name = root_link_name;
    Components.link = all_links[root_link_name].link;
    Components.joint = all_links[root_link_name].joint;
    buildComponentHierarchy(parent_map, all_links, Components);
    return Components;
}

int lyn_urdf_parser::work(lyn_info &info)
{
    return 0;
}
