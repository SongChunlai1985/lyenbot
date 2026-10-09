#include "lyn_gl.h"

glm::vec3 lyn_gl::trans_xyz;
glm::vec3 lyn_gl::scale;
lyn_shader lyn_gl::shader;

meta lyn_gl::camera;                                                                               // 因为是绕自己的轴旋转，不是绕地面坐标系的轴旋转，所以相对地面坐标系会产生绕z轴的偏转。
cv::Point3d lyn_gl::camera_position;

static int mouse_button[3];

static double mouse_x_position;
static double mouse_y_position;

static double mouse_x_position_event;
static double mouse_y_position_event;

static double key_value[5];

static int key_press;

lyn_gl::lyn_gl()
{
    scale = glm::vec3(50.0f, 50.0f, 50.0f);
    camera = camera + cv::Point3d(1000, 0, -2300);                                                 // 摄像机为原点左移1000mm, 后移2300mm
    step["gl_init"].store(lyn_step_free);
    step["gl_main"].store(lyn_step_free);
    step["reconstruction_output"].store(lyn_step_free);
    step["robot_info_meshes"].store(lyn_step_free);
    step["info_joints_points"].store(lyn_step_free);
}

struct pt4d
{
    double x, y, z, a;
    pt4d(double x_, double y_, double z_, double a_)
    {
        x = x_;
        y = y_;
        z = z_;
        a = a_;
    }
};

cv::Mat translucent(cv::Mat img,pt4d tsl=pt4d(1,1,1,0.8))
{
    if(img.empty())return img;
    cv::Mat img1,img14[4];
    cv::cvtColor(img,img1,cv::COLOR_RGB2BGRA);
    cv::split(img1,img14);
    if(tsl.x<1.0)img14[0]=img14[0]*tsl.x;
    if(tsl.y<1.0)img14[1]=img14[1]*tsl.y;
    if(tsl.z<1.0)img14[2]=img14[2]*tsl.x;
    if(tsl.a<1.0)img14[3]=img14[3]*tsl.a;
    cv::merge(img14,4,img1);
    return img1;
}

GLuint gettexture(cv::Mat &img, GLuint idTexture = 0)                                              //制定纹理的函数
{
    if(!idTexture)glGenTextures(1, &idTexture);
    if(img.empty())return 0;
    if (idTexture){			                                                                       //加载纹理映射
        glBindTexture(GL_TEXTURE_2D, idTexture);
        /*gluBuild2DMipmaps(GL_TEXTURE_2D, 3, img.cols, img.rows,
                          GL_BGR, GL_UNSIGNED_BYTE, img.data);*/
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.cols, img.rows,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, img.data);
        glPixelStoref(GL_PACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    }
    return idTexture;
}

GLuint loadtexture(std::string fileName,pt4d tsl=pt4d(1,1,1,0.8)){
    cv::Mat img = translucent(cv::imread(fileName),tsl);
    return gettexture(img);
}

double lyn_gl::get_distance(cv::Point3f p)
{
    cv::Point3f a = camera_gl.position;

    float aspect = window_width / window_height;
    float fov = 75.0f;                                                                             // 粗略的FOV
    float scale = tan(fov * 0.5f * float(M_PI) / 180.0f);

    float x = (2.0f * float(mouse_x_position) / window_width - 1.0f) * aspect * scale;
    float y = (1.0f - 2.0f * float(mouse_y_position) / window_height) * scale * 0.56f;              // 0.56 粗略的系数

    glm::vec3 ray_dir_cam(x, y, -1.0f);

    glm::mat4 view = glm::lookAt(to_glm_vec3(camera_gl.position),
                                 to_glm_vec3(camera_gl.point_z),
                                 to_glm_vec3(camera_gl.point_y - camera_gl.position));
    glm::mat4 inverse_view = glm::inverse(view);
    glm::vec3 ray_dir_world = glm::mat3(inverse_view) * ray_dir_cam;
    ray_dir_world = glm::normalize(ray_dir_world);

    cv::Point3f b = a + to_cv_Point3f(ray_dir_world) * 10000.0f;
    cv::Point3f v0 = b - a;
    cv::Point3f v1 = (p - a).cross(v0);

    return cv::norm(v1) / cv::norm(v0);
}

void lyn_gl::keyCallback([[maybe_unused]] GLFWwindow* window, int key, [[maybe_unused]]int scancode,
int action, [[maybe_unused]] int mods)
{
    if(action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
        case GLFW_KEY_A: camera = camera + unit_vector(camera.position, camera.point_x) * scale.x; break;    // 左移
        case GLFW_KEY_D: camera = camera - unit_vector(camera.position, camera.point_x) * scale.x; break;    // 右移
        case GLFW_KEY_R: camera = camera + unit_vector(camera.position, camera.point_y) * scale.y; break;    // 上移
        case GLFW_KEY_F: camera = camera - unit_vector(camera.position, camera.point_y) * scale.y; break;    // 下移
        case GLFW_KEY_W: camera = camera + unit_vector(camera.position, camera.point_z) * scale.z; break;    // 前移
        case GLFW_KEY_S: camera = camera - unit_vector(camera.position, camera.point_z) * scale.z; break;    // 后移
        case GLFW_KEY_Q: camera.around_line(camera.position, camera.point_z, -0.03); break;                   // 横滚
        case GLFW_KEY_E: camera.around_line(camera.position, camera.point_z,  0.03); break;                   // 横滚
        case GLFW_KEY_T: scale *= 1.5f; break;                                                               // 放大
        case GLFW_KEY_G: scale /= 1.5f; break;                                                               // 缩小
        case GLFW_KEY_Y: key_value[1] += double(scale.x); break;
        case GLFW_KEY_H: key_value[1] -= double(scale.x); break;
        case GLFW_KEY_ESCAPE: abort(); break;
        case GLFW_KEY_B: key_value[3] = 1.0; break;
        }
        key_press = key;
#if 0
        std::cout << "Key Press: " << key << " \n "
                  << "camera.position = " << camera.position << " \n "
                  << "camera.point_x =        " << camera.point_x << " \n "
                  << "camera.point_y =        " << camera.point_y << " \n "
                  << "camera.point_z =        " << camera.point_z << " \n "
                  << std::endl;
#endif
    }
    return;
}

void lyn_gl::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    std::ignore = window;
    std::ignore = xoffset;
    camera = camera + unit_vector(camera.position, camera.point_z) * scale.z * yoffset;
    return;
}

void lyn_gl::mouse_callback([[maybe_unused]] GLFWwindow* window, double xpos, double ypos)          // 鼠标位置回调函数
{
    if(mouse_button[right])
    {
        camera.around_line(camera.position, camera.point_y, (xpos - mouse_x_position) * -0.003);
        camera.around_line(camera.position, camera.point_x, (ypos - mouse_y_position) *  0.003);
    }
    if(mouse_button[middle])
    {
        camera = camera + unit_vector(camera.position, camera.point_x) * scale.x
                * (xpos - mouse_x_position) * -0.03;                                           // 左右移
        camera = camera + unit_vector(camera.position, camera.point_y) * scale.y
                * (ypos - mouse_y_position) * -0.03 ;                                          // 上下移
    }
    mouse_x_position = xpos;
    mouse_y_position = ypos;
    return;
}

void lyn_gl::mouse_button_callback([[maybe_unused]] GLFWwindow* window, int button, int action, [[maybe_unused]] int mods)                   // 鼠标按键回调函数
{
    mouse_button[button] = action;
    //    std::cout << "button: " << button << ", " << "action: " << action << ", " << std::endl;
    mouse_x_position_event = mouse_x_position;
    mouse_y_position_event = mouse_y_position;
    return;
}

GLsizeiptr get_size(cv::Mat &mat)
{
    return uint(mat.rows * mat.cols * mat.channels()) * uint(mat.elemSize1());
}

glm::mat4 to_mat4(cv::Mat mat)
{
    glm::mat4 mat4 = glm::mat4(1.0f);
    mat4[0][0] = float(mat.at<double>(0, 0)); mat4[1][0] = float(mat.at<double>(0, 1)); mat4[2][0] = float(mat.at<double>(0, 2)); mat4[3][0] = float(mat.at<double>(0, 3));
    mat4[0][1] = float(mat.at<double>(1, 0)); mat4[1][1] = float(mat.at<double>(1, 1)); mat4[2][1] = float(mat.at<double>(1, 2)); mat4[3][1] = float(mat.at<double>(1, 3));
    mat4[0][2] = float(mat.at<double>(2, 0)); mat4[1][2] = float(mat.at<double>(2, 1)); mat4[2][2] = float(mat.at<double>(2, 2)); mat4[3][2] = float(mat.at<double>(2, 3));
    mat4[0][3] = float(mat.at<double>(3, 0)); mat4[1][3] = float(mat.at<double>(3, 1)); mat4[2][3] = float(mat.at<double>(3, 2)); mat4[3][3] = float(mat.at<double>(3, 3));
    return mat4;
}

void lyn_gl::thread_reconstruction_data()
{
    while (running)
    {
        lyn_Infos infos;
        {
            std::unique_lock<std::mutex> lock(mutex["reconstruction_output"]);
            condition_variable["reconstruction_output"].wait
                    (lock, [=](){ return step["reconstruction_output"].load() == lyn_step_copying; });
            infos = std::move(input_public["reconstruction_output"]);
        }
        for (uint i = 0; i < infos.size(); ++i)
        {
            std::string name;
            if(std::holds_alternative<std::string>(infos[i]["name"]))
            { name = std::get<std::string>(infos[i]["name"]); }
            else { std::cerr << "std::get: wrong index for variant, name" << std::endl; }
            {
                if(name == "orbbec_point_cloud")
                {
                    if(std::holds_alternative<cv::Mat>(infos[i]["pointcloud_MatD_small_copy"]))
                    {
                        if(update[pointcloud_MatD_small_update] == 0)
                        {
                            cv::Mat ptr_pointcloud_MatD_small = std::move(std::get<cv::Mat>(infos[i]["pointcloud_MatD_small_copy"]));
                            pointcloud_MatD_small = std::move(ptr_pointcloud_MatD_small);
                            update[pointcloud_MatD_small_update] = 1;                      // if(argc > 1)cv::imshow("pointcloud_MatD_small", pointcloud_MatD_small);
                        }
                    } else { std::cerr << "std::get: wrong index for variant, pointcloud_MatD_small_copy" << std::endl; }

                    if(std::holds_alternative<cv::Mat>(infos[i]["colorRawMatD_copy"]))
                    {
                        if(update[colorRawMatD_update] == 0)
                        {
                            cv::Mat ptr_colorRawMatD = std::get<cv::Mat>(infos[i]["colorRawMatD_copy"]);
                            colorRawMatD_copy = std::move(ptr_colorRawMatD);
                            update[colorRawMatD_update] = 1;
                        }
                    } else { std::cerr << "std::get: wrong index for variant, colorRawMatD_copy" << std::endl; }
                }

                if(name == "mid360_point_cloud")
                {
                    if(std::holds_alternative<cv::Mat>(infos[i]["point_data_mat_xyzs_copy"]))
                    {
                        if(update[point_data_mat_xyzs_copy_update] == 0)
                        {
                            cv::Mat ptr_point_data_mat_xyzs_copy = std::move(std::get<cv::Mat>(infos[i]["point_data_mat_xyzs_copy"]));
                            point_data_mat_xyzs_copy = std::move(ptr_point_data_mat_xyzs_copy);
                            update[point_data_mat_xyzs_copy_update] = 1;
                        }
                    } else { std::cerr << "std::get: wrong index for variant, point_data_mat_xyzs_copy" << std::endl; }

                    if(std::holds_alternative<cv::Mat>(infos[i]["IMU_Points"]))
                    {
                        if(update[IMU_Points_update] == 0)
                        {
                            IMU_Points = std::move(std::get<cv::Mat>(infos[i]["IMU_Points"]));
                            update[IMU_Points_update] = 1;
                        }
                    } else { std::cerr << "std::get: wrong index for variant, IMU_Points" << std::endl; }
                }

                if(name == "h12_imu_data")
                {
                    if(std::holds_alternative<cv::Mat>(infos[i]["IMU_Points_h12"]))
                    {
                        if(update[IMU_Points_h12_update] == 0)
                        {
                            IMU_Points_h12 = std::move(std::get<cv::Mat>(infos[i]["IMU_Points_h12"]));
                            update[IMU_Points_h12_update] = 1;
                        }
                    } else { std::cerr << "std::get: wrong index for variant, IMU_Points_h12" << std::endl; }
                }
            }

            // logger << name << " step: " << SetpString[int(step[reconstruction_output])] << std::endl;
        }
        {
            std::unique_lock<std::mutex> lock(mutex["reconstruction_output"]);
            step["reconstruction_output"].store(lyn_step_copy_complete);
        }
        condition_variable["gl_main"].notify_one();
    }
}

void lyn_gl::thread_robot_meshes_data()
{
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["robot_info_meshes"]);
            condition_variable["robot_info_meshes"].wait
                    (lock, [=](){ return step["robot_info_meshes"].load() == lyn_step_copying; });
        }
        {
            std::unique_lock<std::mutex> lock(mutex["robot_info_meshes"]);
            step["robot_info_meshes"].store(lyn_step_copy_complete);
        }
        condition_variable["gl_init"].notify_one();
    }
}

void lyn_gl::thread_robot_joints()
{
    lyn_Infos Robot_info_joints_points;
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["robot_info_joints_points"]);
            condition_variable["robot_info_joints_points"].wait(lock, [=]()
            { return step["robot_info_joints_points"].load() == lyn_step_copying; });

            if(update[robot_info_matrix_update] == 1){ usleep(1000); continue; }
            Robot_info_joints_points = std::move(input_public["robot_info_joints_points"]);
        }

        lyn_Info info_joints_points = Robot_info_joints_points.front();
        if(std::holds_alternative<cv::Mat>(info_joints_points["robot_info_joints_points"]))
        {
            std::unique_lock<std::shared_mutex> exclusive_lock(render_mutex);
            Points = std::get<cv::Mat>(info_joints_points["robot_info_joints_points"]);
            exclusive_lock.unlock();
        }
        step["robot_info_joints_points"].store(lyn_step_copy_complete);
    }
}

void lyn_gl::thread_robot_info_matrixs()
{
    lyn_Infos Robot_info_matrixs;
    while (running)
    {
        {
            std::unique_lock<std::mutex> lock(mutex["robot_info_matrixs"]);
            condition_variable["robot_info_matrixs"].wait(lock, [=]()
            { return step["robot_info_matrixs"].load() == lyn_step_copying; });

            if(update[robot_info_matrix_update] == 1){ usleep(1000); continue; }
            Robot_info_matrixs = std::move(input_public["robot_info_matrixs"]);
            step["robot_info_matrixs"].store(lyn_step_copy_complete);
        }

        if(update[robot_info_matrix_update] == 0)
        {
            robot_matrixs.clear();
            for (uint i = 0; i < Robot_info_matrixs.size(); ++i)
            {
                if(std::holds_alternative<cv::Mat>(Robot_info_matrixs[i]["robot_info_matrix"]))
                {
                    cv::Mat robot_info_matrix
                            = std::get<cv::Mat>(Robot_info_matrixs[i]["robot_info_matrix"]);
                    robot_matrixs.push_back(std::move(robot_info_matrix));
                } else { std::cerr << "std::get: wrong index for variant, matrix" << std::endl; }
            }
            update[robot_info_matrix_update] = 1;
        }
    }
}

void lyn_gl::gl_start()
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);                                                 // 配置 GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 3);                                                               // 3重采样消除锯齿
    glEnable(GL_MULTISAMPLE);

    window = glfwCreateWindow(window_width, window_height, "Lyenbot", nullptr, nullptr);                         // 创建窗口
    if(!window) { std::cerr << "Failed to create GLFW window" << std::endl; glfwTerminate(); return; }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, mouse_callback);                                              // 鼠标移动
    glfwSetMouseButtonCallback(window, mouse_button_callback);                                     // 鼠标按键
    glfwSetScrollCallback(window, scroll_callback);                                                // 鼠标滚轮

    //    sk1 = loadtexture("sk1.jpeg", pt4d(0.9, 0.9, 0.9, 0.3));
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(glewInit() != GLEW_OK) { std::cerr << "Failed to initialize GLEW" << std::endl; return; }

    shader.createShader();
}

void lyn_gl::setup_VAO()
{
    point_data_mat_xyzs_color = cv::Mat(96, 500, CV_32FC3);
    point_data_mat_xyzs_color.setTo(cv::Scalar(1.0, 0.588, 0.196));

    glGenVertexArrays(1, &VAO[color_cloud]);
    glBindVertexArray(VAO[color_cloud]);                                                           // 选中一个VAO
    glGenBuffers(1, &VBO[color_cloud_position_buffer]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_cloud_position_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(pointcloud_MatD_small_copy), pointcloud_MatD_small_copy.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);           // 设置位置属性
    glEnableVertexAttribArray(attrib_0);                                                           // 开启位置属性
    glGenBuffers(1, &VBO[color_cloud_color_buffer]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_cloud_color_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(colorRawMatD_copy_copy), colorRawMatD_copy_copy.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_1);

    glGenVertexArrays(1, &VAO[point_cloud]);
    glBindVertexArray(VAO[point_cloud]);
    glGenBuffers(1, &VBO[point_cloud_position_buffer]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[point_cloud_position_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(point_data_mat_xyzs_copy_copy), point_data_mat_xyzs_copy_copy.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_0);
    glGenBuffers(1, &VBO[point_cloud_color_buffer]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[point_cloud_color_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(point_data_mat_xyzs_color), point_data_mat_xyzs_color.data, GL_STATIC_DRAW);
    glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_1);

    glGenVertexArrays(1, &VAO[color_lines]);
    glGenBuffers(1, &VBO[color_lines_buffer]);
    glBindVertexArray(VAO[color_lines]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_lines_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(IMU_Points_copy), IMU_Points_copy.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_0);
    glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(attrib_1);

    glGenVertexArrays(1, &VAO[color_lines_h12]);
    glGenBuffers(1, &VBO[color_lines_buffer_h12]);
    glBindVertexArray(VAO[color_lines_h12]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_lines_buffer_h12]);
    glBufferData(GL_ARRAY_BUFFER, get_size(IMU_Points_h12_copy), IMU_Points_h12_copy.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_0);
    glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(attrib_1);

    {
        std::unique_lock<std::mutex> lock(mutex["gl_init"]);
        condition_variable["gl_init"].wait(lock, [=](){ return step["robot_info_meshes"].load() == lyn_step_copy_complete; });
    }

    int i = 0;
    //    glBindTexture(GL_TEXTURE_2D, /*sk2*/ sk1);
    for (lyn_Info mesh : input_public["robot_info_meshes"])
    {
        for (auto mesh01 : mesh)
        {
            cv::Mat mesh01_mat;
            if(std::holds_alternative<cv::Mat>(mesh01.second))
            {
                mesh01_mat = std::get<cv::Mat>(mesh01.second);
                mesh01_mats[i] = mesh01_mat;
            }

            GLsizeiptr VBO_GLsizeiptr = get_size(mesh01_mats[i]);
            VBO_GLsizeiptrs[i] = VBO_GLsizeiptr;
            glGenVertexArrays(1, &VAO_MESHES[i]);
            glGenBuffers(1, &VBO_MESHES[i]);
            glBindVertexArray(VAO_MESHES[i]);
            glBindBuffer(GL_ARRAY_BUFFER, VBO_MESHES[i]);
            glBufferData(GL_ARRAY_BUFFER, VBO_GLsizeiptr, mesh01_mats[i].data, GL_STATIC_DRAW);
            glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(attrib_0);
            glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
            glEnableVertexAttribArray(attrib_1);
            logger << " i: " << i << " "
                   << "name: " << mesh01.first
                      //                       << " VAO_MESHES: " << VAO_MESHES[i] << " "
                      //                       << " VBO_MESHES: " << VBO_MESHES[i] << " "
                      //                       << " VBO_GLsizeiptr: " << VBO_GLsizeiptr << " "
                   << std::endl;
            i ++;
        }
    }

    glGenVertexArrays(1, &VAO[color_lines_robot_joints]);
    glGenBuffers(1, &VBO[color_lines_robot_joints_buffer]);
    glBindVertexArray(VAO[color_lines_robot_joints]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_lines_robot_joints_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(Points), Points.data, GL_STREAM_DRAW);
    glVertexAttribPointer(attrib_0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(attrib_0);
    glVertexAttribPointer(attrib_1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(attrib_1);
}

void lyn_gl::gl_update_data()
{
    if(update[pointcloud_MatD_small_update])
    {
        pointcloud_MatD_small_copy = std::move(pointcloud_MatD_small);
        update[pointcloud_MatD_small_update] = 0;
    }
    if(update[colorRawMatD_update])
    {
        colorRawMatD_copy_copy = std::move(colorRawMatD_copy);
        update[colorRawMatD_update] = 0;
        if(!colorRawMatD_copy_copy.empty())
        {
            cv::cvtColor(colorRawMatD_copy_copy, colorRawMatD_copy_copy_BGR, cv::COLOR_RGB2BGR);
            colorRawMatD_copy_copy_BGR.convertTo(colorRawMatD_copy_copy_32f, CV_32FC3);
            colorRawMatD_copy_copy = colorRawMatD_copy_copy_32f / 256;
        }
    }
    if(update[point_data_mat_xyzs_copy_update])
    {
        point_data_mat_xyzs_copy_copy = std::move(point_data_mat_xyzs_copy);
        update[point_data_mat_xyzs_copy_update] = 0;
    }
    if(update[IMU_Points_update])
    {
        IMU_Points_copy = std::move(IMU_Points);
        update[IMU_Points_update] = 0;
    }
    if(update[IMU_Points_h12_update])
    {
        IMU_Points_h12_copy = std::move(IMU_Points_h12);
        update[IMU_Points_h12_update] = 0;
    }
    if(update[robot_info_matrix_update])
    {
        robot_matrixs_copy.clear();
        for (uint i = 0; i < robot_matrixs.size(); ++i)
        {
            robot_matrixs_copy.push_back(robot_matrixs[i]);
        }
        update[robot_info_matrix_update] = 0;
    }
}

void lyn_gl::draw()
{
    glBindVertexArray(VAO[color_lines_h12]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_lines_buffer_h12]);
    glBufferData(GL_ARRAY_BUFFER, get_size(IMU_Points_h12_copy), IMU_Points_h12_copy.data, GL_STREAM_DRAW);
    glDrawArrays(GL_LINES, 0, int(get_size(IMU_Points_h12_copy)) / 24);

    glBindVertexArray(VAO[color_cloud]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_cloud_position_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(pointcloud_MatD_small_copy), pointcloud_MatD_small_copy.data, GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_cloud_color_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(colorRawMatD_copy_copy), colorRawMatD_copy_copy.data, GL_STREAM_DRAW);
    glPointSize(2);
    glDrawArrays(GL_POINTS, 0, int(get_size(pointcloud_MatD_small_copy)) / 12);

    glBindVertexArray(VAO[point_cloud]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[point_cloud_position_buffer]);
    glBufferData(GL_ARRAY_BUFFER, get_size(point_data_mat_xyzs_copy_copy), point_data_mat_xyzs_copy_copy.data, GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[point_cloud_color_buffer]);
    glPointSize(3);
    glDrawArrays(GL_POINTS, 0, int(get_size(point_data_mat_xyzs_copy_copy)) / 12);

    glBindVertexArray(VAO[color_lines]);
    glDrawArrays(GL_LINES, 0, int(get_size(IMU_Points_copy)) / 24);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    //    glEnable(GL_TEXTURE_2D);

    for (uint i = 0; i < robot_matrixs_copy.size(); ++i)
    {
        // if(i > 3) break;          //@@@ test 0 - 3
        // logger << i << "\n" << robot_matrixs_copy[i] << std::endl;

        glUniform3f(shader.colorLoc,
                    1.0f - getColorPalette24[i].r,
                    1.0f - getColorPalette24[i].g,
                    1.0f - getColorPalette24[i].b);
        model = to_mat4(gl_rotate_matrix * robot_matrixs_copy[i]);
        glUniformMatrix4fv(shader.modelLoc, 1, GL_FALSE, &model[0][0]);
        glBindVertexArray(VAO_MESHES[i]);
#define ROBOT_GL_TRIANGLES
#ifdef ROBOT_GL_TRIANGLES
        glDrawArrays(GL_TRIANGLES, 0, int(VBO_GLsizeiptrs[i]) / 24);
#else
        glDrawArrays(GL_LINES, 0, int(VBO_GLsizeiptrs[i]) / 24);
#endif

    }
    glUniform3f(shader.colorLoc, 0, 0, 0);

    //    glDisable(GL_TEXTURE_2D);
    //    glDisable(GL_LIGHTING);
    //    glDisable(GL_LIGHT0);

    model = glm::mat4(1.0f);
    glUniformMatrix4fv(shader.modelLoc, 1, GL_FALSE, &model[0][0]);

    glBindVertexArray(VAO[color_lines_robot_joints]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[color_lines_robot_joints_buffer]);
    {
        std::shared_lock<std::shared_mutex> data_lock(render_mutex);                               // 构造时自动加锁
        glBufferData(GL_ARRAY_BUFFER, get_size(Points), Points.data, GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, int(get_size(Points)) / 24);
        data_lock.unlock();
    }
}

void lyn_gl::thread_gl_main()
{
    if(!glfwInit()) { std::cerr << "Failed to initialize GLFW" << std::endl; return; }
    gl_start();
    setup_VAO();
    while (!glfwWindowShouldClose(window))
    {
        auto starttime_1 = std::chrono::high_resolution_clock::now();
        {
            std::unique_lock<std::mutex> lock(mutex["gl_main"]);
            condition_variable["gl_main"].wait(lock, [=](){return step["reconstruction_output"].load() == lyn_step_copy_complete;});
        }
        gl_update_data();
#if 1
        logger << " "
               << "pointcloud_MatD_small_copy.rows: " << pointcloud_MatD_small_copy.rows << " "
               << "pointcloud_MatD_small_copy.cols: " << pointcloud_MatD_small_copy.cols << " "
               << "colorRawMatD_copy_copy.rows: " << colorRawMatD_copy_copy.rows << " "
               << "colorRawMatD_copy_copy.cols: " << colorRawMatD_copy_copy.cols << " "
                  //                      << "point_data_mat_xyzs_copy_copy.rows: " << point_data_mat_xyzs_copy_copy.rows << " "
                  //                      << "point_data_mat_xyzs_copy_copy.cols: " << point_data_mat_xyzs_copy_copy.cols << " "
                  //                      << "IMU_Points_copy.rows: " << IMU_Points_copy.rows << " "
                  //                      << "IMU_Points_copy.cols: " << IMU_Points_copy.cols << " "
                  //                      << "IMU_Points_h12_copy.rows: " << IMU_Points_h12_copy.rows << " "
                  //                      << "IMU_Points_h12_copy.cols: " << IMU_Points_h12_copy.cols << " "
                  // << "IMU_Points_h12_copy: " << std::endl << IMU_Points_h12_copy << std::endl << " "
               << "int(get_size(IMU_Points_h12_copy)): / 24 " << std::endl << int(get_size(IMU_Points_h12_copy)) / 24 << " "
               << std::endl;
#endif
        camera_gl = camera + camera_position;
        model = glm::mat4(1.0f);
        view = glm::lookAt(to_glm_vec3(camera_gl.position),
                           to_glm_vec3(camera_gl.point_z),
                           to_glm_vec3(camera_gl.point_y - camera_gl.position));
        glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                                        // 如果有深度测试还需要启用
        glUseProgram(shader.shaderProgram);                                                        // 使用着色器程序
        glUniformMatrix4fv(shader.modelLoc, 1, GL_FALSE, &model[0][0]);                            // 将矩阵传递给着色器
        glUniformMatrix4fv(shader.viewLoc, 1, GL_FALSE, &view[0][0]);
        glUniformMatrix4fv(shader.projLoc, 1, GL_FALSE, &shader.projection[0][0]);
#if 1
//        std::cout << std::endl << "get_distance: ";
        {
            std::unique_lock<std::shared_mutex> exclusive_lock(render_mutex);
            if(!Points.empty())
            {
                double distance_min = DBL_MAX;
                int i_min = -1;
                for (int i = 0; i < Points.rows; ++i)
                {
                    double distance = get_distance(Points.at<cv::Point3f>(i, 0));
                    if(distance < distance_min)
                    {
                        distance_min =  distance;
                        i_min = i;
                    }
//                    std::cout << std::fixed << std::setw(6) << distance << ", ";
                }
                index_min = i_min;
//                std::cout << std::fixed << std::setw(6) << "i_min: " << i_min << std::endl ;
            }
        }
#endif
        draw();

        glfwSwapBuffers(window);                                                                   // 交换缓冲区和检查事件
        glfwPollEvents();

        auto starttime_2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(starttime_2 - starttime_1);
        auto sleep_time = duration.count();
        std::this_thread::sleep_for(std::chrono::microseconds(50000 - sleep_time));
        auto starttime_3 = std::chrono::high_resolution_clock::now();
        auto duration_all = std::chrono::duration_cast<std::chrono::microseconds>(starttime_3 - starttime_1);
        logger << duration.count() << " gl working... "  << " fps: " << 1 / (double(duration_all.count()) / 1000000) << std::endl;
    }
    glDeleteProgram(shader.shaderProgram);                                                         // 清理资源
    glfwTerminate();                                                                               // 终止GLFW
    return;
}

int lyn_gl::run(int argc, char **argv)
{
    std::ignore = argv;
    logger.run("gl.log", false);

    thread["reconstruction_output"] = std::thread(&lyn_gl::thread_reconstruction_data, this);
    thread["robot_info_meshes"] = std::thread(&lyn_gl::thread_robot_meshes_data, this);
    thread["robot_info_joints_points"] = std::thread(&lyn_gl::thread_robot_joints, this);          // robot_info_joints_points 换成 info_joints_points就不行？
    thread["robot_info_matrixs"] = std::thread(&lyn_gl::thread_robot_info_matrixs, this);

    if(argc < 2) { return 1; }

    thread["gl_main"] = std::thread(&lyn_gl::thread_gl_main, this);

    return 0;
}

glm::vec3 lyn_gl::to_glm_vec3(cv::Point3d cv_point3d)
{
    return glm::vec3(cv_point3d.x, cv_point3d.y, cv_point3d.z);
}

cv::Point3f lyn_gl::to_cv_Point3f(glm::vec3 vec3)
{
    return cv::Point3f(vec3.x, vec3.y, vec3.z);
}

lyn_gl::~lyn_gl()
{
    running = 0;
}

int lyn_gl::work(lyn_info &info)
{
    info.key_press = key_press;
    key_press = 0;
    info.index_min = index_min;
    index_min = -1;

    info.unload(names["reconstruction_output"], step, input_public, condition_variable);
    info.unload(names["robot_info_meshes"], step, input_public, condition_variable);
    info.unload(names["robot_info_matrixs"], step, input_public, condition_variable);
    info.unload(names["robot_info_joints_points"], step, input_public, condition_variable);
#if 0
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "lyn_gl::work: robot_info_meshes = "
              << SetpString[info.steps["robot_info_meshes"]] << std::endl;
#endif
    return 0;
}

