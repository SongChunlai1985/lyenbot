Lyenbot
一个基于 C++17 的机器人实时控制系统，集成了 IMU、CAN 总线、视觉感知、ONNX 推理、步态控制与 OpenGL 可视化。项目采用模块化设计，适用于人形机器人或足式机器人的感知-规划-控制-输出闭环。

功能特性
多传感器输入

Hi12 IMU 串口数据读取（921600 波特率）

CAN 总线电机通信（支持 can0 ~ can3，1 Mbps）

Orbbec 深度相机（点云 + 彩色图 + IMU）

Livox Mid360 激光雷达点云

RealSense 深度相机

状态估计与重建

IMU 数据卡尔曼滤波、互补滤波、Madgwick 姿态融合

点云重建与坐标变换

规划与控制

直接控制（关节角度直接下发）

基于 ONNX 的神经网络推理控制

人形机器人行走步态生成（Walking 模块）

踝关节并联机构运动学正逆解

平衡控制（陀螺仪反馈）

输出与可视化

OpenGL 3D 可视化（点云、机器人模型、关节坐标轴、IMU 轨迹）

轴角度输出、电机状态监控

基础组件

线程安全日志系统

基于 lyn_info 的模块间数据交换与状态机

URDF 解析与机器人运动学树构建

串口通信（serialib）

CAN 通信封装

目录结构
text
lyenbot/
├── CMakeLists.txt              # 根 CMake，生成 lyenbot 可执行文件
├── main.cpp                    # 程序入口
├── app/                        # 应用主框架
│   ├── CMakeLists.txt
│   ├── app.cpp / app.h         # 主循环，调度 Inputs/Plan/Control/Outputs
│   ├── common/                 # 通用组件（日志、信息、元数据、CAN、串口等）
│   ├── inputs/                 # 输入模块（IMU、CAN、视觉）
│   ├── plan/                   # 规划模块（重建、ONNX 推理等）
│   ├── control/                # 控制模块（直接控制、行走控制）
│   └── outputs/                # 输出模块（OpenGL、轴角度）
└── ...                         # 其他源文件与配置
依赖项
编译环境：CMake ≥ 3.10，支持 C++17 的编译器（GCC / Clang）

第三方库：

OpenCV（core, imgproc, dnn, calib3d, highgui）

GLFW3

GLEW

GLM

ONNX Runtime 或 OpenCV DNN（CPU / CUDA）

Orbbec SDK

Livox SDK

RealSense SDK

serialib

pthread

硬件：

Hi12 IMU（串口）

CAN 总线电机驱动器

Orbbec 深度相机

Livox Mid360

RealSense 相机

构建
bash
git clone <repository-url>
cd lyenbot
mkdir build && cd build
cmake ..
make -j$(nproc)
默认 THIRDPARTY_DIR 设置为 /home/lyenbot/depends_x86，如需修改请编辑根 CMakeLists.txt。

运行
bash
./lyenbot
程序启动后会依次初始化输入、规划、控制、输出模块，并进入 500 μs 周期的实时循环。

需要提前配置好 CAN 接口、串口权限及设备节点。

若需显示可视化窗口或调试图像，可传入参数（argc > 1）。

配置与设备
项目	默认值 / 说明
CAN 接口	can0 ~ can3，比特率 1000000
IMU 串口	/dev/ttyUSB0，波特率 921600
行走配置文件	/home/lyenbot/config/Walking_config.yml
PD 模式配置	/home/lyenbot/config/PD_mode_config.yml
动作文件	/home/lyenbot/modles/new_control.log
ONNX 模型	代码中 modelPath 指定
日志文件	app.log, imu.log, can_in.log, gl.log, direct_control.log, axis_angle.log, reconstruction.log, onnx.log 等
部分操作需要 sudo 权限（如设置 CAN 接口、修改串口属主），请根据实际环境调整。

模块简介
Inputs：imu、can_in、vision（Orbbec / Mid360 / RealSense）

Plan：reconstruction（点云重建）、onnx_inference（神经网络推理）

Control：direct_control（直接控制）、axis_angle（关节角度控制）

Outputs：lyn_gl（OpenGL 可视化）、axis_angle（轴角度输出）

Common：lyn_log、lyn_info、lyn_meta、lyn_can、serialib、kalman_filter、imu_filter_madgwick、imu_complementary_filter 等

注意事项
项目注释中提到“有 IMU 才能启动”，请确保 IMU 设备正常连接。

实时循环周期为 500 μs，若单帧耗时超过 50 μs 会记录日志。

部分硬件驱动和 SDK 需要单独安装并配置环境变量。

日志系统默认关闭控制台打印，可通过 logger.run(filename, true) 开启。建议使用tail -f 文件名查看日志。

许可证
本项目仅供学习与研究使用。涉及第三方库请遵循其各自的开源许可证。

README 根据项目源码自动生成，具体细节请以实际代码为准。

捐赠
BTC: 13SongiriQuWoFhoimsVS21CyaTxozKBVA
