#include <inputs.h>

inputs::inputs()
{

}

int inputs::run(int argc, char *argv[])
{
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "inputs running ..." << std::endl;
    // Vision.run(argc, argv);
    Can_in.run(argc, argv);
    Imu.run(argc, argv);
    return 0;
}

int inputs::work(lyn_info &info)
{
    // Vision.work(info);
    Can_in.work(info);
    Imu.work(info);
    return 0;
}
