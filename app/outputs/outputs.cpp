#include <outputs.h>

outputs::outputs()
{

}

int outputs::run(int argc, char *argv[])
{
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "outputs running ..." << std::endl;
    Lyn_gl.run(argc, argv);
    Axis_angle.run(argc, argv);
    return 0;
}

int outputs::work(lyn_info &info)
{
    Lyn_gl.work(info);
    Axis_angle.work(info);
    return 0;
}
