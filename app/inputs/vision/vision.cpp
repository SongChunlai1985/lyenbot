#include <vision.h>

vision::vision()
{

}

int vision::run(int argc, char *argv[])
{
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "vision running ..." << std::endl;
    Orbbec.run(argc, argv);
    Mid360.run(argc, argv);
    Real_sense.run(argc, argv);
    return 0;
}

int vision::work(lyn_info &info)
{
    Mid360.work(info);
    Orbbec.work(info);
    Real_sense.work(info);
    return 0;
}
