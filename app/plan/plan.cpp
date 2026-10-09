#include <plan.h>

plan::plan()
{

}

int plan::run(int argc, char *argv[])
{
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "Plan running ..." << std::endl;
    Avoiding.run(argc, argv);
    Reconstruction.run(argc, argv);
//    Face_recognition.run(argc, argv);
//    Object_detection.run(argc, argv);
    return 0;
}

int plan::work(lyn_info &info)
{
    Avoiding.work(info);
    Reconstruction.work(info);
//    Face_recognition.work(info);
//    Object_detection.work(info);
    return 0;
}
