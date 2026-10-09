#include <control.h>

control::control()
{

}

int control::run(int argc, char *argv[])
{
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "inputs running ..." << std::endl;
    Direct_control.run(argc, argv);
//    Onnx_inference.run(argc, argv);
    return 0;
}

int control::work(lyn_info &info)
{
    Direct_control.work(info);
//    Onnx_inference.work(info);
    return 0;
}
