#ifndef LYN_CONTROL_H
#define LYN_CONTROL_H
//#include <onnx_inference.h>
#include <direct_control.h>
class control
{
private:

public:
    control();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
//    onnx_inference Onnx_inference;
    direct_control Direct_control;
};
#endif // LYN_INPUTS_H
