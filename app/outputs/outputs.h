#ifndef LYN_OUTPUTS_H
#define LYN_OUTPUTS_H
#include <axis_angle.h>
#include <lyn_gl.h>
#include <lyn_log.h>

class outputs
{
private:

public:
    outputs();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

    axis_angle Axis_angle;
    lyn_gl Lyn_gl;
};
#endif // LYN_OUTPUTS_H
