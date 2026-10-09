#ifndef LYN_INPUTS_H
#define LYN_INPUTS_H

// #include <vision.h>
#include <imu.h>
#include <can_in.h>


class inputs
{
private:

public:
    inputs();
    // vision Vision;
    imu Imu;
    can_in Can_in;
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
};
#endif // LYN_INPUTS_H
