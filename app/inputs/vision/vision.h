#ifndef LYN_VISION_H
#define LYN_VISION_H

#include <thread>
#include <unistd.h>
#include <orbbec.h>
#include <lyn_info.h>
#include <mid360.h>
#include "real_sense.h"

class vision
{
private:

public:
    vision();
    orbbec Orbbec;
    mid360 Mid360;
    real_sense Real_sense;
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
};
#endif // LYN_VISION_H
