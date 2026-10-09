#ifndef LYN_TTYUSB_H
#define LYN_TTYUSB_H

#include <serialib.h>

class usbtty : public serialib
{
public:
    usbtty();
    void run();
};

#endif // LYN_TTYUSB_H
