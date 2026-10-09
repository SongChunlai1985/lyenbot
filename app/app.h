#ifndef LYN_APP_H
#define LYN_APP_H
#include <signal.h>

#include <inputs.h>
#include <plan.h>
#include <control.h>
#include <outputs.h>

class app
{
private:

public:
    app();
    ~app();
    inputs Inputs;
    plan Plan;
    control Control;
    outputs Outputs;
    int run(int argc, char *argv[]);
    int running = true;

    lyn_log logger;
    lyn_info info;
};
#endif // LYN_APP_H
