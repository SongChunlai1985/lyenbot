#include <app.h>

app::app()
{
}


int app::run(int argc, char *argv[])
{
    Inputs.run(argc, argv);
    Plan.run(argc, argv);
    Control.run(argc, argv);
    Outputs.run(argc, argv);

    logger.run("app.log", false);
    std::cout << "[" << getCurrentTimeWithMicroseconds() << "] " << "App running ..." << std::endl;
    while (running)
    {
        auto starttime_1 = std::chrono::high_resolution_clock::now();

        Inputs.work(info);
        Plan.work(info);
        Control.work(info);
        Outputs.work(info);

        auto starttime_2 = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(starttime_2 - starttime_1);
        long count = duration.count();
        std::this_thread::sleep_for(std::chrono::microseconds(500 - count));                    //限制到500us每帧
        if(int(count) > 50)
        {
            logger << "working... "  << " duration: " << int(count) << " us  "
                   << info.steps["onnx_output_public"] << std::endl;
        }
    }

    return 0;
}

app::~app()
{
    running = false;
}
