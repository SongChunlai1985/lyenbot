#include <lyn_log.h>

lyn_log::lyn_log()
{
}

void lyn_log::save_log(std::string log_string)
{
    if(print)
    {
        std::cout << log_string;
    }
    logfile << log_string;
}

void lyn_log::log(std::string log_string)
{
    mutex_log.lock();
    logsA.push_back(log_string);
    mutex_log.unlock();
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.notify_one();
    }
}

void lyn_log::run(std::string filename_, bool print_)                                              // filename不能重复
{
    logfile = std::ofstream(filename_);
    print = print_;
    log_thread = std::thread([=]()
    {
        while (running)
        {
            std::unique_lock<std::mutex> lock(mutex);
            condition_variable.wait(lock, [=]
            {
                mutex_log.lock();
                int size = int(logsA.size());
                mutex_log.unlock();
                return size;
            });

            mutex_log.lock();
            for (uint i = 0; i < logsA.size(); ++i)
            {
                save_log(logsA.front());
                logsA.pop_front();
            }
            mutex_log.unlock();
        }
        logfile.close();
    }
    );

    flush_thread = std::thread([=]()
    {
        while (running)
        {
            usleep(100000);
            mutex_log.lock();
            logfile.flush();                                                                   // 手动刷新缓冲区, 否则文件不完整
            mutex_log.unlock();
        }
        logfile.close();
    }
    );
}

std::string lyn_log::getCurrentTimeWithMicroseconds_()
{
    auto now = std::chrono::system_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>
            (now.time_since_epoch());
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm local_time = *std::localtime(&time_t_now);
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0')
        << (microseconds.count() % 1000000);
    return oss.str();
}
