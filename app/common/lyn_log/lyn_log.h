#ifndef LYN_LOG_H
#define LYN_LOG_H

#include <fstream>
#include <thread>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <list>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <condition_variable>

class lyn_log
{
public:
    lyn_log();
    void log(std::string log_string);
    template<typename T>
    lyn_log& operator << (const T& value)
    {
        std::ostringstream oss;
        if(newline) {
            oss << "[" << getCurrentTimeWithMicroseconds_() << "] " << value;
            newline = false;
        } else {
            oss << value;
        }
        log(oss.str());
        return *this;
    }

    lyn_log& operator << (std::ostream& (*manip)(std::ostream&))
    {
        std::ostringstream oss;
        if(newline)
        {
            oss << "[" << getCurrentTimeWithMicroseconds_() << "] ";
            newline = false;
        }
        oss << manip;
        log(oss.str());

        if(manip == static_cast<std::ostream&(*)(std::ostream&)>(&std::endl))
        {
            newline = true;
        }
        return *this;
    }

    void run(std::string filename_ = "lyn_log.txt", bool print_ = true);
private:
    std::ofstream logfile;
    std::list<std::string> logsA;
    int running = 1;
    void save_log(std::string log_string);
    std::thread log_thread;
    std::thread flush_thread;
    std::string last_log_string;
    bool print = true;
    std::mutex mutex;
    std::mutex mutex_log;
    bool newline = true;
    std::condition_variable condition_variable;
    std::string getCurrentTimeWithMicroseconds_();
};
#endif // LYN_LOG_H
