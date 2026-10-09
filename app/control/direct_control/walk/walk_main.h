
#ifndef WALK_MAIN_H
#define WALK_MAIN_H

#include <lyn_log.h>
#include <lyn_info.h>
#include <lyn_can.h>
#include <lyn_meta.h>

#include "Walking.h"

class Walk
{
public:
    Walk();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);
    cv::Mat GetFrameAtTimeMs();

    void WalkTimeInitialize();
    void SetWalkingStepSize(float step, float stepHeight);
    void walking_process(uint8_t walk_line);
    void walking_init();
    void set_time_msec(double time);
    void stop();
    double get_m_Time();
    void start();
    Walking walking;

    void set_m_Time(double time);
    int get_state();
private:

    lyn_log logger;
    std::thread thread[2];

    std::atomic<lyn_step> step;
    std::mutex mutex;
    lyn_Infos Walk_public;
    std::condition_variable condition_variable;

    std::atomic<int> key_press = 0;

    int hip_offset = 0;
    int knee_offset = 0;
    int ankle_offset = 0;


     std::unordered_map<int, std::string> ID_to_name_map =
     {
         {0, "RHipYaw"},
         {1, "RHipRoll"},
         {2, "RHipPitch"},
         {3, "RKneePitch"},
         {4, "RAnklePitch"},
         {5, "RAnkleRoll"},

         {6, "LHipYaw"},      // 0
         {7, "LHipRoll"},
         {8, "LHipPitch"},
         {9, "LKneePitch"},
         {10, "LAnklePitch"},
         {11, "LAnkleRoll"},

         {12, "RShoulderPitch"},
         {13, "LShoulderPitch"},

         {14, "RElbowPitch"},
         {15, "LElbowPitch"}
     };


     std::unordered_map<std::string, float> walk_motor_direction =
     {
         {"RHipYaw", 1},
         {"RHipRoll", -1},
         {"RHipPitch", 1},
         {"RKneePitch", 1},
         {"RAnklePitch", 1},
         {"RAnkleRoll", 1},

         {"LHipYaw", 1},
         {"LHipRoll", -1},
         {"LHipPitch", 1},
         {"LKneePitch", 1},
         {"LAnklePitch", 1},
         {"LAnkleRoll", 1},

         {"RShoulderPitch", -1},
         {"LShoulderPitch", 1},

         {"RElbowPitch", 1},
         {"LElbowPitch", 1}
     };

     cv::Mat Frame_RAD = cv::Mat(1, 128, CV_32FC1, cv::Scalar(0.0));
     bool initialized = false;

};
#endif // WALK_MAIN_H
