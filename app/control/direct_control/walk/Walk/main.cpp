#include <array>
#include <stdint.h>
#include <stdio.h>

#include "Walking.h"

// Walking.cpp expects these globals to exist.
float g_fb_gyro = 0.0f;
float g_rl_gyro = 0.0f;
int hip_offset = 0;
int knee_offset = 0;
int ankle_offset = 0;

// Stubs for platform hooks used by Walking.cpp.
extern "C" void Motor_SetPosition(uint8_t* motor_id, short* motor_pos, uint8_t motor_num)
{
    (void)motor_id;
    (void)motor_pos;
    (void)motor_num;
}

extern "C" uint8_t GetWorkingState(void)
{
    return 1;
}

extern "C" void WalkingWalkStateChange(void)
{
}

extern Walking walking;

// Input: time in ms. Output: one frame (16 joints).
std::array<double, 16> GetFrameAtTimeMs(int time_ms)
{
    static bool initialized = false;
    static int last_time_ms = -1;

    if (!initialized || time_ms < last_time_ms)
    {
        walking.Initialize();
        walking.Start();
        walking.time_Msec = 5; // record every 5 ms
        initialized = true;
    }

    walking.Process();
    last_time_ms = time_ms;

    std::array<double, 16> frame{};
    for (int i = 0; i < 16; ++i)
    {
        frame[i] = (double)walking.m_Joint.WalkAngle[i];
    }
    return frame;
}

int main(void)
{
    for (int t = 0; t <= 1000; t += 5)
    {
        std::array<double, 16> frame = GetFrameAtTimeMs(t);
        printf("t=%dms:", t);
        for (int i = 0; i < 16; ++i)
        {
            printf(" %.3f", frame[i]);
        }
        printf("\n");
    }
    return 0;
}
