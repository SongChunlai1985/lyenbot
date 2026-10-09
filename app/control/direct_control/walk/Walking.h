/*
 *   Walking.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef WALKING_ENGINE_H
#define WALKING_ENGINE_H

#include <string.h>
#include <cstdint>
#include <iostream>

typedef unsigned           char 		u8;
typedef unsigned short     int 			u16;

#define WALKING_SECTION "Walking Config"
#define INVALID_VALUE   -1024.0

#define MOTION_CNT 30

#define CAMERA_DISTANCE   33.2                   // mm // x轴 摄像头与质心上半身质心的距离
#define EYE_TILT_OFFSET_ANGLE   40.0             // degree

#define LEG_SIDE_OFFSET   116                    //mm  //37.0(darwin);质心与腿之间距离 y轴 //原本是116
#define THIGH_LENGTH   320                       //mm //93.0(darwin); 大腿长度 z轴
#define CALF_LENGTH   360                        //mm  //93.0(darwin); 小腿长度 z轴
#define ANKLE_LENGTH  40.7                       //mm  //33.5(darwin);脚底与踝关节距离   z轴    46 + 5.5 增加橡胶鞋垫5.5
#define LEG_LENGTH   (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)    //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

//namespace Robot
//{
class Walking
{
public:
    double time_Msec;
    Walking();
    double m_Time;
    bool ctrl_Running;
    bool real_Running;


private:
    enum
    {
        RHipYaw_Index = 0,
        RHipRoll_Index,
        RHipPitch_Index,
        RKneePitch_Index,
        RAnklePitch_Index,
        RAnkleRoll_Index,
        LHipYaw_Index,
        LHipRoll_Index,
        LHipPitch_Index,
        LKneePitch_Index,
        LAnklePitch_Index,
        LAnkleRoll_Index,
        RShoulderPitch_Index,
        LShoulderPitch_Index,
        RElbowRoll_Index,
        LElbowRoll_Index
    };

    enum
    {
        R_HIP_YAW,
        R_HIP_ROLL,
        R_HIP_PITCH,
        R_KNEE,
        R_ANKLE_PITCH,
        R_ANKLE_ROLL,

        L_HIP_YAW,
        L_HIP_ROLL,
        L_HIP_PITCH,
        L_KNEE,
        L_ANKLE_PITCH,
        L_ANKLE_ROLL,

        R_ARM_SWING,
        L_ARM_SWING,
        R_ELBOW,
        L_ELBOW,

        NUMBER_OF_JOINTS
    };
    //		static Walking* m_UniqueInstance;
    // variable for walking

    double x_Swap_PeriodTime;
    double x_Move_PeriodTime;
    double y_Swap_PeriodTime;
    double y_Move_PeriodTime;
    double z_Swap_PeriodTime;
    double z_Move_PeriodTime;
    double a_Move_PeriodTime;
    double ssp_Time;
    double ssp_Time_Start_L;
    double ssp_Time_End_L;
    double ssp_Time_Start_R;
    double ssp_Time_End_R;
    double phase_Time1;
    double phase_Time2;
    double phase_Time3;

    double x_Offset;
    double y_Offset;
    double z_Offset;
    double r_Offset;
    double p_Offset;
    double a_Offset;

    double x_Swap_Phase_Shift;
    double x_Swap_Amplitude;
    double x_Swap_Amplitude_Shift;
    double x_Move_Phase_Shift;
    double x_Move_Amplitude;
    double x_Move_Amplitude_Shift;
    double y_Swap_Phase_Shift;
    double y_Swap_Amplitude;
    double y_Swap_Amplitude_Shift;
    double y_Move_Phase_Shift;
    double y_Move_Amplitude;
    double y_Move_Amplitude_Shift;
    double z_Swap_Phase_Shift;
    double z_Swap_Amplitude;
    double z_Swap_Amplitude_Shift;
    double z_Move_Phase_Shift;
    double z_Move_Amplitude;
    double z_Move_Amplitude_Shift;
    double a_Move_Phase_Shift;
    double a_Move_Amplitude;
    double a_Move_Amplitude_Shift;

    double hip_Pitch_Offset;
    double hip_Pitch_Offset_parm;
    //		double m_Arm_Swing_Gain;

    //		int    m_Phase;
    //		double body_Swing_Y;
    //		double body_Swing_Z;

    u16 WalkValue[MOTION_CNT];
    u8 WalkID[MOTION_CNT];

    double wsin(double time, double period, double period_shift, double mag, double mag_shift);
    bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
    void update_param_time();
    void update_param_move();
    void update_param_balance();

    const int MX28_CENTER_VALUE = 0;
    const int MX28_MIN_VALUE = -2048;
    const int MX28_MAX_VALUE = 2048;
    const double MX28_MIN_ANGLE = -180.0; // degree
    const double MX28_MAX_ANGLE = 180.0; // degree
    const double MX28_RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
    const double MX28_RATIO_ANGLE2VALUE = 11.378; // 4096 / 360

    const int MX28_PARAM_BYTES = 7;

    double MX28_Angle2Value(double angle);
    double MX28_Value2Angle(double value);
    void SetIdAndAngle(int index, double value);

public:
    // Walking initial pose
    double init_x_offset;
    double init_y_offset;
    double init_z_offset;
    double init_a_offset;
    double init_p_offset;
    double init_r_offset;

    // Walking control
    double periodTime;
    double dsp_Ratio;
    double ssp_Ratio;
    double step_fb_Ratio;
    double init_x_Move_Amplitude;
    double init_y_Move_Amplitude;
    double init_z_Move_Amplitude;
    double init_a_Move_Amplitude;
    double init_y_Swap_Amplitude;
    double init_z_Swap_Amplitude;
    bool a_Move_Aim_On;
    double pelvis_Offset_parm;
    double pelvis_Offset;
    double pelvis_Swing;
    double pelvis_Swing_parm;

    // Balance control
    bool   balance_Enable;
    double balance_Knee_Gain;//BALANCE_KNEE_GAIN;
    double balance_Ankle_Pitch_Gain;//BALANCE_ANKLE_PITCH_GAIN;
    double balance_Hip_Roll_Gain;//BALANCE_HIP_ROLL_GAIN;
    double balance_Ankle_Roll_Gain;//BALANCE_ANKLE_ROLL_GAIN;
    double balance_Hip_Pitch_Gain;//BALANCE_HIP_PITCH_GAIN;
    double arm_Swing_Gain;
    //		double PELVIS_OFFSET;
    double init_hip_Pitch_Offset;

    //		int    P_GAIN;
    //		int    I_GAIN;
    //		int    D_GAIN;

    //		int GetCurrentPhase()		{ return m_Phase; }
    //		float GetBodySwingY()		{ return m_Body_Swing_Y; }
    //		float GetBodySwingZ()		{ return m_Body_Swing_Z; }

    //		static Walking* GetInstance() { return m_UniqueInstance; }

    void Initialize();
    void Start();
    void Stop();
    void Process();
    bool IsRunning();

    virtual ~Walking();
    //        void LoadINISettings(minIni* ini);
    //        void LoadINISettings(minIni* ini, const std::string &section);
    //        void SaveINISettings(minIni* ini);
    //        void SaveINISettings(minIni* ini, const std::string &section);

    double g_fb_gyro = 0.0;
    double g_rl_gyro = 0.0;

    uint8_t WalkTime = 0;

    double Pelvis_offset_l = 1;
    double Pelvis_offset_r = 1;
    double Pelvis_offset_lr = 1;

    uint8_t WalkLineSype = 0; //0直线   1左转   2右转  3左横移   4右横移
    int step = 0;
    int last_step = -1;
    int state = 0;
    void read_config(std::string filename);
    void write_config(std::string filename);

    double WalkAngle[NUMBER_OF_JOINTS];
};
//}

#endif
