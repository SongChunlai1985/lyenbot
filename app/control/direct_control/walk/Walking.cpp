#define BALANCE_FLAG   // @@@ 20260110

#ifdef BALANCE_FLAG
#include <stdio.h>
#include <math.h>
#include "Vector.h"
#include "Matrix.h"
#include "MX28.h"
#include "Walking.h"

#include "opencv2/core.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
#define PI (3.14159265)
#define HIP_YAW_OFFSET_LIMIT  30   // 226  34b  6d6
//extern int hip_offset, knee_offset, ankle_offset;
#ifdef __cplusplus  
}
#endif


Walking::Walking()
{
}

Walking::~Walking()
{
}

void Walking::read_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["periodTime"] >> periodTime;
    fs["dsp_Ratio"] >> dsp_Ratio;
    fs["step_fb_Ratio"] >> step_fb_Ratio;
    fs["time_Msec"] >> time_Msec;

    fs["init_x_Move_Amplitude"] >> init_x_Move_Amplitude;
    fs["init_y_Move_Amplitude"] >> init_y_Move_Amplitude;
    fs["init_z_Move_Amplitude"] >> init_z_Move_Amplitude;

    fs["balance_Ankle_Roll_Gain"] >> balance_Ankle_Roll_Gain;
    fs["balance_Ankle_Pitch_Gain"] >> balance_Ankle_Pitch_Gain;
    fs["balance_Knee_Gain"] >> balance_Knee_Gain;
    fs["balance_Hip_Roll_Gain"] >> balance_Hip_Roll_Gain;

    fs["init_y_Swap_Amplitude"] >> init_y_Swap_Amplitude;
    fs["init_z_Swap_Amplitude"] >> init_z_Swap_Amplitude;
    fs["arm_Swing_Gain"] >> arm_Swing_Gain;

    fs["pelvis_Offset_parm"] >> pelvis_Offset_parm;
    fs["pelvis_Swing_parm"] >> pelvis_Swing_parm;

    fs["init_x_offset"] >> init_x_offset;
    fs["init_y_offset"] >> init_y_offset;
    fs["init_z_offset"] >> init_z_offset;

    fs["init_r_offset"] >> init_r_offset;
    fs["init_p_offset"] >> init_p_offset;
    fs["init_a_offset"] >> init_a_offset;

    fs["hip_Pitch_Offset_parm"] >> hip_Pitch_Offset_parm;
    fs["init_a_Move_Amplitude"] >> init_a_Move_Amplitude;
    fs["a_Move_Aim_On"] >> a_Move_Aim_On;
    fs["balance_Enable"] >> balance_Enable;
    fs["WalkTime"] >> WalkTime;
    fs.release();
}

void Walking::write_config(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "periodTime" << periodTime;
    fs << "dsp_Ratio" << dsp_Ratio;
    fs << "step_fb_Ratio" << step_fb_Ratio;
    fs << "time_Msec" << time_Msec;

    fs << "init_x_Move_Amplitude" << init_x_Move_Amplitude;
    fs << "init_y_Move_Amplitude" << init_y_Move_Amplitude;
    fs << "init_z_Move_Amplitude" << init_z_Move_Amplitude;

    fs << "balance_Ankle_Roll_Gain" << balance_Ankle_Roll_Gain;
    fs << "balance_Ankle_Pitch_Gain" << balance_Ankle_Pitch_Gain;
    fs << "balance_Knee_Gain" << balance_Knee_Gain;
    fs << "balance_Hip_Roll_Gain" << balance_Hip_Roll_Gain;

    fs << "init_y_Swap_Amplitude" << init_y_Swap_Amplitude;
    fs << "init_z_Swap_Amplitude" << init_z_Swap_Amplitude;
    fs << "arm_Swing_Gain" << arm_Swing_Gain;

    fs << "pelvis_Offset_parm" << pelvis_Offset_parm;
    fs << "pelvis_Swing_parm" << pelvis_Swing_parm;

    fs << "init_x_offset" << init_x_offset;
    fs << "init_y_offset" << init_y_offset;
    fs << "init_z_offset" << init_z_offset;

    fs << "init_r_offset" << init_r_offset;
    fs << "init_p_offset" << init_p_offset;
    fs << "init_a_offset" << init_a_offset;

    fs << "hip_Pitch_Offset_parm" << hip_Pitch_Offset_parm;
    fs << "init_a_Move_Amplitude" << init_a_Move_Amplitude;
    fs << "a_Move_Aim_On" << a_Move_Aim_On;
    fs << "balance_Enable" << balance_Enable;
    fs << "WalkTime" << WalkTime;

    fs.release();
}

void Walking::Initialize()
{
    periodTime = 1200;                                     // 600     // Ts  1200
    dsp_Ratio = 0.35;                                      // 0.1  // r 双脚支撑时间占步行周期比例
    ssp_Ratio = 1 - dsp_Ratio;
    step_fb_Ratio = 0.28;                                  // 0.28 // step_forward_back_ratio
    time_Msec = 10;

    init_x_Move_Amplitude = 100;                            // 30  // pmx  步长
    init_y_Move_Amplitude = 0;                             // pmy  沿着X前进 Y轴为0
    init_z_Move_Amplitude = 25;                            // 50  // pmz  抬脚高度
    
    balance_Ankle_Roll_Gain = -0.5;                           // -1 // -0.5   10
    balance_Ankle_Pitch_Gain = 0.4;                          // 0.4   //1.2
    balance_Knee_Gain = 0.2;                                 // 0.2   //0.6
    balance_Hip_Roll_Gain = 0.5;                             // -0.5 //0.5

    init_y_Swap_Amplitude = 11.6;                           // 20  越大 左右偏移越大
    init_z_Swap_Amplitude = 13;                            // pbz 机器人身高1%左右
    arm_Swing_Gain = 0.2;                                  // 1.5 摆臂幅度

    pelvis_Offset_parm = 5;
    pelvis_Offset = pelvis_Offset_parm * MX28_RATIO_ANGLE2VALUE;           // 10   左右摆幅 //如果init_y_swap_amplitiude 为20 单位度 ankle roll -- 设置10的时候 ankelroll转正负9度，20的时候是转大概正负19
    pelvis_Swing_parm = 1;
    pelvis_Swing = pelvis_Offset * pelvis_Swing_parm;                       // 0.35  右脚相对摇晃幅度  1
    
    init_x_offset = -20;	                               // 调整机器人前倾的幅度  减小小往前倾-30
    init_y_offset = 0.0;	                               // 调整机器人左右晃动的幅度  增大往右偏移30
    init_z_offset = 30;                                    // 调整机器人下蹲的幅度 增大下蹲幅度越大30
    init_r_offset = 0;
    init_p_offset = 0;
    init_a_offset = 0;
    hip_Pitch_Offset_parm = 1;
    hip_Pitch_Offset = hip_Pitch_Offset_parm * MX28_RATIO_ANGLE2VALUE;       // 增大前倾   1.8
    
    init_a_Move_Amplitude = 0;
    a_Move_Aim_On = false;
    balance_Enable = true;
    
    x_Swap_Phase_Shift   =    PI;                          // -det_bx
    x_Swap_Amplitude_Shift =  0 ;                          // ubx
    x_Move_Phase_Shift =      (PI / 2);                    // -det_mx
    x_Move_Amplitude_Shift =  0;                           // umx
    y_Swap_Phase_Shift =      0;                           // -det_by
    y_Swap_Amplitude_Shift =  0;                           // uby
    y_Move_Phase_Shift =      (PI / 2);                    // -det_my
    z_Swap_Phase_Shift =      (PI * 3 / 2);                // -det_bz
    z_Move_Phase_Shift =      (PI / 2);                    // -det_mz
    a_Move_Phase_Shift =      (PI / 2);

    ctrl_Running = false;
    real_Running = false;
    m_Time = 0;

//    write_config("/home/lyenbot/config/Walking_config.yml");
    read_config("/home/lyenbot/config/Walking_config.yml");
}

void Walking::update_param_time()
{
    x_Swap_PeriodTime = periodTime / 2;                    // 2pi/wbx
    x_Move_PeriodTime = periodTime * ssp_Ratio;            // 2pi/wbm
    y_Swap_PeriodTime = periodTime;                        // 2pi/wby
    y_Move_PeriodTime = periodTime * ssp_Ratio;            // 2pi/wmy
    z_Swap_PeriodTime = periodTime / 2;                    // 2pi/wbz
    z_Move_PeriodTime = periodTime * ssp_Ratio / 2;        // 2pi/wmz
    a_Move_PeriodTime = periodTime * ssp_Ratio;

    ssp_Time = periodTime * ssp_Ratio;
    ssp_Time_Start_L = (1 - ssp_Ratio) * periodTime / 4;
    ssp_Time_End_L = (1 + ssp_Ratio) * periodTime / 4;
    ssp_Time_Start_R = (3 - ssp_Ratio) * periodTime / 4;
    ssp_Time_End_R = (3 + ssp_Ratio) * periodTime / 4;

    phase_Time1 = (ssp_Time_End_L + ssp_Time_Start_L) / 2;
    phase_Time2 = (ssp_Time_Start_R + ssp_Time_End_L) / 2;
    phase_Time3 = (ssp_Time_End_R + ssp_Time_Start_R) / 2;
}

void Walking::update_param_move()
{
    // Forward/Back
    x_Move_Amplitude = init_x_Move_Amplitude;
    x_Swap_Amplitude = x_Move_Amplitude * step_fb_Ratio;   // pbx

    // Right/Left
    y_Move_Amplitude = init_y_Move_Amplitude / 2;          // pmy
    if(y_Move_Amplitude > 0)
        y_Move_Amplitude_Shift = y_Move_Amplitude;
    else
        y_Move_Amplitude_Shift = -y_Move_Amplitude;
    y_Swap_Amplitude = init_y_Swap_Amplitude + y_Move_Amplitude_Shift * 0.04;   // pby

    z_Move_Amplitude = init_z_Move_Amplitude / 2;          // pmz
    z_Move_Amplitude_Shift = z_Move_Amplitude / 2;
    z_Swap_Amplitude = init_z_Swap_Amplitude;              // pbz
    z_Swap_Amplitude_Shift = z_Swap_Amplitude;

    // Direction
    if(a_Move_Aim_On == false)
    {
        a_Move_Amplitude = init_a_Move_Amplitude * PI / 180.0 / 2;
        if(a_Move_Amplitude > 0)
            a_Move_Amplitude_Shift = a_Move_Amplitude;
        else
            a_Move_Amplitude_Shift = -a_Move_Amplitude;
    }
    else
    {
        a_Move_Amplitude = -a_Move_Amplitude * PI / 180.0 / 2;
        if(a_Move_Amplitude > 0)
            a_Move_Amplitude_Shift = -a_Move_Amplitude;
        else
            a_Move_Amplitude_Shift = a_Move_Amplitude;
    }
}

void Walking::update_param_balance()
{
    x_Offset = init_x_offset;
    y_Offset = init_y_offset;
    z_Offset = init_z_offset;
    r_Offset = init_r_offset * PI / 180.0;
    p_Offset = init_p_offset * PI / 180.0;
    a_Offset = init_a_offset * PI / 180.0;
    //    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*MX28_RATIO_ANGLE2VALUE;
}

void Walking::Start()
{
    ctrl_Running = true;
    real_Running = true;
}

void Walking::Stop()
{
    ctrl_Running = false;
}

bool Walking::IsRunning()
{
    return real_Running;
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
    return mag * sin(2 * PI / period * time - period_shift) + mag_shift;
}

bool Walking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
    Matrix3D Tad, Tda, Tcd, Tdc, Tac;
    Vector3D vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;


    Tad.SetTransform(Point3D(x, y, z - LEG_LENGTH), Vector3D(a * 180.0 / PI, b * 180.0 / PI, c * 180.0 / PI));

    vec.X = x + Tad.m[2] * ANKLE_LENGTH;
    vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
    vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.Length();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if(isnan(_Acos) == 1)
        return false;
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = Tad;
    if(Tda.Inverse() == false)
        return false;
    _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if(_m > 1.0)
        _m = 1.0;
    else if(_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if(isnan(_Acos) == 1)
        return false;
    if(Tda.m[7] < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
    Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
    Tdc = Tcd;
    if(Tdc.Inverse() == false)
        return false;
    Tac = Tad * Tdc;
    _Atan = atan2(-Tac.m[1] , Tac.m[5]);
    if(isinf(_Atan) == 1)
        return false;
    *(out) = _Atan;

    // Get Hip Roll
    _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    if(isinf(_Atan) == 1)
        return false;
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
    _m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
    _n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if(isinf(_Atan) == 1)
        return false;
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
}

double Walking::MX28_Angle2Value(double angle) { return angle*MX28_RATIO_ANGLE2VALUE+MX28_CENTER_VALUE; }
double Walking::MX28_Value2Angle(double value) { return (value-MX28_CENTER_VALUE)*MX28_RATIO_VALUE2ANGLE; }

void Walking::SetIdAndAngle(int index, double value)
{
    if(value < MX28_MIN_VALUE)
        value = MX28_MIN_VALUE;
    else if(value > MX28_MAX_VALUE)
        value = MX28_MAX_VALUE;
    WalkAngle[index] = MX28_Value2Angle(value);
}

void Walking::Process()
{
    double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    double pelvis_offset_r, pelvis_offset_l;
    double angle[16], ep[12];
    double offset;
    //double TIME_UNIT = MotionModule::TIME_UNIT;
    //                        60         58          56         54         52              50         61            59        57         55        53             51
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING, R_ELBOW, L_ELBOW
    double dir[16]       = {     1,         1,          1,         1,          1,            1,           1,            1,          1,       1,         1,           1,            1,           -1,           1,      1 };
    double initAngle[16] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,          0.0,        0.0,     0.0,       0.0,         0.0,          0.0,          0.0,         0.0,    0.0 };
    double outValue[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if(WalkTime==0)
    {
        WalkTime = 1;
        m_Time = 0;
        z_Move_Amplitude = 0;
        z_Move_Amplitude_Shift = z_Move_Amplitude / 2;
    }
    // Update walk parameters
    if(m_Time == 0.0)
    {
        update_param_time();
        update_param_move();
        if(ctrl_Running == false)
        {
            if(x_Move_Amplitude == 0.0 && y_Move_Amplitude == 0.0 && a_Move_Amplitude == 0.0)
            {
                real_Running = false;
            }
            else
            {
                x_Move_Amplitude = 0;                                                              // 振幅
                y_Move_Amplitude = 0;
                a_Move_Amplitude = 0;
            }
        }
    }
    else if(m_Time >= (phase_Time1 - time_Msec/2) && m_Time < (phase_Time1 + time_Msec/2))
    {
        update_param_move();
    }
    else if(m_Time >= (phase_Time2 - time_Msec/2) && m_Time < (phase_Time2 + time_Msec/2))
    {
        update_param_time();
        m_Time = phase_Time2;
        if(ctrl_Running == false)
        {
            if(x_Move_Amplitude == 0.0 && y_Move_Amplitude == 0.0 && a_Move_Amplitude == 0.0)
            {
                real_Running = false;
            }
            else
            {
                x_Move_Amplitude = 0;
                y_Move_Amplitude = 0;
                a_Move_Amplitude = 0;
            }
        }
    }
    else if(m_Time >= (phase_Time3 - time_Msec/2) && m_Time < (phase_Time3 + time_Msec/2))
    {
        update_param_move();
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_Time, x_Swap_PeriodTime, x_Swap_Phase_Shift, x_Swap_Amplitude, x_Swap_Amplitude_Shift);
    y_swap = wsin(m_Time, y_Swap_PeriodTime, y_Swap_Phase_Shift, y_Swap_Amplitude, y_Swap_Amplitude_Shift);
    z_swap = wsin(m_Time, z_Swap_PeriodTime, z_Swap_Phase_Shift, z_Swap_Amplitude, z_Swap_Amplitude_Shift);
    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    pelvis_offset_l = wsin(m_Time, y_Swap_PeriodTime, y_Swap_Phase_Shift, -pelvis_Offset, 0);
    pelvis_offset_r = wsin(m_Time, y_Swap_PeriodTime, y_Swap_Phase_Shift, pelvis_Swing, 0);
    
    if(m_Time <= ssp_Time_Start_L)
    {
        x_move_l = wsin(ssp_Time_Start_L, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, x_Move_Amplitude, x_Move_Amplitude_Shift);
        y_move_l = wsin(ssp_Time_Start_L, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, y_Move_Amplitude, y_Move_Amplitude_Shift);
        z_move_l = wsin(ssp_Time_Start_L, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_L, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_l = wsin(ssp_Time_Start_L, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, a_Move_Amplitude, a_Move_Amplitude_Shift);
        x_move_r = wsin(ssp_Time_Start_L, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, -x_Move_Amplitude, -x_Move_Amplitude_Shift);
        y_move_r = wsin(ssp_Time_Start_L, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, -y_Move_Amplitude, -y_Move_Amplitude_Shift);
        z_move_r = wsin(ssp_Time_Start_R, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_R, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_r = wsin(ssp_Time_Start_L, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, -a_Move_Amplitude, -a_Move_Amplitude_Shift);

        //      pelvis_offset_l = 0;
        //      pelvis_offset_r = 0;
    }
    else if(m_Time <= ssp_Time_End_L)
    {
        x_move_l = wsin(m_Time, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, x_Move_Amplitude, x_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, y_Move_Amplitude, y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_L, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, a_Move_Amplitude, a_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, -x_Move_Amplitude, -x_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, -y_Move_Amplitude, -y_Move_Amplitude_Shift);
        z_move_r = wsin(ssp_Time_Start_R, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_R, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, -a_Move_Amplitude, -a_Move_Amplitude_Shift);
        //        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, 0);
        //        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, 0);

    }
    else if(m_Time <= ssp_Time_Start_R)
    {
        x_move_l = wsin(ssp_Time_End_L, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, x_Move_Amplitude, x_Move_Amplitude_Shift);
        y_move_l = wsin(ssp_Time_End_L, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, y_Move_Amplitude, y_Move_Amplitude_Shift);
        z_move_l = wsin(ssp_Time_End_L, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_L, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_l = wsin(ssp_Time_End_L, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, a_Move_Amplitude, a_Move_Amplitude_Shift);
        x_move_r = wsin(ssp_Time_End_L, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_L, -x_Move_Amplitude, -x_Move_Amplitude_Shift);
        y_move_r = wsin(ssp_Time_End_L, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_L, -y_Move_Amplitude, -y_Move_Amplitude_Shift);
        z_move_r = wsin(ssp_Time_Start_R, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_R, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_r = wsin(ssp_Time_End_L, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_L, -a_Move_Amplitude, -a_Move_Amplitude_Shift);
        //        pelvis_offset_l = 0;
        //        pelvis_offset_r = 0;
    }
    else if(m_Time <= ssp_Time_End_R)
    {
        x_move_l = wsin(m_Time, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_R + PI, x_Move_Amplitude, x_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_R + PI, y_Move_Amplitude, y_Move_Amplitude_Shift);
        z_move_l = wsin(ssp_Time_End_L, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_L, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_R + PI, a_Move_Amplitude, a_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_R + PI, -x_Move_Amplitude, -x_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_R + PI, -y_Move_Amplitude, -y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_R, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_R + PI, -a_Move_Amplitude, -a_Move_Amplitude_Shift);
        //        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R,  m_Pelvis_Swing / 2, 0);
        //        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R,-m_Pelvis_Offset / 2, 0);
    }
    else
    {
        x_move_l = wsin(ssp_Time_End_R, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_R + PI, x_Move_Amplitude, x_Move_Amplitude_Shift);
        y_move_l = wsin(ssp_Time_End_R, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_R + PI, y_Move_Amplitude, y_Move_Amplitude_Shift);
        z_move_l = wsin(ssp_Time_End_L, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_L, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_l = wsin(ssp_Time_End_R, a_Move_PeriodTime, a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_R + PI, a_Move_Amplitude, a_Move_Amplitude_Shift);
        x_move_r = wsin(ssp_Time_End_R, x_Move_PeriodTime, x_Move_Phase_Shift + 2 * PI / x_Move_PeriodTime * ssp_Time_Start_R + PI, -x_Move_Amplitude, -x_Move_Amplitude_Shift);
        y_move_r = wsin(ssp_Time_End_R, y_Move_PeriodTime, y_Move_Phase_Shift + 2 * PI / y_Move_PeriodTime * ssp_Time_Start_R + PI, -y_Move_Amplitude, -y_Move_Amplitude_Shift);
        z_move_r = wsin(ssp_Time_End_R, z_Move_PeriodTime, z_Move_Phase_Shift + 2 * PI / z_Move_PeriodTime * ssp_Time_Start_R, z_Move_Amplitude, z_Move_Amplitude_Shift);
        c_move_r = wsin(ssp_Time_End_R, a_Move_PeriodTime,a_Move_Phase_Shift + 2 * PI / a_Move_PeriodTime * ssp_Time_Start_R + PI, -a_Move_Amplitude, -a_Move_Amplitude_Shift);
        //        pelvis_offset_l = 0;
        //        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;
    
    ep[0] = x_swap + x_move_r + x_Offset;
    ep[1] = y_swap + y_move_r - y_Offset / 2;
    ep[2] = z_swap + z_move_r + z_Offset;
    ep[3] = a_swap + a_move_r - r_Offset / 2;
    ep[4] = b_swap + b_move_r + p_Offset;
    ep[5] = c_swap + c_move_r - a_Offset / 2;
    ep[6] = x_swap + x_move_l + x_Offset;
    ep[7] = y_swap + y_move_l + y_Offset / 2;
    ep[8] = z_swap + z_move_l + z_Offset;
    ep[9] = a_swap + a_move_l + r_Offset / 2;
    ep[10] = b_swap + b_move_l + p_Offset;
    ep[11] = c_swap + c_move_l + a_Offset / 2;

    // Compute arm swing
    if(x_Move_Amplitude == 0.0)
    {
        angle[R_ARM_SWING] = 0;                                                                             // Right
        angle[L_ARM_SWING] = 0;                                                                             // Left

        angle[R_ELBOW] = 0;                                                                             // Right elbow
        angle[L_ELBOW] = 0;                                                                             // Left  elbow
    }
    else
    {
        angle[R_ARM_SWING] = wsin(m_Time, periodTime, PI * 1.5, -x_Move_Amplitude * arm_Swing_Gain, 0);
        angle[L_ARM_SWING] = wsin(m_Time, periodTime, PI * 1.5, x_Move_Amplitude * arm_Swing_Gain, 0);

        angle[R_ELBOW] = 0;                                                                            // Right elbow
        angle[L_ELBOW] = 0;                                                                            // Left  elbow
        //        angle[14] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        //        angle[15] = wsin(m_Time, m_PeriodTime, PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    // Compute angles
    if((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
            && (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1))
    {
        for(int i=0; i<12; i++)
            angle[i] *= 180.0 / PI;
    }
    else
    {
        return; // Do not use angle;
    }
    state = 0;
    for(int i=0; i<16; i++)                                                                        // Compute motor value

    {
        offset = double(dir[i] * angle[i] * MX28_RATIO_ANGLE2VALUE);
        if(m_Time <= ssp_Time_Start_L)                                                             // 双脚着地 调整重心到右腿
        {
            if(i == R_HIP_ROLL)  offset -= double(dir[i] * pelvis_offset_r*1);                                      //1.0
            else if(i == L_HIP_ROLL) offset += double(dir[i] * pelvis_offset_l*1);                                      //0.8
            else if(i == R_ANKLE_ROLL) offset -= double(dir[i] * pelvis_offset_r*Pelvis_offset_lr);                       //1.26
            else if(i == L_ANKLE_ROLL) offset += double(dir[i] * pelvis_offset_l*Pelvis_offset_lr);                       //1.26
            else if(i == R_HIP_PITCH || i == L_HIP_PITCH) offset -= double(dir[i] * hip_Pitch_Offset);
            step = 0;
            if(last_step != step)
            {
                state = 1;
                std::cout << std::endl << std::fixed << m_Time << " 0: 双脚着地 调整重心到右腿" ;
            }
        }
        else if(m_Time <= ssp_Time_End_L && m_Time > ssp_Time_Start_L)                             // 迈左腿
        {
            if(i == R_HIP_ROLL) offset -= double(dir[i] * pelvis_offset_r*2);//  0.2
            else if(i == L_HIP_ROLL) offset += double(dir[i] * pelvis_offset_l*2);//  1.2
            else if(i == R_ANKLE_ROLL) offset -= double(dir[i] * pelvis_offset_r*Pelvis_offset_r);//  1.5   Pelvis_offset_r
            else if(i == L_ANKLE_ROLL) offset += double(dir[i] * pelvis_offset_l*Pelvis_offset_r);//  0.2
            else if(i == R_HIP_PITCH || i == L_HIP_PITCH) offset -= double(dir[i] * hip_Pitch_Offset);
            else if(i == L_HIP_YAW)
            {
                if (WalkLineSype == 1 && m_Time>((ssp_Time_End_L-ssp_Time_Start_L)/2+ssp_Time_Start_L)) // 左转
                {
                    offset = -1*HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;
                }
            }
            step = 1;
            if(last_step != step)
            {
                 state = 2;
                std::cout << std::endl << std::fixed << m_Time << " 1: 迈左腿";
            }
        }
        else if(m_Time <= ssp_Time_Start_R)                                                        // 双脚着地 调整重心到左腿
        {
            if(i == R_HIP_ROLL) offset -= double(dir[i] * pelvis_offset_r*1);                      //1
            else if(i == L_HIP_ROLL) offset += double(dir[i] * pelvis_offset_l*1);                 //1
            else if(i == R_ANKLE_ROLL) offset -= double(dir[i] * pelvis_offset_r*Pelvis_offset_lr);//1.26
            else if(i == L_ANKLE_ROLL) offset += double(dir[i] * pelvis_offset_l*Pelvis_offset_lr);//1.26
            else if(i == R_HIP_PITCH || i == L_HIP_PITCH) offset -= double(dir[i] * hip_Pitch_Offset);
            else if(i == L_HIP_YAW)
            {
                if (WalkLineSype == 1  && m_Time>((ssp_Time_Start_R-ssp_Time_End_L)/2+ssp_Time_End_L)) // 左转
                {
                    offset = -1*HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;
                }
            }
            step = 2;
            if(last_step != step)
            {
                state = 3;
                std::cout << std::endl << std::fixed << m_Time  << " 2:双脚着地 调整重心到左腿";
            }
        }
        else if(m_Time <= ssp_Time_End_R && m_Time > ssp_Time_Start_R)                             // 迈右腿
        {
            if(i == R_HIP_ROLL) offset -= double(dir[i] * pelvis_offset_r*2);                      //     1.2
            else if(i == L_HIP_ROLL) offset += double(dir[i] * pelvis_offset_l*2);                 //     0.2
            else if(i == R_ANKLE_ROLL) offset -= double(dir[i] * pelvis_offset_r*Pelvis_offset_l); //     0.2
            else if(i == L_ANKLE_ROLL) offset += double(dir[i] * pelvis_offset_l*Pelvis_offset_l); //     1.4  Pelvis_offset_l
            else if(i == R_HIP_PITCH || i == L_HIP_PITCH)
                offset -= double(dir[i] * hip_Pitch_Offset);
            else if(i == R_HIP_YAW)
            {
                if (WalkLineSype == 2 && m_Time>((ssp_Time_End_R-ssp_Time_Start_R)/2+ssp_Time_Start_R)) // 右转
                {
                    offset = HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;
                }
            }
            step = 3;
            if(last_step != step)
            {
                 state = 4;
                std::cout << std::endl << std::fixed << m_Time << " 3:迈右腿";
            }
        }
        else
        {
            if(i == R_HIP_ROLL) offset -= double(dir[i] * pelvis_offset_r);
            else if(i == L_HIP_ROLL) offset += double(dir[i] * pelvis_offset_l);
            else if(i == R_ANKLE_ROLL) offset -= double(dir[i] * pelvis_offset_r);
            else if(i == L_ANKLE_ROLL) offset += double(dir[i] * pelvis_offset_l);
            else if(i == R_HIP_PITCH || i == 8) offset -= double(dir[i] * hip_Pitch_Offset);
            else if(i == R_HIP_YAW)
            {
                if (WalkLineSype == 2 && m_Time<((periodTime-ssp_Time_End_R)/2+ssp_Time_End_R))    // 右转
                {
                    offset = HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;
                }
            }
            step = 4;
            if(last_step != step)
            {
                // state = 5;
                std::cout << std::endl << std::fixed << m_Time << " 4:不知道";
            }
        }
        last_step = step;
        outValue[i] = MX28_Angle2Value(initAngle[i]) + offset;
    }

    // adjust balance offset
    if(balance_Enable == true)
    {
        double rlGyroErr = g_rl_gyro;                                                              //  初始位置 0
        double fbGyroErr = g_fb_gyro ;                                                             //  初始位置90

        outValue[R_HIP_ROLL] -= (dir[R_HIP_ROLL] * rlGyroErr * balance_Hip_Roll_Gain);                               // R_HIP_ROLL
        outValue[R_KNEE] -= (dir[R_KNEE] * fbGyroErr * balance_Knee_Gain);                                   // R_KNEE
        outValue[R_ANKLE_PITCH] -= (dir[R_ANKLE_PITCH] * fbGyroErr * balance_Ankle_Pitch_Gain);                            // R_ANKLE_PITCH
        outValue[R_ANKLE_ROLL] -= (dir[R_ANKLE_ROLL] * rlGyroErr * balance_Ankle_Roll_Gain);                             // R_ANKLE_ROLL

        outValue[L_HIP_ROLL] -= (dir[L_HIP_ROLL] * rlGyroErr * balance_Hip_Roll_Gain);                               // L_HIP_ROLL
        outValue[L_KNEE] -= (dir[L_KNEE] * fbGyroErr * balance_Knee_Gain);                                   // L_KNEE
        outValue[L_ANKLE_PITCH] -= (dir[L_ANKLE_PITCH] * fbGyroErr * balance_Ankle_Pitch_Gain);                          // L_ANKLE_PITCH
        outValue[L_ANKLE_ROLL] -= (dir[L_ANKLE_ROLL] * rlGyroErr * balance_Ankle_Roll_Gain);                           // L_ANKLE_ROLL

    }

    SetIdAndAngle(RHipYaw_Index,        outValue[R_HIP_YAW]);
    SetIdAndAngle(RHipRoll_Index,       outValue[R_HIP_ROLL]);
    SetIdAndAngle(RHipPitch_Index,      outValue[R_HIP_PITCH]);
    SetIdAndAngle(RKneePitch_Index,     outValue[R_KNEE]);
    SetIdAndAngle(RAnklePitch_Index,    outValue[R_ANKLE_PITCH]);
    SetIdAndAngle(RAnkleRoll_Index,     outValue[R_ANKLE_ROLL]);

    SetIdAndAngle(LHipYaw_Index,        outValue[L_HIP_YAW]);
    SetIdAndAngle(LHipRoll_Index,       outValue[L_HIP_ROLL]);
    SetIdAndAngle(LHipPitch_Index,      outValue[L_HIP_PITCH]);
    SetIdAndAngle(LKneePitch_Index,     outValue[L_KNEE]);
    SetIdAndAngle(LAnklePitch_Index,    outValue[L_ANKLE_PITCH]);
    SetIdAndAngle(LAnkleRoll_Index,     outValue[L_ANKLE_ROLL]);
    
    SetIdAndAngle(RShoulderPitch_Index, outValue[R_ARM_SWING]);
    SetIdAndAngle(LShoulderPitch_Index, outValue[L_ARM_SWING]);

    SetIdAndAngle(RElbowRoll_Index,     outValue[R_ELBOW]);
    SetIdAndAngle(LElbowRoll_Index,     outValue[L_ELBOW]);
    
    if(real_Running == true)
    {
        m_Time += time_Msec;
        if(m_Time >= periodTime) m_Time = 0;
    }
}


#endif 

