
#ifdef BALANCE_FLAG
#include <stdio.h>
#include <math.h>
#include "Vector.h"
#include "Matrix.h"
#include "MX28.h"
#include "Walking.h"


//#include "MotionProcess.h"

#define PI (3.14159265)
#define HIP_YAW_OFFSET_LIMIT  30   // 226  34b  6d6
/*
//#define	DSP_RATIO      0.2  //0.1  // r ˫��֧��ʱ��ռ�������ڱ���
//#define	STEP_FB_RATIO  0.38  //0.28 // step_forward_back_ratio

//#define	Y_SWAP_AMPLITUDE  (20) //20  Խ�� ����ƫ��Խ��
//#define	Z_SWAP_AMPLITUDE  6 //  pbz  5  darwin-op 45.4cm   // ����������1%����
////#define	PELVIS_OFFSET     (11)//10  Խ�� �ż��Խ��
////#define	PELVIS_OFFSET_2   (13)//10  Խ�� �ż��Խ��
//#define	ARM_SWING_GAIN    1.2//1.5



//#define	BALANCE_ANKLE_ROLL_GAIN    0//-1 //-0.5
//#define	BALANCE_ANKLE_PITCH_GAIN   0//0.4   //1.2
//#define	BALANCE_KNEE_GAIN          0//0.2   //0.6
//#define	BALANCE_HIP_ROLL_GAIN      0//-0.5 //0.5
////#define	BALANCE_HIP_PITCH_GAIN     0.4
//#define	HIP_PITCH_OFFSET           -2    //����ǰ��



//??#define m_X_Swap_Phase_Shift       PI             // -det_bx
//#define m_X_Swap_Amplitude_Shift   0              // ubx
//#define m_X_Move_Phase_Shift       (PI / 2)       // -det_mx
//#define m_X_Move_Amplitude_Shift   0              // umx
//#define m_Y_Swap_Phase_Shift       0              // -det_by
//#define m_Y_Swap_Amplitude_Shift   0              // uby
//#define m_Y_Move_Phase_Shift       (PI / 2)       // -det_my
//#define m_Z_Swap_Phase_Shift       (PI * 3 / 2)   // -det_bz
//#define m_Z_Move_Phase_Shift       (PI / 2)       // -det_mz
//??#define m_A_Move_Phase_Shift       (PI / 2)


//#define	P_GAIN    P_GAIN_DEFAULT
//#define I_GAIN    I_GAIN_DEFAULT
//#define D_GAIN    D_GAIN_DEFAULT

//#define m_PeriodTime PERIOD_TIME
//?? #define m_DSP_Ratio  DSP_RATIO
//?? #define m_SSP_Ratio  (1 - DSP_RATIO)

//#define m_Pelvis_Offset    (PELVIS_OFFSET*MX28_RATIO_ANGLE2VALUE)
//#define m_Pelvis_Swing     (m_Pelvis_Offset*0.96)//0.35  ������ƫ��  0.78
//#define m_Arm_Swing_Gain   ARM_SWING_GAIN
//#define m_Hip_Pitch_Offset (HIP_PITCH_OFFSET*MX28_RATIO_ANGLE2VALUE)


//float m_Pelvis_Offset ;//   (PELVIS_OFFSET*MX28_RATIO_ANGLE2VALUE)
//float m_Pelvis_Swing  ;//   (m_Pelvis_Offset*0.96)//0.35  ������ƫ��  0.78

*/
extern float g_fb_gyro;
extern float g_rl_gyro;
extern int hip_offset, knee_offset, ankle_offset;


float Pelvis_offset_l = 1;
float Pelvis_offset_r = 1;
float Pelvis_offset_lr = 1;




#ifdef __cplusplus  
extern "C"
{  
#endif
void SetWalkingStepSize(float step,float stepHeight);
void walking_process(uint8_t walk_line);
void WalkingWalkStateChange(void);
void Motor_SetPosition(uint8_t* motor_id, short* motor_pos, uint8_t motor_num);
void walking_init(void);
void Set_XYZ_Offset (float x_offset, float y_offset, float z_offset);
uint8_t GetWorkingState(void);
void WalkTimeInitialize(void);       
void SetWalkingTime(float time);  
void SetPelvisOffsetSwing(float offset);
    
#ifdef __cplusplus  
}
#endif


Walking walking;


uint8_t WalkTime = 0;

void WalkTimeInitialize(void)
{
    WalkTime = 0;
}

/*
offset��Խ�� �ż��Խ��  
ratio��������ƫ�� 
*/
void SetPelvisOffsetSwing(float offset)
{
//    walking.pelvis_Offset = offset*MX28_RATIO_ANGLE2VALUE;//   Խ�� �ż��Խ��  
//    walking.pelvis_Swing = walking.pelvis_Offset*0.4;  //0.35  ������ƫ��  0.78
}

/*
step: ����
stepHeight��̧�ȸ߶�
*/
void SetWalkingStepSize(float step,float stepHeight)
{
    walking.init_x_Move_Amplitude = step;
    walking.init_z_Move_Amplitude  = stepHeight;
}

/*
x_offset ����������ǰ��ķ���  ��СС��ǰ��
y_offset �������������һζ��ķ���  ��������ƫ��
z_offset �����������¶׵ķ��� �����¶׷���Խ��
*/
void Set_XYZ_Offset (float x_offset, float y_offset, float z_offset)
{
//    walking.init_x_offset = x_offset;
//    walking.init_y_offset = y_offset;
//    walking.init_z_offset = z_offset;	
}

void SetWalkingTime(float time)
{
//    walking.time_Msec = time;
}

uint8_t WalkLineSype = 0; //0ֱ��   1��ת   2��ת  3�����   4�Һ���
void walking_process(uint8_t walk_line)
{
    WalkLineSype = walk_line;
	walking.Process();
}

void walking_init(void)
{
    walking.Initialize();
   
    walking.Start();
}


Walking::Walking()
{
}

Walking::~Walking()
{
}

void Walking::Initialize()
{
	periodTime = 1200;//600     // Ts  1200
    dsp_Ratio = 0.4;  //0.1  // r ˫��֧��ʱ��ռ�������ڱ���
    ssp_Ratio = 1 - dsp_Ratio;
    step_fb_Ratio = 0.28;  //0.28 // step_forward_back_ratio
    time_Msec = 10;
    
    init_x_Move_Amplitude = FORWARD_STEP_SIZE;//30  // pmx  ����
    init_y_Move_Amplitude = 0;       // pmy  ����Xǰ�� Y��Ϊ0
    init_z_Move_Amplitude = STEP_HEIGHT_20;//50  // pmz  ̧�Ÿ߶�
    
    balance_Ankle_Roll_Gain = 0;// -1 // -0.5   10
    balance_Ankle_Pitch_Gain = 0;// 0.4   //1.2
    balance_Knee_Gain = 0;// 0.2   //0.6
    balance_Hip_Roll_Gain = 0;// -0.5 //0.5

    init_y_Swap_Amplitude = 20; // 20  Խ�� ����ƫ��Խ��
    init_z_Swap_Amplitude = 6; //  pbz ����������1%����
    arm_Swing_Gain = 1.2;//// 1.5
    
    pelvis_Offset = 10*MX28_RATIO_ANGLE2VALUE; // 10  Խ�� �ż��Խ��   
    pelvis_Swing = pelvis_Offset*1;  // 0.35  ������ƫ��  0.78
    
    init_x_offset = -18;	// ����������ǰ��ķ���  ��СС��ǰ��-30
	init_y_offset = 30;	// �������������һζ��ķ���  ��������ƫ��30
	init_z_offset = 30;  // �����������¶׵ķ��� �����¶׷���Խ��30
	init_r_offset = 0;
	init_p_offset = 0;
	init_a_offset = 0;
    hip_Pitch_Offset = 0.8*MX28_RATIO_ANGLE2VALUE;    //����ǰ��   1.8
    
 	init_a_Move_Amplitude = 0;	
	a_Move_Aim_On = false;
	balance_Enable = true;
    
    x_Swap_Phase_Shift   =    PI;             // -det_bx
    x_Swap_Amplitude_Shift =  0 ;             // ubx
    x_Move_Phase_Shift =      (PI / 2);       // -det_mx
    x_Move_Amplitude_Shift =  0;            // umx
    y_Swap_Phase_Shift =      0;              // -det_by
    y_Swap_Amplitude_Shift =  0;              // uby
    y_Move_Phase_Shift =      (PI / 2);       // -det_my
    z_Swap_Phase_Shift =      (PI * 3 / 2);   // -det_bz
    z_Move_Phase_Shift =      (PI / 2);       // -det_mz
    a_Move_Phase_Shift =      (PI / 2);

    ctrl_Running = false;
	real_Running = false;
	m_Time = 0;
    
//	update_param_time();
//	update_param_move();
}


void Walking::update_param_time()
{
    x_Swap_PeriodTime = periodTime / 2;             // 2pi/wbx
    x_Move_PeriodTime = periodTime * ssp_Ratio;   // 2pi/wbm
    y_Swap_PeriodTime = periodTime;                 // 2pi/wby
    y_Move_PeriodTime = periodTime * ssp_Ratio;   // 2pi/wmy
    z_Swap_PeriodTime = periodTime / 2;             // 2pi/wbz
    z_Move_PeriodTime = periodTime * ssp_Ratio / 2;   // 2pi/wmz       
    a_Move_PeriodTime = periodTime * ssp_Ratio;

    ssp_Time = periodTime * ssp_Ratio;
    ssp_Time_Start_L = (1 - ssp_Ratio) * periodTime / 4;
    ssp_Time_End_L = (1 + ssp_Ratio) * periodTime / 4;
    ssp_Time_Start_R = (3 - ssp_Ratio) * periodTime / 4;
    ssp_Time_End_R = (3 + ssp_Ratio) * periodTime / 4;

    phase_Time1 = (ssp_Time_End_L + ssp_Time_Start_L) / 2;
    phase_Time2 = (ssp_Time_Start_R + ssp_Time_End_L) / 2;
    phase_Time3 = (ssp_Time_End_R + ssp_Time_Start_R) / 2;

    
    
//    pelvis_Offset = 10*MX28_RATIO_ANGLE2VALUE; //10  Խ�� �ż��Խ��   
//    pelvis_Swing = walking.pelvis_Offset*0.96;  //0.35  ������ƫ��  0.78
//    arm_Swing_Gain = 1.2;//1.5
}

void Walking::update_param_move()
{
	// Forward/Back
    x_Move_Amplitude = init_x_Move_Amplitude;
    x_Swap_Amplitude = x_Move_Amplitude * step_fb_Ratio;   // pbx

    // Right/Left
    y_Move_Amplitude = init_y_Move_Amplitude / 2;               // pmy
    if(y_Move_Amplitude > 0)
        y_Move_Amplitude_Shift = y_Move_Amplitude;
    else
        y_Move_Amplitude_Shift = -y_Move_Amplitude;
    y_Swap_Amplitude = init_y_Swap_Amplitude + y_Move_Amplitude_Shift * 0.04f;   // pby

    z_Move_Amplitude = init_z_Move_Amplitude / 2;             // pmz
    z_Move_Amplitude_Shift = z_Move_Amplitude / 2;
    z_Swap_Amplitude = init_z_Swap_Amplitude;                 // pbz
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
    r_Offset = init_r_offset * PI / 180.0f;
    p_Offset = init_p_offset * PI / 180.0f;
    a_Offset = init_a_offset * PI / 180.0f;
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
    int dir[16]         = {    1,        1,          1,          -1,        -1,           -1,           1,            1,        -1,       1,         1,          -1,            1,           -1,            1,      1 };
    double initAngle[16] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,          0.0,        0.0,     0.0,       0.0,         0.0,          0.0,          0.0,         0.0,    0.0 };
    int outValue[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    
    
    if(WalkTime==0)
    {
        WalkTime = 1;
        m_Time = 0;  
        z_Move_Amplitude = 0;             
        z_Move_Amplitude_Shift = z_Move_Amplitude / 2;        
    }
    // Update walk parameters
    if(m_Time == 0)
    {
        update_param_time();
//        update_param_move();
        if(ctrl_Running == false)
        {
            if(x_Move_Amplitude == 0 && y_Move_Amplitude == 0 && a_Move_Amplitude == 0)
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
            if(x_Move_Amplitude == 0 && y_Move_Amplitude == 0 && a_Move_Amplitude == 0)
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

    pelvis_offset_l = wsin(m_Time, y_Swap_PeriodTime, y_Swap_Phase_Shift, -walking.pelvis_Offset, 0);
    pelvis_offset_r = wsin(m_Time, y_Swap_PeriodTime, y_Swap_Phase_Shift, walking.pelvis_Swing, 0);
    
    if( GetWorkingState()==0 && m_Time >= (ssp_Time_Start_R - time_Msec))
    {
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        WalkingWalkStateChange();
        m_Time = 0;
    }
    
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
    if(x_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
        //add elbow
        angle[14] = 0; // Right elbow
        angle[15] = 0; // Left  elbow
    }
    else
    {
        angle[12] = wsin(m_Time, periodTime, PI * 1.5, -x_Move_Amplitude * arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, periodTime, PI * 1.5, x_Move_Amplitude * arm_Swing_Gain, 0);
        //add elbow
        angle[14] = 0; // Right elbow
        angle[15] = 0; // Left  elbow
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


    // Compute motor value
    for(int i=0; i<16; i++)
    {
		offset = (double)dir[i] * angle[i] * MX28_RATIO_ANGLE2VALUE;
        if(m_Time <= ssp_Time_Start_L)// ˫���ŵ� �������ĵ�����
        {
            if(i == 1) // R_HIP_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*1;//1.0
            else if(i == 7) // L_HIP_ROLL
                offset += (double)dir[i] * pelvis_offset_l*1;//0.8
            else if(i == 5) // R_ANKLE_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*Pelvis_offset_lr;//1.26
            else if(i == 11) // L_ANKLE_ROLL
                offset += (double)dir[i] * pelvis_offset_l*Pelvis_offset_lr;//1.26
            else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
                offset -= (double)dir[i] * hip_Pitch_Offset;
        }
        else if(m_Time <= ssp_Time_End_L && m_Time > ssp_Time_Start_L)// ������
        {
            if(i == 1) // R_HIP_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*0.2;//  0.2
            else if(i == 7) // L_HIP_ROLL
                offset += (double)dir[i] * pelvis_offset_l*2;//  1.2
            else if(i == 5) // R_ANKLE_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*Pelvis_offset_r;//  1.5   Pelvis_offset_r
            else if(i == 11) // L_ANKLE_ROLL
                offset += (double)dir[i] * pelvis_offset_l*0.2;//  0.2  
            else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
                offset -= (double)dir[i] * hip_Pitch_Offset;
            else if(i == 6)
            {
                if (WalkLineSype == 1 && m_Time>((ssp_Time_End_L-ssp_Time_Start_L)/2+ssp_Time_Start_L)) // ��ת
                {  
                    offset = -1*HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;  
                }
//                else
//                {
//                    offset = 3 * MX28_RATIO_ANGLE2VALUE; 
//                }
            }
        }
        else if(m_Time <= ssp_Time_Start_R) // ˫���ŵ� �������ĵ�����
        {
            if(i == 1) // R_HIP_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*1;//1
            else if(i == 7) // L_HIP_ROLL
                offset += (double)dir[i] * pelvis_offset_l*1;//1
            else if(i == 5) // R_ANKLE_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*Pelvis_offset_lr;//1.26
            else if(i == 11) // L_ANKLE_ROLL
                offset += (double)dir[i] * pelvis_offset_l*Pelvis_offset_lr;//1.26
            else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
                offset -= (double)dir[i] * hip_Pitch_Offset;
            else if(i == 6)
            {
                if (WalkLineSype == 1  && m_Time>((ssp_Time_Start_R-ssp_Time_End_L)/2+ssp_Time_End_L)) // ��ת
                {  
                    offset = -1*HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;  
                }
//                else
//                {
//                    offset = 3 * MX28_RATIO_ANGLE2VALUE; 
//                }
            }
        }
        else if(m_Time <= ssp_Time_End_R && m_Time > ssp_Time_Start_R)// ������
        {
            if(i == 1) // R_HIP_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*2; //     1.2
            else if(i == 7) // L_HIP_ROLL
                offset += (double)dir[i] * pelvis_offset_l*0.2; //     0.2
            else if(i == 5) // R_ANKLE_ROLL
                offset -= (double)dir[i] * pelvis_offset_r*0.2; //     0.2
            else if(i == 11) // L_ANKLE_ROLL
                offset += (double)dir[i] * pelvis_offset_l*Pelvis_offset_l; //     1.4  Pelvis_offset_l
            else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
                offset -= (double)dir[i] * hip_Pitch_Offset;
            else if(i == 0)
            {
                if (WalkLineSype == 2 && m_Time>((ssp_Time_End_R-ssp_Time_Start_R)/2+ssp_Time_Start_R)) // ��ת
                {  
                    offset = HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;   
                }
//                else
//                {
//                    offset = 5 * MX28_RATIO_ANGLE2VALUE; 
//                }
            }
        }
        else
        {
            if(i == 1) // R_HIP_ROLL
                offset -= (double)dir[i] * pelvis_offset_r;
            else if(i == 7) // L_HIP_ROLL
                offset += (double)dir[i] * pelvis_offset_l*1.2;
            else if(i == 5) // R_ANKLE_ROLL
                offset -= (double)dir[i] * pelvis_offset_r;
            else if(i == 11) // L_ANKLE_ROLL
                offset += (double)dir[i] * pelvis_offset_l;
            else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
                offset -= (double)dir[i] * hip_Pitch_Offset;
            else if(i == 0)
            {
                if (WalkLineSype == 2 && m_Time<((walking.periodTime-ssp_Time_End_R)/2+ssp_Time_End_R)) // ��ת
                {  
                    offset = HIP_YAW_OFFSET_LIMIT * MX28_RATIO_ANGLE2VALUE;   
                }
//                else
//                {
//                    offset = 5 * MX28_RATIO_ANGLE2VALUE; 
//                }
            }
        }   
        
        outValue[i] = MX28_Angle2Value(initAngle[i]) + (int)offset;
    }

    
    // adjust balance offset  
    if(balance_Enable == true)
    {
        float rlGyroErr = g_rl_gyro;//��ʼλ�� 0
        float fbGyroErr = g_fb_gyro ;//  ��ʼλ��90
        /*if(g_fb_gyro>=0)
        {
            fbGyroErr = 90-g_fb_gyro;
        } 
        else
        {
            fbGyroErr = -90-g_fb_gyro;
        }   */ 

        outValue[1] -= (int)(dir[1] * rlGyroErr * balance_Hip_Roll_Gain); // R_HIP_ROLL
        outValue[3] -= (int)(dir[3] * fbGyroErr * balance_Knee_Gain); // R_KNEE
        outValue[4] -= (int)(dir[4] * fbGyroErr * balance_Ankle_Pitch_Gain); // R_ANKLE_PITCH
        outValue[5] -= (int)(dir[5] * rlGyroErr * balance_Ankle_Roll_Gain); // R_ANKLE_ROLL
                  
        outValue[7] -= (int)(dir[7] * rlGyroErr * balance_Hip_Roll_Gain); // L_HIP_ROLL
        outValue[9] -= (int)(dir[9] * fbGyroErr * balance_Knee_Gain); // L_KNEE
        outValue[10] -= (int)(dir[10] * fbGyroErr * balance_Ankle_Pitch_Gain); // L_ANKLE_PITCH      
        outValue[11] -= (int)(dir[11] * rlGyroErr * balance_Ankle_Roll_Gain); // L_ANKLE_ROLL

    }

    m_Joint.SetIdAndAngle(RHipYaw_Index,        RHipYaw_ID,          outValue[0]);
    m_Joint.SetIdAndAngle(RHipRoll_Index,       RHipRoll_ID,         outValue[1]);    
    m_Joint.SetIdAndAngle(RHipPitch_Index,      RHipPitch_ID,        outValue[2]);
    m_Joint.SetIdAndAngle(RKneePitch_Index,     RKneePitch_ID,       outValue[3]);
    m_Joint.SetIdAndAngle(RAnklePitch_Index,    RAnklePitch_ID,      outValue[4]);
    m_Joint.SetIdAndAngle(RAnkleRoll_Index,     RAnkleRoll_ID,       outValue[5]);    
    m_Joint.SetIdAndAngle(LHipYaw_Index,        LHipYaw_ID,          outValue[6]);
    m_Joint.SetIdAndAngle(LHipRoll_Index,       LHipRoll_ID,         outValue[7]);
    m_Joint.SetIdAndAngle(LHipPitch_Index,      LHipPitch_ID,        outValue[8]);
    m_Joint.SetIdAndAngle(LKneePitch_Index,     LKneePitch_ID,       outValue[9]);    
    m_Joint.SetIdAndAngle(LAnklePitch_Index,    LAnklePitch_ID,      outValue[10]);
    m_Joint.SetIdAndAngle(LAnkleRoll_Index,     LAnkleRoll_ID,       outValue[11]);
    
    m_Joint.SetIdAndAngle(RShoulderPitch_Index, RShoulderPitch_ID,   outValue[12]);
    m_Joint.SetIdAndAngle(LShoulderPitch_Index, LShoulderPitch_ID,   outValue[13]); 
    //add elbow
    m_Joint.SetIdAndAngle(RElbowRoll_Index,     RElbowRoll_ID,       outValue[14]);
    m_Joint.SetIdAndAngle(LElbowRoll_Index,     LElbowRoll_ID,       outValue[15]);

	Motor_SetPosition(m_Joint.WalkID,(short *)m_Joint.WalkAngle,16);// 12  ֻ������  16 ���ϸ첲
    
    if(real_Running == true)
    {
        
        m_Time += time_Msec;
        if(m_Time >= periodTime)
            m_Time = 0;
        

        if(m_Time == 0)
        {
            
            WalkingWalkStateChange();
        }
    }

}


#endif 

