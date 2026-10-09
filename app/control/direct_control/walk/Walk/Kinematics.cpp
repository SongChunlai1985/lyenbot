/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */
#ifdef BALANCE_FLAG
#include <math.h>
#include "Kinematics.h"

//using namespace Robot;

const double Kinematics::CAMERA_DISTANCE = 33.2; //mm // x轴 摄像头与质心上半身质心的距离
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree

const double Kinematics::LEG_SIDE_OFFSET = 50; //mm  //37.0(darwin);质心与腿之间距离 y轴
const double Kinematics::THIGH_LENGTH = 115; //mm //93.0(darwin); 大腿长度 z轴
const double Kinematics::CALF_LENGTH = 110; //mm  //93.0(darwin); 小腿长度 z轴
const double Kinematics::ANKLE_LENGTH = 52; //mm  //33.5(darwin);脚底与踝关节距离   z轴    46 + 5.5 增加橡胶鞋垫5.5
const double Kinematics::LEG_LENGTH = 277; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)



//Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}
#endif
