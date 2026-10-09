/*
 *   MotionStatus.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_STATUS_H_
#define _MOTION_STATUS_H_

#include "JointData.h"

//namespace Robot
//{
    enum {
        BACKWARD    = -1,
        STANDUP     = 0,
        FORWARD     = 1
    };

	class MotionStatus
	{
	private:

	public:
//	    static const int FALLEN_F_LIMIT     = 390;
//	    static const int FALLEN_B_LIMIT     = 580;
//	    static const int FALLEN_MAX_COUNT   = 30;

		static JointData m_CurrentJoints;
		static float FB_GYRO;
		static float RL_GYRO;
//		static float FB_ACCEL;
//		static float RL_ACCEL;

//		static int BUTTON;
//		static int FALLEN;
	};
//}

#endif
