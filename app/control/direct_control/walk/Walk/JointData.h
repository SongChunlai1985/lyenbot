/*
 *   JointData.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _JOINT_DATA_H_
#define _JOINT_DATA_H_


#ifdef MX28_1024
const int MX28_CENTER_VALUE = 0;
const int MX28_MIN_VALUE = -512;
const int MX28_MAX_VALUE = 512;
const double MX28_MIN_ANGLE = -150.0; // degree
const double MX28_MAX_ANGLE = 150.0; // degree
const double MX28_RATIO_VALUE2ANGLE = 0.293; // 300 / 1024
const double MX28_RATIO_ANGLE2VALUE = 3.413; // 1024 / 300

const int MX28_PARAM_BYTES = 5;
#else
const int MX28_CENTER_VALUE = 0;
const int MX28_MIN_VALUE = -2048;
const int MX28_MAX_VALUE = 2048;
const double MX28_MIN_ANGLE = -180.0; // degree
const double MX28_MAX_ANGLE = 180.0; // degree
const double MX28_RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
const double MX28_RATIO_ANGLE2VALUE = 11.378; // 4096 / 360

const int MX28_PARAM_BYTES = 7;
#endif

static int MX28_Angle2Value(double angle) { return (int)(angle*MX28_RATIO_ANGLE2VALUE)+MX28_CENTER_VALUE; }
static double MX28_Value2Angle(int value) { return (double)(value-MX28_CENTER_VALUE)*MX28_RATIO_VALUE2ANGLE; }




//enum
//{
//	SLOPE_HARD			= 16,
//	SLOPE_DEFAULT		= 32,
//	SLOPE_SOFT			= 64,
//	SLOPE_EXTRASOFT		= 128
//};

//enum
//{
//		P_GAIN_DEFAULT      = 32,
//		I_GAIN_DEFAULT      = 0,
//		D_GAIN_DEFAULT      = 0
//};
enum
{
    ID_R_SHOULDER_PITCH     = 0X61,
    ID_L_SHOULDER_PITCH     = 0X81,
    ID_R_SHOULDER_ROLL      = 0X62,
    ID_L_SHOULDER_ROLL      = 0X82,
    ID_R_ELBOW              = 0X64,
    ID_L_ELBOW              = 0X84,      //不知道这个是啥
    ID_R_HIP_YAW            = 0X03,
    ID_L_HIP_YAW            = 0X23,
    ID_R_HIP_ROLL           = 0X02,
    ID_L_HIP_ROLL           = 0X22,
    ID_R_HIP_PITCH          = 0X01,
    ID_L_HIP_PITCH          = 0X21,
    ID_R_KNEE               = 0X04,
    ID_L_KNEE               = 0X24,
    ID_R_ANKLE_PITCH        = 0X05,
    ID_L_ANKLE_PITCH        = 0X25,
    ID_R_ANKLE_ROLL         = 0X06,
    ID_L_ANKLE_ROLL         = 0X26,
    ID_HEAD_PAN             = 19,
    ID_HEAD_TILT            = 20,
    NUMBER_OF_JOINTS
};

		
//namespace Robot
//{	
	class JointData  
	{
	protected:
//		bool m_Enable[NUMBER_OF_JOINTS];
//		int m_Value[NUMBER_OF_JOINTS];
//		double m_Angle[NUMBER_OF_JOINTS];
	
        
    
//		int m_CWSlope[NUMBER_OF_JOINTS];
//		int m_CCWSlope[NUMBER_OF_JOINTS];
//		int m_PGain[NUMBER_OF_JOINTS];
//        int m_IGain[NUMBER_OF_JOINTS];
//        int m_DGain[NUMBER_OF_JOINTS];

	public:
		JointData();
		~JointData();
        short WalkAngle[NUMBER_OF_JOINTS];
		unsigned char WalkID[NUMBER_OF_JOINTS]; 
    
    
//		void SetValue(int id, int value);
//		int GetValue(int id);
//	
//		void SetAngle(int id, double angle);
//		double GetAngle(int id);
	
        void SetIdAndAngle(int index, int id, short value);
            
//        void SetEnable(int id, bool enable);
//		void SetEnable(int id, bool enable, bool exclusive);
//		void SetEnableHeadOnly(bool enable);
//        void SetEnableHeadOnly(bool enable, bool exclusive);
//		void SetEnableRightArmOnly(bool enable);
//        void SetEnableRightArmOnly(bool enable, bool exclusive);
//		void SetEnableLeftArmOnly(bool enable);
//        void SetEnableLeftArmOnly(bool enable, bool exclusive);
//		void SetEnableRightLegOnly(bool enable);
//        void SetEnableRightLegOnly(bool enable, bool exclusive);
//		void SetEnableLeftLegOnly(bool enable);
//        void SetEnableLeftLegOnly(bool enable, bool exclusive);
//		void SetEnableUpperBodyWithoutHead(bool enable);
//        void SetEnableUpperBodyWithoutHead(bool enable, bool exclusive);
//		void SetEnableLowerBody(bool enable);
//        void SetEnableLowerBody(bool enable, bool exclusive);
//		void SetEnableBodyWithoutHead(bool enable);
//        void SetEnableBodyWithoutHead(bool enable, bool exclusive);
//		void SetEnableBody(bool enable);
//        void SetEnableBody(bool enable, bool exclusive);
//		bool GetEnable(int id);

//		void SetRadian(int id, double radian);
//		double GetRadian(int id);

//		void SetSlope(int id, int cwSlope, int ccwSlope);
//		void SetCWSlope(int id, int cwSlope);
//		int  GetCWSlope(int id);
//		void SetCCWSlope(int id, int ccwSlope);
//		int  GetCCWSlope(int id);

//		void SetPGain(int id, int pgain) { m_PGain[id] = pgain; }
//		int  GetPGain(int id)            { return m_PGain[id]; }
//		void SetIGain(int id, int igain) { m_IGain[id] = igain; }
//		int  GetIGain(int id)            { return m_IGain[id]; }
//		void SetDGain(int id, int dgain) { m_DGain[id] = dgain; }
//		int  GetDGain(int id)            { return m_DGain[id]; }
	};
//}

#endif
