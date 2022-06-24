#include "math.h"
#include "sysconfig.h"
#include "Task_StateMachine.h"
#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI/180
#define PI 3.14159267

#define Body_Width 104
#define Body_Length 245
#define Body_Thickness 0
#define Foot_Width 191.8
#define Foot_Length 245

//单位：mm
#define a1 43.9
#define a2 110
#define a3 113

//单位：mm
#define S_ 20   //横移步长/旋转步长
#define S  50   //前进步长
#define H  20   //跨步高度
#define T  35  //步态周期  

#define Beta acos(S_/sqrt(Foot_Width*Foot_Width+Foot_Length*Foot_Length))-atan2(Foot_Width,Foot_Length) //旋转时Y轴与Z轴夹角

//摆动相初始坐标
#define SwayStatus_Xst 150
#define SwayStatus_Yst 43.9
#define SwayStatus_Zst -0.5*S

//支撑相初始坐标
#define SupportingStatus_Xst 150
#define SupportingStatus_Yst 43.9
#define SupportingStatus_Zst 0.5*S


#define Move_to_Left_SwayStatus_Yst        SwayStatus_Yst-0.5*S_
#define Move_to_Left_SupportingStatus_Yst  SupportingStatus_Yst+0.5*S_
#define Move_to_Right_SwayStatus_Yst       SwayStatus_Yst+0.5*S_
#define Move_to_Right_SupportingStatus_Yst SupportingStatus_Yst-0.5*S_
//三维坐标
typedef struct
{
	float x;
	float y;
	float z;
} floatXYZ;

//欧拉角
typedef struct 
{
	float R;
	float P;
	float Y;
}floatRPY;

//关节转角
typedef struct
{
	float theta_1;
	float theta_2;
	float theta_3;
} floatTheta;

floatXYZ Leg_Position_Vector(int num,floatXYZ pos,floatRPY angle);

void inverse_kinematics(floatXYZ AB[],int len);

floatXYZ SwayStatus_Trajectory(MainState_t MainState,int num,float t);  //输入为时间微分

floatXYZ SupportingStatus_Trajectory(MainState_t MainState,int num,float t);  //输入为时间微分


