#include "Inverse_K.h"
#include "MPU6050.h"
#include "stdio.h"



//单腿关节转角
floatTheta Leg_angle[4];

uint8_t mode;    //工作模式

/*求出单腿位置向量*/
floatXYZ Leg_Position_Vector(int num,floatXYZ pos,floatRPY angle)
{
	//单腿位置向量
  floatXYZ AB[4];
	//化为弧度制
	angle.R=angle.R*DEG_TO_RAD;
	angle.P=angle.P*DEG_TO_RAD;
	angle.Y=angle.Y*DEG_TO_RAD;
	
	float rotx_R[3][3]={{            1,              0,             0 },
	                    {            0,   cos(angle.R), -sin(angle.R) },
	                    {            0,   sin(angle.R),  cos(angle.R) }};
	
	float roty_P[3][3]={{ cos(angle.P),              0,  sin(angle.P) },
	                    {            0,              1,             0 },
	                    {-sin(angle.P),              0,  cos(angle.P) }};	
	
	float rotz_Y[3][3]={{ cos(angle.Y),  -sin(angle.Y),             0 },
	                    { sin(angle.Y),   cos(angle.Y),             0 },
	                    {            0,              0,             1 }};	
	
	float R_P[3][3]={0},R[3][3]={0};
	for(int i=0; i<3; i++) 
	{		
		for(int j=0; j<3; j++) 
		{			
			for(int k=0; k<3; k++)
			{	
				R_P[i][j] += rotx_R[i][k] * roty_P[k][j];
			}			
		}
	}
	//计算得到旋转矩阵R[3][3]
	for(int i=0; i<3; i++) 
	{		
		for(int j=0; j<3; j++) 
		{			
			for(int k=0; k<3; k++)
			{	
				R[i][j] += R_P[i][k] * rotz_Y[k][j];	
			}			
		}
	}

	floatXYZ footprint_struct[4],body_struct[4];
	//左前腿 A1B1
	footprint_struct[0].x =  0.5*Foot_Length;
	footprint_struct[0].y =  0.5*Foot_Width;
	footprint_struct[0].z =  0;
	body_struct[0].x      =  0.5*Body_Length;
	body_struct[0].y      =  0.5*Body_Width;
	body_struct[0].z      = -Body_Thickness;
	//右前腿 A2B2
	footprint_struct[1].x =  0.5*Foot_Length;
	footprint_struct[1].y = -0.5*Foot_Width;
	footprint_struct[1].z =  0;
	body_struct[1].x      =  0.5*Body_Length;
	body_struct[1].y      = -0.5*Body_Width;
	body_struct[1].z      = -Body_Thickness;
  //左后腿 A3B3
	footprint_struct[2].x = -0.5*Foot_Length;
	footprint_struct[2].y =  0.5*Foot_Width;
	footprint_struct[2].z =  0;
	body_struct[2].x      = -0.5*Body_Length;
	body_struct[2].y      =  0.5*Body_Width;
	body_struct[2].z      = -Body_Thickness;	
	//右后腿 A4B4
	footprint_struct[3].x = -0.5*Foot_Length;
	footprint_struct[3].y = -0.5*Foot_Width;
	footprint_struct[3].z =  0;
	body_struct[3].x      = -0.5*Body_Length;
	body_struct[3].y      = -0.5*Body_Width;
	body_struct[3].z      = -Body_Thickness;		
	
	AB[num].x=-pos.x-(R[0][0]*body_struct[num].x+R[0][1]*body_struct[num].y+R[0][2]*body_struct[num].z)+footprint_struct[num].x;
	AB[num].y=-pos.y-(R[1][0]*body_struct[num].x+R[1][1]*body_struct[num].y+R[1][2]*body_struct[num].z)+footprint_struct[num].y;
	AB[num].z=-pos.z-(R[2][0]*body_struct[num].x+R[2][1]*body_struct[num].y+R[2][2]*body_struct[num].z)+footprint_struct[num].z;
	
	float temp;
  //坐标系方向调整
	temp=AB[num].x;
	AB[num].x=-AB[num].z;
	AB[num].z=temp;
	//printf("i:%d,AB.x=%f,AB.y=%f,AB.z=%f \n",num,AB[num].x,AB[num].y,AB[num].z);
	return AB[num];
}


/*求出运动学逆解*/
void inverse_kinematics(floatXYZ AB[],int len)
{
	for(int i=0;i<len;i++)
	{
		//逆解
		float fai;
		float omega;
		if(i==0||i==2)
			omega=atan2(-AB[i].y,AB[i].x);
		else
		  omega=atan2(AB[i].y,AB[i].x);
		//theta_1范围 [-90,90]
		Leg_angle[i].theta_1=asin( a1/sqrt( pow(AB[i].x,2)+pow(AB[i].y,2) ) )+omega;
		//theta_3范围 [-180,0]	
		Leg_angle[i].theta_3=-acos((pow((AB[i].x*cos(Leg_angle[i].theta_1)+AB[i].y*sin(Leg_angle[i].theta_1)),2)+pow(AB[i].z,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3));
		fai=atan2(AB[i].z,(AB[i].y*sin(Leg_angle[i].theta_1)+AB[i].x*cos(Leg_angle[i].theta_1)));
		//theta_2范围 [0,90]
		Leg_angle[i].theta_2=asin(-a3*sin(Leg_angle[i].theta_3)/sqrt(pow((AB[i].x*cos(Leg_angle[i].theta_1)+AB[i].y*sin(Leg_angle[i].theta_1)),2)+pow(AB[i].z,2)))-fai;
		//printf("Leg[%d]:theta_1=%1.f,theta_2=%1.f,theta_3=%1.f\n",i,Leg_angle[i].theta_1*RAD_TO_DEG,Leg_angle[i].theta_2*RAD_TO_DEG,Leg_angle[i].theta_3*RAD_TO_DEG);
	}
}

//摆动相足端轨迹
floatXYZ SwayStatus_Trajectory(MainState_t MainState,int num,float t)  //输入为时间微分
{
	floatXYZ AB;
	switch(GetMainState()) 
	{				 
		 case MainState_Step:	
		 {
				AB.z=-10*t/T;
				if(num%2==1)   
					AB.y=-SwayStatus_Yst;
				else       
					AB.y=SwayStatus_Yst;
			}			 
		 break;	
		 /*向前*/
		 case MainState_Forward:
		 {
				AB.z=SwayStatus_Zst+S*(t/T-(sin(2*PI*t/T))/(2*PI));
				if(num%2==1)   
					AB.y=-SwayStatus_Yst;
				else       
					AB.y=SwayStatus_Yst;
			}
		 break;
		/*向后*/
		 case MainState_Backward:
		 {
				AB.z=-(SwayStatus_Zst+S*(t/T-(sin(2*PI*t/T))/(2*PI)));
				if(num%2==1)   
					AB.y=-SwayStatus_Yst;
				else         
					AB.y=SwayStatus_Yst; 
			}				
		 break;		
		/*向左平移*/
		 case MainState_Move_to_Left:
		 {
				AB.z=0;
			 	if(num%2==1) 
					AB.y=-Move_to_Left_SwayStatus_Yst+S_*(t/T-(sin(2*PI*t/T))/(2*PI));
				else
					AB.y=Move_to_Left_SwayStatus_Yst+S_*(t/T-(sin(2*PI*t/T))/(2*PI));
		 }	  
		 break;	
		 /*向右平移*/
		 case MainState_Move_to_Right:
		 {
				AB.z=0;
			 	if(num%2==1) 
					AB.y=-Move_to_Right_SwayStatus_Yst-S_*(t/T-(sin(2*PI*t/T))/(2*PI));
				else
					AB.y=Move_to_Right_SwayStatus_Yst-S_*(t/T-(sin(2*PI*t/T))/(2*PI));
		 }		 
		 break;	
		 
		 case MainState_Turn_Left:
			{
				switch(num) 
				{
					case 0:   //左前腿
						AB.y=SwayStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					break;
					
					case 1:   //右前腿
						AB.y=-SwayStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					break;
					
					case 2:   //左后腿
						AB.y=SwayStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
          break;
          					
					case 3:   //右后腿
						AB.y=-SwayStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
          break;          					
				}
			}
		 break;	
		 
		 case MainState_Turn_Right:
		 {
			 switch(num) 
				{
					case 0:   //左前腿
						AB.y=SwayStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					break;
					
					case 1:   //右前腿
						AB.y=-SwayStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=+0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					break;
					
					case 2:   //左后腿
						AB.y=SwayStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
          break;
          					
					case 3:   //右后腿
						AB.y=-SwayStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
					  AB.z=+0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*(t/T-(sin(2*PI*t/T))/(2*PI));
          break;          					
				} 
			}				
		 break;	
		 
     default:
			 ;
	 }
	AB.x=SwayStatus_Xst-H*(0.5-0.5*cos(2*PI*t/T));
	return AB;
}

//支撑相足端轨迹
floatXYZ SupportingStatus_Trajectory(MainState_t MainState,int num,float t)  //输入为时间微分
{
	floatXYZ AB;
	switch(GetMainState()) 
	{				 
		 case MainState_Step:	
		 {
				AB.z=-10*t/T;
				if(num%2==1)   
					AB.y=-SwayStatus_Yst;
				else       
					AB.y=SwayStatus_Yst;
			}			 
		 break;	
		 /*向前*/
		 case MainState_Forward:
		 {
				AB.z=SupportingStatus_Zst-S*t/T;
				if(num%2==1)
					AB.y=-SupportingStatus_Yst;
				else 
					AB.y=SupportingStatus_Yst;
			}
		 break;
		/*向后*/
		 case MainState_Backward:
		 {
				AB.z=-SupportingStatus_Zst+S*t/T;
				if(num%2==1)
					AB.y=-SupportingStatus_Yst;
				else 
					AB.y=SupportingStatus_Yst;
			}				
		 break;		
		/*向左平移*/
		 case MainState_Move_to_Left:
		 {
				AB.z=0;
			 	if(num%2==1) 
					AB.y=-Move_to_Left_SupportingStatus_Yst-S_*t/T;
				else
					AB.y=Move_to_Left_SupportingStatus_Yst-S_*t/T;
		 }	  
		 break;	
		 /*向右平移*/
		 case MainState_Move_to_Right:
		 {
				AB.z=0;
			 	if(num%2==1) 
					AB.y=-Move_to_Right_SupportingStatus_Yst+S_*t/T;
				else
					AB.y=Move_to_Right_SupportingStatus_Yst+S_*t/T;
		 }		 
		 break;	
		 
		 case MainState_Turn_Left:
			{
				switch(num) 
				{
					case 0:   //左前腿
						AB.y=SupportingStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*t/T;
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*t/T;
					break;
					
					case 1:   //右前腿
						AB.y=-SupportingStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*t/T;
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*t/T;
					break;
					
					case 2:   //左后腿
						AB.y=SupportingStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*t/T;
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*t/T;
          break;
          					
					case 3:   //右后腿
						AB.y=-SupportingStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*t/T;
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*t/T;
          break;          					
				}
			}
		 break;	
		 
		 case MainState_Turn_Right:
			{
				switch(num) 
				{
					case 0:   //左前腿
						AB.y=SupportingStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*t/T;
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*t/T;
					break;
					
					case 1:   //右前腿
						AB.y=-SupportingStatus_Yst+0.5*S_*sin(Beta)-0.5*2*S_*sin(Beta)*t/T;
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*t/T;
					break;
					
					case 2:   //左后腿
						AB.y=SupportingStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*t/T;
					  AB.z=0.5*S_*cos(Beta)-0.5*2*S_*cos(Beta)*t/T;
          break;
          					
					case 3:   //右后腿
						AB.y=-SupportingStatus_Yst-0.5*S_*sin(Beta)+0.5*2*S_*sin(Beta)*t/T;
					  AB.z=-0.5*S_*cos(Beta)+0.5*2*S_*cos(Beta)*t/T;
          break;          					
				}
			}   
		 break;	
		 
     default:
			 ;
	 }

	AB.x=SupportingStatus_Xst;
	return AB;
}
