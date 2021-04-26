
#include "PID.h"

PID_Para motor1;
float uk1 ;
float sum_err;
PID_Para PID_para_vel,PID_para_line;
//float Kp = 0.005, Ki = 0.1, Kd=0.05;
float Error_value=0, P_part =0,I_part =0,D_part =0;
float out =0;
float pre_Error_value=0,pre_Error=0,Error=0;
//float out_line =0;

extern float setkp_line, setki_line, setkd_line;
extern float setkp_speed, setki_speed, setkd_speed;
extern uint8_t PIDflag;


void PID_Init() // ham khoi tao
{
	PID_para_vel.Kp = 3;
	PID_para_vel.Ki = 4;
	PID_para_vel.Kd = 0.015;
	PID_para_vel.Ts_ = 0.01;
	PID_para_vel.ek = 0 ;
	PID_para_vel.ek_1 = 0 ;
	PID_para_vel.ek_2 = 0;
	PID_para_vel.uk = 0;
	PID_para_vel.uk_1 = 0;
	
	PID_para_line.Kp = 0.002;
	PID_para_line.Ki = 0.2;
	PID_para_line.Kd = 0.1;
	PID_para_line.Ts_ = 0.01;
	PID_para_line.ek = 0 ;
	PID_para_line.ek_1 = 0 ;
	PID_para_line.ek_2 = 0;
	PID_para_line.uk = 0;
	PID_para_line.uk_1 = 0;
	
}

void Restart_PID1(void)
{
	motor1.ek = 0 ;
	motor1.ek_1 = 0 ;
	motor1.ek_2 = 0;
	motor1.uk = 40;
	motor1.uk_1 = 40 ;
//	V_1 = 0;
}

void PID_GA25_Lifting(float x_ref, float x_measure) // v_sv RPM
{
	if (x_measure >= (x_ref-(float)0.04) && x_measure <= (x_ref+ (float)0.4)) 
	{
	motor1.uk =0;
	}
	else
		{
	motor1.ek = x_ref - x_measure;
	motor1.uk = motor1.uk_1 + motor1.Kp*(motor1.ek-motor1.ek_1)
													+ motor1.Ki*motor1.Ts_*(motor1.ek + motor1.ek_1)*0.5
													+ motor1.Kd*(motor1.ek-2*motor1.ek_1+motor1.ek_2)/motor1.Ts_;

	motor1.uk_1 = motor1.uk;
	motor1.ek_2 = motor1.ek_1;
	motor1.ek_1 = motor1.ek;
		}
	//uk1 = motor1.uk;
	if (motor1.uk >= 100) motor1.uk =100;
	else if (motor1.uk <= -100) motor1.uk = -100;
	uk1= motor1.uk;
	
}

float PID_Velocity(float x_ref, float x_measure) // v_sv RPM
{
	if (PIDflag == 0) 
	{
		PID_para_vel.uk = 0;
		PID_para_vel.uk_1 = 0;
		return 0;
	}	

	
	PID_para_vel.ek = x_ref - x_measure;
	PID_para_vel.uk = PID_para_vel.uk_1 + setkp_speed*(PID_para_vel.ek-PID_para_vel.ek_1)
													+ setki_speed*PID_para_vel.Ts_*(PID_para_vel.ek + PID_para_vel.ek_1)*0.5
													+setkd_speed*(PID_para_vel.ek-2*PID_para_vel.ek_1+PID_para_vel.ek_2)/PID_para_vel.Ts_;

	PID_para_vel.uk_1 = PID_para_vel.uk;
	PID_para_vel.ek_2 = PID_para_vel.ek_1;
	PID_para_vel.ek_1 = PID_para_vel.ek;
	if (PID_para_vel.uk >= 100) PID_para_vel.uk =100;
	else if (PID_para_vel.uk <= -100) PID_para_vel.uk = -100;
	return  PID_para_vel.uk;
}

float error1 = 0;
float preError1 = 0;
float sum_error = 0;

float Kp = 0.5;
float Ki = 0.0005;
float Kd = 0.05;


float P = 0,I=0,D=0,previous_error=0;

void PID_Line(float x_measure,float udk)
{
	if(PIDflag == 0)
	{	 
	   preError1 = 0;
	}
	else if (PIDflag == 1)
	{
	   int error = x_measure - 1000;
     int out_line = setkp_line * error + setkd_line * (error - preError1);
     preError1 = error;
	
   	 if (out_line > udk) out_line  = udk;
  	 else if (out_line  < -udk) out_line  = -udk;
	
	   //int rightMotorSpeed = udk + 2 + out_line/1.6;              
	 //  int leftMotorSpeed = udk - out_line/1.6;     
 int rightMotorSpeed = udk + 2 + out_line/3;              
	   int leftMotorSpeed = udk - out_line/3;  		
	
	   Run_Motor(LEFT_MOTOR,leftMotorSpeed);
	   Run_Motor(RIGHT_MOTOR,rightMotorSpeed);
	}
}

