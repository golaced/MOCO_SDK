#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "moco_sdk_uart.h"
#include "moco_sdk.h"

using namespace std;
char state_sdk=0;
float timer=0;

char set_gait_mode(char set_mode)
{
	char mode_fd=0;
	switch(set_mode)
	{
	case GAIT_IDLE:mode_fd=M_SAFE ;break;
	case GAIT_TROT:mode_fd=M_TROT ;break;
	case GAIT_FTROT:mode_fd=M_F_TROT ;break;
	case GAIT_ST_RC:mode_fd=M_STAND_RC ;break;
	case GAIT_ST_IMU:mode_fd=M_STAND_IMU ;break;
	case GAIT_ST_PUSH:mode_fd=M_STAND_PUSH ;break;
	case GAIT_RECOVER:mode_fd=M_RECOVER ;break;
	case GAIT_FALL:mode_fd=M_FALLING ;break;
	}
	//printf("RX_MODE::%d %d\n",mode_fd,moco.param.robot_mode);
	moco_sdk.gait_mode=set_mode;
	if(moco.param.robot_mode==mode_fd)
		return 1;
	else 
		return 0;
}

void power_off(void)
{
	moco_sdk.power_off=1;
	moco_sdk.gait_mode=0;
	moco_sdk.rc_att_cmd[PITr]=0;
	moco_sdk.rc_att_cmd[ROLr]=0;
	moco_sdk.rc_att_cmd[YAWr]=0;
	moco_sdk.rc_spd_cmd[Xr]=0;
	moco_sdk.rc_spd_cmd[Yr]=0;
	moco_sdk.rc_spd_cmd[Zr]=0;
	timer=0;
}

void sdk_reset(void)
{
	moco_sdk.power_off=1;
	moco_sdk.gait_mode=0;
	moco_sdk.rc_att_cmd[PITr]=0;
	moco_sdk.rc_att_cmd[ROLr]=0;
	moco_sdk.rc_att_cmd[YAWr]=0;
	timer=0;
}

void set_pos_z(float set)
{
	moco_sdk.rc_pos_cmd[Zr]=set;
}

void set_spd_x(float set)
{
	moco_sdk.rc_spd_cmd[Xr]=set;
}

void set_spd_y(float set)
{
	moco_sdk.rc_spd_cmd[Yr]=set;
}

void set_spd_yaw_rate(float set)
{
	moco_sdk.rc_att_rate_cmd[YAWr]=set;
}

void set_att_pit(float set)
{
	moco_sdk.rc_att_cmd[PITr]=set;
}

void set_att_rol(float set)
{
	moco_sdk.rc_att_cmd[ROLr]=set;
}

void set_att_yaw(float set)
{
	moco_sdk.rc_att_cmd[YAWr]=set;
}

void sdk_init(void)
{
	state_sdk=0;
	moco_sdk.gait_mode=0;
	moco_sdk.rc_att_cmd[PITr]=0;
	moco_sdk.rc_att_cmd[ROLr]=0;
	moco_sdk.rc_att_cmd[YAWr]=0;
	timer=0;
	cout<<"SDK::Init Done!!"<<endl;
}

void spd_test(float dt)
{
static float set_sin=0;

	switch(state_sdk)
	{
	case 0:
	set_pos_z(0.1);
	timer+=dt;
	if(timer>2.5)
	{
	cout<<"SDK::System ON!!"<<endl;
	state_sdk++;
	}
	break;
	
	case 1:
	if(set_gait_mode(GAIT_ST_RC))
	{
	cout<<"SDK::Stand_Rc Active!!"<<endl;
	state_sdk++;
	timer=0;
	}
	break;
	
	case 2:
	timer+=dt;
	if(timer>1.5)
	{
		if(set_gait_mode(GAIT_TROT))
		{
		cout<<"SDK::TROT Active!!"<<endl;
		state_sdk++;
		timer=0;
		}
	}
	break;
	
	case 3:
	timer+=dt;
	if(timer<2.5)
	set_spd_x(0.03);
	else
	set_spd_x(-0.03);

	if(timer>5)
	{	
		set_spd_x(0);
		if(set_gait_mode(GAIT_ST_IMU))
		{
		cout<<"SDK::STAND_IMU to RC Active!!"<<endl;
		state_sdk++;
		timer=0;
		set_gait_mode(GAIT_ST_RC);
		}
	}
	break;

	case 4:
	set_sin+=dt*2;
	timer+=dt;
	

	if(timer>6)
	{
		set_att_pit(0);
		set_att_yaw(0);
		set_pos_z(0.1-(timer-6)*0.04);
		if(moco.pos_n[Zr]<=0.05)
		{
		power_off();
		cout<<"SDK::POWER_OFF!!"<<endl;
		state_sdk++;
		timer=0;
		}
	}else{
	set_att_pit(sin(set_sin)*8);
	set_att_yaw(cos(set_sin)*12);
	}
	break;
	

	case 99:
		if(set_gait_mode(GAIT_RECOVER))
		{
		cout<<"SDK::Recovery Active!!"<<endl;
		state_sdk++;
		timer=0;
		}
	break;
	case 100:
		if(moco.param.robot_mode==M_SAFE||moco.param.robot_mode==M_STAND_RC||moco.param.robot_mode==M_STAND_IMU )
		{
		cout<<"SDK::Recovery Done!!"<<endl;
		state_sdk=0;
		}
	break;
	}
	
	if(moco.param.robot_mode==M_FALLING&&state_sdk!=99){
	   cout<<"SDK::Robot Falling!!"<<endl;
	   state_sdk=99;
	}
}


void att_test(float dt)
{

	switch(state_sdk)
	{
	case 0:
	set_pos_z(0.1);
	timer+=dt;
	if(timer>3.5)
	{
	cout<<"SDK::System ON!!"<<endl;
	state_sdk++;
	timer=0;
	}
	break;
	
	case 1:
	if(set_gait_mode(GAIT_ST_RC))
	{
	cout<<"SDK::Stand_Rc Active!!"<<endl;
	state_sdk++;
	timer=0;
	}
	break;
	
	case 2:
	timer+=dt*1.5;
	
	set_att_pit(sin(timer)*8);
	set_att_yaw(sin(timer)*10);
	
	break;
	

	case 99:
		if(set_gait_mode(GAIT_RECOVER))
		{
		cout<<"SDK::Recovery Active!!"<<endl;
		state_sdk++;
		timer=0;
		}
	break;
	case 100:
		if(moco.param.robot_mode==M_SAFE||moco.param.robot_mode==M_STAND_RC||moco.param.robot_mode==M_STAND_IMU )
		{
		cout<<"SDK::Recovery Done!!"<<endl;
		state_sdk=0;
		}
	break;
	}
	
	if(moco.param.robot_mode==M_FALLING&&state_sdk!=99){
	   cout<<"SDK::Robot Falling!!"<<endl;
	   state_sdk=99;
	}
}

//rostopic pub -1 moco/cmd_gait std_msgs/Float32 -- 4   5  1
//rostopic pub -r 1 moco/cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
void ros_test(char ros_connect,float dt)
{
	static char ros_connect_reg=0;
	switch(state_sdk)
	{
	case 0:
	set_pos_z(0.1);
	if(ros_connect&&moco_sdk.mode_ros!=GAIT_POWER_OFF)
		timer+=dt;
	else
		timer=0;
	if(timer>1)
	{
	cout<<"SDK::ROS Connect In!!"<<endl;
	state_sdk++;
	}
	break;
	
	case 1:
	if(moco_sdk.mode_ros==GAIT_ST_RC){
		if(set_gait_mode(GAIT_ST_RC))
		{
		cout<<"SDK::Stand_Rc Active!!"<<endl;
		state_sdk++;
		timer=0;
		}
	}
	break;
	
	case 2:
	timer+=dt;
	if(timer>2)
	{
		if(moco_sdk.mode_ros==GAIT_TROT){
			if(set_gait_mode(GAIT_TROT))
			{
			cout<<"SDK::TROT Active!!"<<endl;
			state_sdk++;
			timer=0;
			}
		}
	}
	break;
	
	case 3:
	timer+=dt;
		
	if(timer>0.5)
	{	
		if(moco_sdk.mode_ros==GAIT_ST_IMU||moco_sdk.mode_ros==GAIT_ST_RC){
			if(set_gait_mode(GAIT_ST_IMU))
			{
			cout<<"SDK::STAND_IMU to RC Active!!"<<endl;
			state_sdk++;
			timer=0;
			set_gait_mode(GAIT_ST_RC);
		}
		}
	}
	break;

	case 4:
	timer+=dt;
	if(timer>0.5)
	{
		if(moco_sdk.mode_ros==GAIT_TROT){
			if(set_gait_mode(GAIT_TROT))
			{
			cout<<"SDK::TROT Active!!"<<endl;
			state_sdk=3;
			timer=0;
			}
		}
	}
	break;
	//Protect
	case 99:
		if(set_gait_mode(GAIT_RECOVER))
		{
		cout<<"SDK::Recovery Active!!"<<endl;
		state_sdk++;
		timer=0;
		}
	break;
	case 100:
		if(moco.param.robot_mode==M_SAFE||moco.param.robot_mode==M_STAND_RC||moco.param.robot_mode==M_STAND_IMU )
		{
		cout<<"SDK::Recovery Done!!"<<endl;
		state_sdk=0;
		}
	break;

	case 110:
		power_off();
		cout<<"SDK::POWER_OFF!!"<<endl;
		state_sdk++;
	break;
	case 111:
		if(ros_connect&&moco_sdk.mode_ros!=GAIT_POWER_OFF)
			state_sdk=0;
	break;
	}
	
	if(moco.param.robot_mode==M_FALLING&&state_sdk!=99){
	   cout<<"SDK::Robot Falling!!"<<endl;
	   state_sdk=99;
	}

	if(moco_sdk.mode_ros==GAIT_POWER_OFF&&state_sdk<110){
	   cout<<"SDK::FORCE POWER_OFF!!"<<endl;
	   state_sdk=110;
	}

	if(!ros_connect&&ros_connect_reg&&state_sdk!=110){
	   cout<<"SDK::ROS LOSS!!"<<endl;
	   state_sdk=110;
	}
	ros_connect_reg=ros_connect;
}
