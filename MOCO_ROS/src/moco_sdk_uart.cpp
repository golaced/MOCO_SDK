#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "moco_sdk_uart.h"

using namespace std;

_ROBOT moco;

//RX
unsigned char data_buf[BUFF_SIZE] = {0};

bool checkcommand(const unsigned char *data_buf, int data_length)
{
	uint8_t sum=0;
	int i;
	

	if (!(*(data_buf) == 0xBA && *(data_buf + 1) == 0xBF))
	{
		cout<<"Head Check Fail!!!"<<endl;
		return false; 
	}

	for(int32_t i=0;i<data_length-1;++i)
	{
	 sum += *(data_buf+i);
	}

	if (*(data_buf + data_length - 1) != sum)
	{
		cout<<"Sum Check Fail!"<<endl;
		return false;
	}
	return true;
}

uint32_t uint32FromDataf(unsigned char *data,int* anal_cnt)
{
	uint32_t i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return i;
}

uint16_t uint16FromDataf(unsigned char *data,int* anal_cnt)
{
	uint16_t i = 0x00;
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=2;
	return i;
}

float floatFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	float out=0;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	
	 out=(*(float *)&i);
	return out;
}

float int16FromDataf(unsigned char *data,int* anal_cnt)
{
	int16_t temp = (int16_t)((*(data+*anal_cnt+0)) | (*(data+*anal_cnt+1))<< 8 ); 
	float result = temp;
	*anal_cnt +=2;
	return result;
}

int32_t int24FromDataf(unsigned char *data,int* anal_cnt)
{
	int32_t temp = (int32_t)((*(data+*anal_cnt+0)) << 8 | (*(data+*anal_cnt+1))<< 16 | (*(data+*anal_cnt+2))<< 24) / 256; 
	int32_t result = temp;
	*anal_cnt +=3;
	return result;
}

char charFromDataf(unsigned char *data,int* anal_cnt)
{
	char out=0;
	 out=(*(data+*anal_cnt));
	*anal_cnt +=1;
	return (out);
}

char reverseFromDataf(unsigned char *data,int* anal_cnt,int r_num)
{
	*anal_cnt +=r_num;
	return (0);
}

int intFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return (i);
}

void decode(unsigned char *data_buf, int data_length)
{
int anal_cnt=4;

if(*(data_buf+2)==0x91){//
moco.connect=1;
moco.loss_cnt=0;
moco.att[PITr]=floatFromDataf(data_buf,&anal_cnt);	
moco.att[ROLr]=floatFromDataf(data_buf,&anal_cnt);	
moco.att[YAWr]=floatFromDataf(data_buf,&anal_cnt);
moco.att_rate[PITr]=floatFromDataf(data_buf,&anal_cnt);	
moco.att_rate[ROLr]=floatFromDataf(data_buf,&anal_cnt);	
moco.att_rate[YAWr]=floatFromDataf(data_buf,&anal_cnt);	

moco.pos_n[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.pos_n[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.pos_n[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.spd_n[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.spd_n[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.spd_n[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.acc_n[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.acc_n[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.acc_n[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.epos_b[2][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_b[2][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[2][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[0][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_b[0][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[0][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[3][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_b[3][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[3][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[1][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_b[1][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_b[1][Zr]=floatFromDataf(data_buf,&anal_cnt);

if(0){
printf("att0=%.2f att1=%.2f att2=%.2f\n", moco.att[PITr],moco.att[ROLr],moco.att[YAWr]);
printf("att0_r=%.2f att1_r=%.2f att2_r=%.2f\n", moco.att_rate[PITr],moco.att_rate[ROLr],moco.att_rate[YAWr]);
} 

}else if(*(data_buf+2)==0x92){
moco.param.W=floatFromDataf(data_buf,&anal_cnt);	
moco.param.H=floatFromDataf(data_buf,&anal_cnt);	
moco.param.L1=floatFromDataf(data_buf,&anal_cnt);	
moco.param.L2=floatFromDataf(data_buf,&anal_cnt);	
moco.param.L3=floatFromDataf(data_buf,&anal_cnt);	

moco.q[2][D_LEG]=floatFromDataf(data_buf,&anal_cnt);	
moco.q[2][X_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[2][T_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[0][D_LEG]=floatFromDataf(data_buf,&anal_cnt);	
moco.q[0][X_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[0][T_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[3][D_LEG]=floatFromDataf(data_buf,&anal_cnt);	
moco.q[3][X_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[3][T_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[1][D_LEG]=floatFromDataf(data_buf,&anal_cnt);	
moco.q[1][X_LEG]=floatFromDataf(data_buf,&anal_cnt);
moco.q[1][T_LEG]=floatFromDataf(data_buf,&anal_cnt);

moco.system.rc_spd_cmd[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.system.rc_spd_cmd[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.system.rc_att_rate_cmd[Zr]=floatFromDataf(data_buf,&anal_cnt);	
moco.system.rc_pos_cmd[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.system.bat=floatFromDataf(data_buf,&anal_cnt);	
moco.system.rc_weight=floatFromDataf(data_buf,&anal_cnt);	

moco.param.rc_mode=charFromDataf(data_buf,&anal_cnt);	
moco.param.robot_mode=charFromDataf(data_buf,&anal_cnt);	
moco.param.imu_mode=charFromDataf(data_buf,&anal_cnt);	

if(1&&0){
printf("W=%.2f H=%.2f L1=%.2f\n", moco.param.W,moco.param.H,moco.param.L1);
} 
}else if(*(data_buf+2)==0x93){

moco.cog_nn[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.cog_nn[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.cog_nn[Zr]=floatFromDataf(data_buf,&anal_cnt);	
	
moco.zmp_nn[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.zmp_nn[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.zmp_nn[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.support_cog_nn[Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.support_cog_nn[Yr]=floatFromDataf(data_buf,&anal_cnt);	
moco.support_cog_nn[Zr]=floatFromDataf(data_buf,&anal_cnt);	

moco.td_state[2]=charFromDataf(data_buf,&anal_cnt);
moco.td_state[0]=charFromDataf(data_buf,&anal_cnt);
moco.td_state[3]=charFromDataf(data_buf,&anal_cnt);
moco.td_state[1]=charFromDataf(data_buf,&anal_cnt);

moco.epos_nn[2][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_nn[2][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[2][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[0][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_nn[0][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[0][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[3][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_nn[3][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[3][Zr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[1][Xr]=floatFromDataf(data_buf,&anal_cnt);	
moco.epos_nn[1][Yr]=floatFromDataf(data_buf,&anal_cnt);
moco.epos_nn[1][Zr]=floatFromDataf(data_buf,&anal_cnt);

if(1&&0){
printf("cogx=%.2f cogy=%.2f cogz=%.2f\n", moco.cog_nn[Xr],moco.cog_nn[Yr],moco.cog_nn[Zr]);
} 
}

}


unsigned char RxBuffer1[BUFF_SIZE];
unsigned char RxState1 = 0;
int RxBufferNum1 = 0;
int RxBufferCnt1 = 0;
int _data_len1 = 0;
int _data_cnt1 = 0;
void rx_anal(unsigned char com_data)//Rx interupt
{
if(RxState1==0&&com_data==0xBA)
{
	RxState1=1;
	RxBuffer1[0]=com_data;
}
else if(RxState1==1&&com_data==0xBF)
{
	RxState1=2;
	RxBuffer1[1]=com_data;
}
else if(RxState1==2&&com_data>0&&com_data<0XF1)
{
	RxState1=3;
	RxBuffer1[2]=com_data;
}
else if(RxState1==3&&com_data<255)
{
	RxState1= 4;
	RxBuffer1[3]=com_data;
	_data_len1 = com_data;
	_data_cnt1 = 0;
}
else if(RxState1==4&&_data_len1>0)
{
	_data_len1--;
	RxBuffer1[4+_data_cnt1++]=com_data;
	if(_data_len1==0)
		RxState1= 5;
}
else if(RxState1==5)
{
	RxState1 = 0;
	RxBuffer1[4+_data_cnt1]=com_data;
		decode(RxBuffer1,_data_cnt1+5);
}
else
			RxState1 = 0;
}




//Tx
_SDK moco_sdk;

char SendBuff_USB[BUFF_SIZE];
int usb_send_cnt=0;

void setDataInt(int i)
{
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
}

void setDataChar(char i)
{
	SendBuff_USB[usb_send_cnt++] = i ;
}

void setDataFloat(float f)
{
	int i = *(int *)&f;
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
}

int uart_formate_ocu_spd_remote(void)
{ 
	int i;	char sum = 0;
	int cnt_reg=0;
	cnt_reg=usb_send_cnt;

	SendBuff_USB[usb_send_cnt++]=0xCA;
	SendBuff_USB[usb_send_cnt++]=0xCF;
	SendBuff_USB[usb_send_cnt++]=0x93;
	SendBuff_USB[usb_send_cnt++]=0;
	//moco_sdk.rc_spd_cmd[Xr]=0.1;moco_sdk.rc_spd_cmd[Yr]=-0.23;
	//moco_sdk.gait_mode=0;
	setDataChar(moco_sdk.gait_mode);
	setDataFloat(moco_sdk.rc_spd_cmd[Xr]);
	setDataFloat(moco_sdk.rc_spd_cmd[Yr]);
	setDataFloat(moco_sdk.rc_att_rate_cmd[Zr]);
	setDataFloat(moco_sdk.rc_spd_cmd[Zr]);
	setDataFloat(moco_sdk.rc_pos_cmd[Zr]);
	setDataFloat(moco_sdk.rc_att_cmd[PITr]);
	setDataFloat(moco_sdk.rc_att_cmd[ROLr]);
	setDataFloat(moco_sdk.rc_att_cmd[YAWr]);
	
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
	return usb_send_cnt;
}

int uart_formate_power_off(void)
{ 
	int i;	char sum = 0;
	int cnt_reg=0;
	cnt_reg=usb_send_cnt;

	SendBuff_USB[usb_send_cnt++]=0xCA;
	SendBuff_USB[usb_send_cnt++]=0xCF;
	SendBuff_USB[usb_send_cnt++]=0x52;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataChar(1);
	setDataChar(1);
	setDataChar(1);
	setDataChar(1);

	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
	return usb_send_cnt;
}

int sdk_thread(void)
{
	usb_send_cnt=0;
	if(moco_sdk.power_off>0)
	{
	  moco_sdk.power_off++;
	  if(moco_sdk.power_off>5)
	     moco_sdk.power_off=0;
	return uart_formate_power_off();
	}else
	return uart_formate_ocu_spd_remote();
}

