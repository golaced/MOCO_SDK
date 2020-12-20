#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <wiringSerial.h>
#include "moco_sdk_uart.h"
#include "moco_sdk.h"

#define RX_IN_USB 0

using namespace std;

#if !RX_IN_USB
serial::Serial m_serial("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(5));
#else
serial::Serial m_serial("/dev/ttyUSB0",115200 , serial::Timeout::simpleTimeout(5));
#endif

int main()
{
int data_length = 0,fd=0,data_cnt=0;
int tx_length =0;
unsigned char rx_temp[200]={0};
unsigned char tx_temp[200]={0};

#if !RX_IN_USB
    while(( fd = serialOpen ("/dev/ttyS0",115200))<0)
#else
    while(( fd = serialOpen ("/dev/ttyUSB0",115200))<0)
#endif
    {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
    }
	cout<<"serial Open!"<<endl;

 	sdk_init();

 while (1)
 {
	data_length = m_serial.available();
	if (data_length>0)
	{
		//cout<<"data_length: "<<data_length<<endl;
		m_serial.read(rx_temp, data_length);
		if(data_length>0&&data_length<199){//push into fifo
			for(int i=0;i<data_length;i++)
		            rx_anal(rx_temp[i]);
		}
	}

	//SDK 
	spd_test(0.0025);
	//att_test(0.0025);
	//TX
	tx_length = sdk_thread();
	if (tx_length>0&&tx_length<199)
	{
		//cout<<"tx_length: "<<tx_length<<endl;
		for(int i=0;i<tx_length;i++)
			tx_temp[i]=SendBuff_USB[i];
	   	m_serial.write(tx_temp, tx_length);
	}
    }

}
