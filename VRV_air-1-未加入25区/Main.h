/*--------------------------------------------------------------------------
Main.H

EEROM ADDRE	0X01	BOX
		0X02	light AREA	灯光区域号
		0X03	blinds AREA	窗帘区域号
		0X04	air conditioner  AREA	空调区域号
		0X05	触摸屏设备代码
		0X06	触摸屏设备BOX
		0X07	温控探头设备代码
		0X08	温控探头设备BOX	 

Header file for F340
Copyright (c) 2009 Pan xiao bin
All rights reserved.
V1.0
--------------------------------------------------------------------------*/

#ifndef __Main_H__
#define __Main_H__

#define uchar unsigned char
#define uint  unsigned int

typedef unsigned char	uint8;
typedef unsigned int	uint16;
typedef unsigned long	uint32;



#define OFF   0
#define ON    1
#define ver   1				  //modbus专用于徐月卿，modbus device id固定为2，只能读取holding register



#define TH1_VAL		0xF4			//定时器1的重载值
#define TL1_VAL		0xF4			//9600,7.3728,smod=0
//#define TH0_VAL		0xf8			//7.3728mHZ 2400
//#define TL0_VAL		0x30			//2MS定时值 2400

#define TH0_VAL		0xf4			//7.3728mHZ
#define TL0_VAL		0x18			//2MS定时值


#define DEVICE_CODE		0xb9



#define ROOM_NO			0x235		//房间号码 以此为基准  0，+1，+2 受控制

#define EEPROM			0xA0
#define TSK1_ADDR 0X20
#define TSK2_ADDR 0X22
#define TSK3_ADDR 0X24
#define TSK4_ADDR 0X26
#define TSK5_ADDR 0X28
#define TSK6_ADDR 0X2a
#define TSK7_ADDR 0X2c
#define TSK8_ADDR 0X2e

#define IN_LEN		8



sbit	SCL		=P0^7;
sbit	SDA		=P0^6;

sbit	OE0		=P2^1;
sbit	OE1		=P2^2;
sbit	service_led	=P2^3;



extern void clear_wdt();
extern void Delay(uchar n);
extern void Delay_ms(uint n);
extern void refresh_modbus_reg(uint16 i2c_addr,uint8 i);
extern int WriteEE(unsigned int SubAdr,unsigned char Num,unsigned char *Wbuf);
extern int ReadEE(unsigned int SubAdr,unsigned char Num,unsigned char *Rbuf);


extern void trans_data1();
extern void set_tranready1();
extern void resend_msg_proc();


extern void trans_data();

extern void Start();
extern void Stop();

extern void receive_data1();
extern void receive_data();


extern void send_BUF_TRAN();
extern uint16 crc_chk(uint8 *pData,uint8 nLen);
extern void Modbus_Send(uint8 length);

extern void Delay_100MS_proc();


extern	bit     sreceive;
extern	bit     sreceiveend;
extern  bit     TRFLAG;
extern  bit     serviceled;

extern  bit 	resend_msg;

extern  bit 	sreceive_1; 
extern  bit 	sreceiveend_1;
extern  bit 	tranready;
extern  bit 	tranready_1;

extern  bit 	setup_mode;
extern  bit 	block_writer;
extern  bit 	back_light;
extern  bit 	read_panel_0;
extern  bit 	panel_0_LED_action;

extern  bit 	keyaction		;			
extern  bit  	keyprese		;
extern  bit 	keyrelase		;

extern  bit 	panel_0_LED_0		;

extern  bit 	area_4_OFF		;
extern  bit 	area_6_OFF		;
extern  bit 	guest_IN		;	
extern  bit 	all_off		;



extern unsigned char 	bdata   panel_0_LED;
extern unsigned char 	idata   receivetime_1;
extern unsigned char 	idata	read_panel_0_save;
extern unsigned char 	idata	read_panel_1_save;
extern unsigned char 	idata 	key_NO;



extern unsigned char 	data 	time20ms;
extern unsigned char 	data 	time20ms_out;
extern unsigned char 	data 	box;
extern unsigned char 	data 	back_light_delay;
extern unsigned char 	data 	COM0_length;






extern unsigned char	data	RTC_BUF[4];
extern unsigned char 	xdata	BUF_RECE[180];
extern unsigned char 	xdata 	*COM_buf_point;
extern unsigned char 	data 	BUF_RECE1[IN_LEN];
extern unsigned char 	data 	*COM1_buf_point;
extern unsigned char 	data 	BUF_TRCE1[IN_LEN];
extern unsigned char 	data 	*COM1_buf_trce_point;


extern unsigned int 	xdata 	eerom_address;

extern unsigned char 	resend_msg_length;
extern unsigned char 	xdata	resend_msg_buf[64];
extern unsigned char 	resend_msg_sec;
extern unsigned char 	resend_msg_counter;
extern unsigned char 	temp;
extern unsigned char 	xdata	receive_length;
extern unsigned char 	xdata	receivetime;

extern unsigned char 	xdata	returne_air_mode[3];
extern unsigned char 	xdata	returne_set_temperature[3];
extern unsigned char 	xdata	returne_temperature[3];
extern unsigned char 	xdata	returne_fan1_speed[3];
extern unsigned char 	xdata	returne_fan2_speed[3];


/***********************	added @2012-19 by jin	********************************/
extern bit status_ctrl;
extern bit status_initial;
extern bit status_cmd_ack;
extern bit RQ_ST;
extern uint8 	xdata air_condi_num;
extern uint8	xdata air_box_num[64];
extern uint16 	xdata air_power[64];
extern uint16	xdata shadow_power[64];
extern uint16 	xdata air_temperature[64];
extern uint16 	xdata air_mode[64];
extern uint16 	xdata air_speed[64];
extern uint16 	xdata actual_temperature[64];
//extern uint8 resend_times;

extern void set_vrv_power_off(uint8 vrv_addr);
extern void set_air_status(uint8 mode , uint8 speed,uint8 temperature ,uint8 vrv_addr);


//extern  xdata struct Task_d Task_data[9]  ;


//extern   struct	Task_d   xdata *Task_point;



#endif











//onetouch_TIME	EQU	050			;1 SEC
//SIGNON_TIME	EQU	150			;3 SEC


//config_state	equ	08H







