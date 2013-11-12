/***********************************************************************

                      金陵饭店和台湾按键  接口程序

	CPU       : CF340
	frequence : 12 MHZ
	
	Author    : Pan xiao bin
	Version   : 1.0
	Date      : 2011.12.10
	
	All rights reserved.

**********************************************************************/
#include "c8051F340.h"
#include "Main.h"
#include "cmd.h"


//#include "c8051f340-modbus.h"

void Delay(uchar n);
void Delay_ms(uint n);
void clear_wdt();




unsigned char bdata Flag1;

sbit keyaction			=Flag1^0;			
sbit keyprese			=Flag1^1;
sbit keyrelase			=Flag1^2;
sbit sreceive			=Flag1^3;
sbit Sreceiveend		=Flag1^4;
sbit tranready			=Flag1^5;
sbit command_echo		=Flag1^6;	//command_echo =1时 需要有应答



unsigned char bdata Flag2 = 0;
sbit signal			=Flag2^0;		
sbit setup_mode			=Flag2^1;
sbit block_writer		=Flag2^2;
sbit keyenable			=Flag2^3;
sbit flag20ms			=Flag2^4;

sbit resend_msg			=Flag2^6;


unsigned char bdata Flag3;
sbit ledon		=Flag3^0;
sbit TIME_1MS		=Flag3^1;



unsigned char bdata Flag4;
sbit tranbuffull_1	=Flag4^0;
sbit sreceive_1		=Flag4^1;
sbit sreceiveend_1	=Flag4^2;
sbit servicesw		=Flag4^3;
sbit tranready_1	=Flag4^4;

sbit led_status		=Flag4^5;
sbit TRFLAG		=Flag4^6;
sbit serviceled	 	=Flag4^7;



unsigned char bdata status =0;

sbit	guest_IN	=status^2;
sbit	all_off		=status^3;




unsigned char	data	RTC_BUF[4];
unsigned char 	xdata 	*COM_buf_point;
unsigned char 	data 	*COM1_buf_point;
unsigned char 	data 	*COM1_buf_trce_point;

unsigned char 	data 	netactivetime;
unsigned char 	data	second;
unsigned char 	data	box;
unsigned char 	data 	time20ms;
unsigned char 	data 	time20ms_out;

unsigned char 	data 	COM0_length;
unsigned int 	xdata 	eerom_address;


//unsigned char 	data 	*eeprom_point;




unsigned char 	idata   receivetime_1;


unsigned char 	xdata	BUF_RECE[180];
unsigned char 	data 	BUF_RECE1[IN_LEN];
unsigned char 	data 	BUF_TRCE1[IN_LEN];

unsigned char 	resend_msg_length=0;
unsigned char 	xdata	resend_msg_buf[64];
unsigned char 	resend_msg_sec=0;
unsigned char 	resend_msg_counter=0;
unsigned char 	temp=0;
unsigned char 	xdata	receive_length=0;
unsigned char 	xdata	receivetime =0;

unsigned char 	xdata	returne_air_mode[3];
unsigned char 	xdata	returne_set_temperature[3];
unsigned char 	xdata	returne_temperature[3];
unsigned char 	xdata	returne_fan1_speed[3];
unsigned char 	xdata	returne_fan2_speed[3];

unsigned char 	xdata	set_air_mode[3];
unsigned char 	xdata	set_temperature[3];
unsigned char 	xdata	set_fan1_speed[3];
unsigned char 	xdata	set_fan2_speed[3];
unsigned char 	xdata 	air_RQ_num = 0;

/************* added by jin *************/
uint16 xdata CRC_result,last_register_addr,temp_status;	 //last_register_addr  用来存储上次发送的寄存器地址
uint8 xdata flag_1ms,PermitTrans,readySend,last_function_code,function_code_copy;//last_function_code 用来存碨上次使用的功能码
uint8 xdata Modbus_Rcvd_Num,poling_mode,air_condi_num;															 
uint8 xdata Modbus_Reg[85],status_connect[4],status_communication[4],air_box_num[64];
//uint16 xdata svd_air_param[64][8];
//uint16 xdata function_register[192];
//uint16 xdata status_register[384];
//uint16 xdata hold_register[193];
uint16 	xdata air_power[64];
uint16	xdata shadow_power[64];
uint16 	xdata air_temperature[64];
uint16 	xdata air_mode[64];
uint16 	xdata air_speed[64];
uint16 	xdata actual_temperature[64];
uint16 	xdata RQ_address;

uint8	bdata flag_state =0;
sbit RQ_ST=flag_state^0;

uint8 xdata	Reg_temp[50];
uint8 xdata roll_poling,roll_flag,flag_fisrt_run=1;
uint32 xdata interval_ms;
uint16 xdata last_reg_num;

uint8 xdata station_ip,control_mode,actual_tempture[64],actual_tempture1;

uint8 xdata status_adapter=0,resend_times;
uint16 xdata temp_array[50];

unsigned char bdata flag_air_sys=0;
sbit status_initial 			=	flag_air_sys^0;		//适配器是否准备好的标志
sbit status_ctrl				=	flag_air_sys^1;		//是否是本模块在控制的标志
sbit status_check_function		=	flag_air_sys^2;		//检查功能标志位
sbit status_vrv_pointed			=	flag_air_sys^3;		//vrv地址是否通过调试软件指派了
sbit status_cmd_ack				= 	flag_air_sys^4;		//命令是否确认
sbit status_no_other_dakin_vrv	=	flag_air_sys^5;		//是否存在有其他的VRV模块
sbit status_device_first_run	=	flag_air_sys^6;		//设备是否首次启动
sbit status_repeat_0x06			=	flag_air_sys^7;		//0x06的命令是否重复

unsigned char bdata flag_sys_status=0;
sbit status_ctrl_cmd_sent		=	flag_sys_status^0;
sbit status_reply_flag			=	flag_sys_status^1;
sbit flag_1s					=	flag_sys_status^2;
sbit flag_inc					= flag_sys_status^3;

typedef enum haier_state
{
	requesting = 0,
	sending,
	ST_onoff,	   // 0 关 1开
	ST_mode,		   // 0 自动 1送风 2制冷 3除湿 4制热
	ST_temperature,// 16~30
	ST_speed,		   // 0 自动 1低风 2中风 3高风
	RQ_onoff,
	RQ_mode,
	RQ_temperature,
	RQ_speed,
	RQ_out_state
}haier_state_machine;		

haier_state_machine modbus_state;
haier_state_machine state_copy;	
uint16 last_address;														   

/***************************************/

void Timer_Init()
{
	TCON=0x10;
	TMOD=0x21;
	//CKCON=000; 2400
	TL0=0xA0;
	TH0=0xA0;
	//TH1=0x30; 2400
	 
	CKCON=0x01;
//	TH1 = 0XB2;
 	TH1=0x64;		//9600
//	TH1=0X98;  //4800bps
}

void UART_Init()
{
//	SCON0	=0x10; //原来的设置
	SCON0	=0X90; //9位数据
	SBRLL1	=0x8F;
	SBRLH1	=0xFD;
	SCON1	=0x10;
	SBCON1	=0x43;	
}



void	Port_IO_Init()
{
    	P0SKIP    = 0xCF;
    	XBR0      = 0x01;
    	XBR1      = 0x40;
    	XBR2      = 0x01;

}

void	Oscillator_Init()
{
	uchar i;
        OSCICN=0x83;
	CLKMUL=0x80;
    	Delay(255);
    	CLKMUL |=0xC0;
        do
        {       i=(CLKMUL &0x20);
        }while(!i);
/*Osc_Wait2:
    mov  A,         CLKMUL
    jnb  ACC.5,     Osc_Wait2*/
}

void	Interrupts_Init()
{
    	EIE1 =000;
    	EIE2 =002;    //EIE2 =002;
   	IE =0x92;
}



void Init_Device()
{
	Timer_Init();
    	UART_Init();
    	//SPI_Init();
    	Port_IO_Init();
    	Oscillator_Init();
    	Interrupts_Init();
}

void Delay(uchar n)
{
	uchar i;
	for(i=0;i<n;i++);
}

void Delay_ms(uint n)
{
	uint i;
	for(i=0;i<n;i++)Delay(80);
}

void clear_wdt() /*清看门狗*/
{
        EA=0;
        ReadEE(0xff,1,RTC_BUF);
        EA=1;
}

/*定时器0 中断服务程序,用于1MS */
void Service_Timer0() interrupt 1	using 1
{
	TL0 = TL0_VAL;	
	TH0 = TH0_VAL;
	TIME_1MS = 1;

	flag_1ms++;
	interval_ms++;
	/*
	if(sreceive)
	{
		if(receivetime >8 )
		{
			sreceive = 0;
		 	receivetime = 0;
		 	OE0 = 1;
			REN0 = 1;
			TI0 = 0;
			receive_length =0;
		}
		else 	receivetime ++;
	}
	*/
	if(sreceive_1)
	{
		if(receivetime_1 > 4)
		{
			sreceive_1 = 0;
		 	receivetime_1 = 0;
		 	OE1 = 1;
			SCON1 |=0x10;
			SCON1&=~(0x02);
		}
		else 	receivetime_1++;
	}

}

/********** added by jin******************************************/

/***********************
* Modbus send :设置发送标志位，其中length 表示要发送的字节数
*			   PermitTrans=1表示总线空闲，允许发送
*			   readySend=1时表示有数据要发送
*		！！！   发送前写好BUF_RECE    ！！！
************************/
void Modbus_Send(uint8 length)
{
	COM0_length = length;
	PermitTrans = 0;
	readySend = 1;	
}

/**********************************
* CRC_CHK:crc_16 校验程序  *pData为要校验的数组
*						   nLen 表示要校验的长度
*！！！	      使用时需要注意发送时低位在前还是高位在前	 ！！！
***********************************/
// uint16 crc_chk(uint8 *pData,uint8 nLen)
// {
// uint16 temp_crc=0xffff,temp1,i,j;
//  for(i=0;i<nLen;i++)
//     {
//     temp_crc^=*(pData+i);
// 	for(j=0;j<8;j++)
// 		{
// 		temp1=temp_crc;
// 		temp_crc>>=1;
// 		if(temp1&0x0001)
// 			temp_crc^=0xa001;
//         }
//     }
// return(temp_crc);
// }												  


uint8 code auchCRCHi[]=
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

uint8 code auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

uint16 crc_chk(uint8 *pData,uint8 nLen)
{
  uint8 uchCRCHi=0xff;
  uint8 uchCRCLo=0xff;
  uint16  uindex;
  while(nLen--)
  {
		uindex=uchCRCHi^*pData++;
		uchCRCHi=uchCRCLo^auchCRCHi[uindex];
		uchCRCLo=auchCRCLo[uindex];
  }
  return (((uint16) uchCRCLo)<<8|uchCRCHi);
}


/*************************************************************/
void init_Modbus_Reg(void)
{
	uint16 index;
	for(index=0;index<64;index++)
	{
		air_power[index]=0;
		air_mode[index] = 0;
		air_speed[index]= 0;
		actual_temperature[index] = 20;
		air_temperature[index] = 25;
	}

}

/*******	added @2012-12-14 	*************/
/*	the following codes are designed to 	*/
/*	convert LDS Protocol to Haier modbus 	*/
/*	Protoclol 								*/
/*******	added @2012-12-14 	*************/
void RQ_air_condi( uint16 air_num,uint8 reg_num )
{

//	requesting = 0,
//	sending,
//	ST_onoff,	   // 0 关 1开
//	ST_mode,		   // 0 自动 1送风 2制冷 3除湿 4制热
//	ST_temperature,// 16~30
//	ST_speed,		   // 0 自动 1低风 2中风 3高风
//	RQ_onoff,
//	RQ_mode,
//	RQ_temperature,
//	RQ_speed,
//	RQ_out_state
	uint16 address;
	if((air_num>64)||(reg_num>252)) 	return;
	if((reg_num==0))	return;
	
	BUF_RECE[0] = station_ip;
	switch(modbus_state)
	{
		case RQ_onoff:
		{
			BUF_RECE[1] = 0X02;		//功能码
			address = air_num*152;
			RQ_address = address;
			BUF_RECE[4] = 0;
			BUF_RECE[5] = 8;
			modbus_state = RQ_mode;				
			break;
		}
		case RQ_mode:
		{
			BUF_RECE[1] = 0X02;		//功能码
			address = air_num*152+8;
			RQ_address = address;
			BUF_RECE[4] = 0;
			BUF_RECE[5] = 8;
			modbus_state = RQ_temperature;				
			break;			
		}
		case RQ_temperature:
		{
			BUF_RECE[1] = 0X04;		//功能码
			address = air_num*156;
			RQ_address = address;
			BUF_RECE[4] = 0;
			BUF_RECE[5] = 1;
			modbus_state = RQ_speed;				
			break;		
		}
		case RQ_speed:
		{
			BUF_RECE[1] = 0X02;		//功能码
			address = air_num*152+16;
			RQ_address = address;
			BUF_RECE[4] = 0;
			BUF_RECE[5] = 8;			
			modbus_state = RQ_onoff;
			break;		
//					if(flag_inc)
//					{
//						air_RQ_num++;
//					}
//					for(;air_RQ_num<64;air_RQ_num++)
//					{
//						if(air_box_num[air_RQ_num]!=0)
//						{
//							flag_inc=1;
//							break;
//						}
//					}
//					if(air_RQ_num>=64)
//					{
//						air_RQ_num=0;
//					}			
		}
		default:
		{
			BUF_RECE[1] = 0X02;		//功能码
			address = air_num*152;
			RQ_address = address;
			BUF_RECE[4] = 0;
			BUF_RECE[5] = 8;
			modbus_state = RQ_mode;				
			break;
		}
	}
	BUF_RECE[2] = (uint8) (address>>8);
	BUF_RECE[3] = (uint8) address;
	RQ_address = address;
	address = crc_chk(BUF_RECE,6);
	BUF_RECE[7] = (uint8) (address>>8);
	BUF_RECE[6] = (uint8) address;
	Modbus_Send(7);
	
}	

void ST_air_condi(uint8 air_num,uint8 reg_num,uint8* param)
{		
//	requesting = 0,
//	sending,
//	ST_onoff,	   // 0 关 1开
//	ST_mode,		   // 0 自动 1送风 2制冷 3除湿 4制热
//	ST_temperature,// 16~30
//	ST_speed,		   // 0 自动 1低风 2中风 3高风
//	RQ_onoff,
//	RQ_mode,
//	RQ_temperature,
//	RQ_speed,
//	RQ_out_state
	uint8	i,j;
	uint8 	get_param[10];
 	uint16 address,temperature;
	if((air_num>64)||(reg_num>252))
	{
		RQ_ST = 0;
		return;
	}
	if((reg_num==0))
	{
		RQ_ST = 0;
		return;
	}
	BUF_RECE[0] = station_ip;
//	if((modbus_state==ST_onoff)&&(shadow_power[i] !=1))
//	{
//		
//	}
  switch(modbus_state)
	{
		case ST_onoff:
		{
			BUF_RECE[1] = 0X0f;
			address = 152*air_num;
			//此处需要填写数据
			if(param[0] ==0)
			{
				RQ_ST = 0;	
			}
//			shadow_power[i] = 0;
			BUF_RECE[2] = (uint8) (address>>8);
			BUF_RECE[3] = (uint8) address;
			BUF_RECE[4] = 0;			
			BUF_RECE[5] = 0X08;
			BUF_RECE[6] = 0X01;
			BUF_RECE[7] = *param;
			address = crc_chk(BUF_RECE,8);
			BUF_RECE[8] = (uint8) address;
			BUF_RECE[9] = (uint8) (address>>8);
			Modbus_Send(9);			
			modbus_state = ST_mode;//如果是开的状态，设置模式
//		modbus_state = state_copy;//如果是关的状态，直接回查询状态的模式
			break;				
		}
		case ST_mode:
		{
			BUF_RECE[1] = 0X0f;
			address = 152*air_num+8;
			BUF_RECE[2] = (uint8) (address>>8);
			BUF_RECE[3] = (uint8) address;
			BUF_RECE[4] = 0;				
			BUF_RECE[5] = 0X08;
			BUF_RECE[6] = 0X01;
			BUF_RECE[7] = *(param+1);
			address = crc_chk(BUF_RECE,8);
			BUF_RECE[8] = (uint8) address;
			BUF_RECE[9] = (uint8) (address>>8);
			Modbus_Send(9);	
			modbus_state = ST_temperature;
			break;
		}
		case ST_temperature:
		{
			BUF_RECE[1] = 0X06;
			address = 156*air_num;
			BUF_RECE[2] = (uint8) (address>>8);
			BUF_RECE[3] = (uint8) address;
			temperature = *(param+2);
			if(temperature>20)
			{
				temperature = temperature*50;
				temperature |=0x0800;
			}
			else
			{
				temperature = temperature*100;
			}
			BUF_RECE[5] = (uint8) temperature;
			BUF_RECE[4] = (uint8) (temperature>>8);			
			address = crc_chk(BUF_RECE,6);
			BUF_RECE[6] = (uint8) address;
			BUF_RECE[7] = (uint8) (address>>8);
			Modbus_Send(7);	 
			modbus_state = ST_speed;
			break;
		}
		case ST_speed:
		{
			RQ_ST = 0;
			BUF_RECE[1] = 0X0f;
			address = 152*air_num+16;
			BUF_RECE[2] = (uint8) (address>>8);
			BUF_RECE[3] = (uint8) address;
			BUF_RECE[4] = 0;				
			BUF_RECE[5] = 0X08;
			BUF_RECE[6] = 0X01;
			BUF_RECE[7] = *(param+3);
			address = crc_chk(BUF_RECE,8);
			BUF_RECE[8] = (uint8) address;
			BUF_RECE[9] = (uint8) (address>>8);
			Modbus_Send(9);	
			break;
		}
		default:
		{
//			RQ_ST = 0;
			modbus_state = ST_onoff;
//			modbus_state = state_copy;
			return;
		}
	}



}

void process_received( void )
{
//	RQ_onoff,
//	RQ_mode,
//	RQ_temperature,
//	RQ_speed,
//	ST_onoff,	   // 0 关 1开
//	ST_mode,		   // 0 自动 1送风 2制冷 3除湿 4制热
//	ST_temperature,// 16~30
//	ST_speed,		   // 0 自动 1低风 2中风 3高风
	
// uint16 	xdata air_power[64];
// uint16 	xdata air_temperature[64];
// uint16 	xdata air_mode[64];
// uint16 	xdata air_speed[64];
// uint16 	xdata actual_temperature[64];	
	
//	由于查询后直接将状态机修改成下一状态，接收时查询更新上衣状态的值	
	uint8 i;
	uint16 address;
	if(BUF_RECE[1] == 0X02)
	{
		if((Modbus_Rcvd_Num-5)!=(BUF_RECE[2])) return;
		switch(modbus_state)
		{
			case RQ_onoff:		//实际为等待风速的状态
			{
				i = BUF_RECE[3];
				switch(i)
				{
					case 1:				//风速关
					{
						air_speed[RQ_address/152] = 0;
						break;
					}
// 					case 2:				//风速自动
// 					{
// 						air_speed[RQ_address/152] = 4;
// 						break;
// 					}		
					case 3:				//风速高
					{
						air_speed[RQ_address/152] = 3;
						break;
					}	
					case 4:				//风速中
					{
						air_speed[RQ_address/152] = 2;
						break;
					}	
					case 5:				//风速低
					{
						air_speed[RQ_address/152] = 1;
						break;
					}		
					case 6:				//风速低
					{
						air_speed[RQ_address/152] = 1;
						break;
					}	
					default:
						air_speed[RQ_address/152] = 1;
						break;
				}
			}
			case RQ_mode:		//实际为查询开关机状态
			{
				air_power[RQ_address/152] = BUF_RECE[3];
				break;
			}
			case RQ_temperature://实际为查询模式
			{
				i = BUF_RECE[4];
				switch(i)
				{
					case 1:				//模式加热
					{
						air_mode[RQ_address/152] = 3;
						break;
					}
					case 2:				//模式制冷
					{
						air_mode[RQ_address/152] = 2;
						break;
					}		
					default:				//模式干燥，当送风模式
					{
						air_speed[RQ_address/152] = 1;
						break;
					}		
				}					
			}
		}
	}
	else if(BUF_RECE[1] == 0X04)
			{	
				if(BUF_RECE[2] != ((Modbus_Rcvd_Num-5)))
					return;
				if(modbus_state == RQ_speed)
				{
					address = (uint16) (BUF_RECE[3]<<8)|BUF_RECE[4];
					if((address&0x8000)!=0x8000)
					{
						if((address&0x7800)!=0)
						{
							address &= 0x07ff;
							actual_temperature[RQ_address/156] = (uint8) (address/50);
						}
						else
						{
							actual_temperature[RQ_address/156] = (uint8) (address/100);
						}
					}
					else
					{
						actual_temperature[RQ_address/156] = 0;
					}
				}
			}
}


void main()
{
	unsigned char 	data  i,j;
	unsigned char 	air_box_index,RQ_TIMES;
//	uint8 temp_mdbs_data;
//	uint8 sv_data[2];
	uint16 buf_index;
	
//	uint16 modbus_reg_addr,modbus_rqst_len;
	


	//struct	Task_d   xdata *Task_point;

    PCA0MD &= ~(0x40);      // clear Watchdog Enable bit
	Init_Device();
    init_Modbus_Reg();   
	TR1	= ON;
	TR0	= ON;
	EA	= ON;
	OE0	= ON;
	RI0	= ON;
        OE1	= ON;
        SCON1  |=0x01;
		
//		flag_first_run = 0;        
		status_device_first_run = 1;
        for(box=0;box<255;box++)
        {
        	Delay_ms(60);
        }
		/*
		for(box=0;box<5;box++)
		{
			air_box[box] = box;
			power_ip[box] = box;	
		}
		air_box[5] = 0xff;
		power_ip[5] = 0xff;
		*/
//		WriteEE(0x360,6,air_box);
//		WriteEE (0x370,6,power_ip);
		       
	        ReadEE(0x01,1,RTC_BUF);
	        box = RTC_BUF[0];
	    	ReadEE(0x235,3,RTC_BUF);
/**********************************************************************/
/*	EEPROM中地址0x235 modbus从机的地址  				 			  */
/*	  			0x400 海尔内机对应的box号				 			  */
/**********************************************************************/
		station_ip = RTC_BUF[0];
		control_mode = RTC_BUF[1];
		poling_mode = RTC_BUF[2];
		ReadEE(0x400,64,air_box_num);
//		control_mode = 0;//广播模式或者是单控模式
		if(control_mode == 0xff)
			status_no_other_dakin_vrv = 1;
		else 
			status_no_other_dakin_vrv = 0;
//		ReadEE(0x360,16,air_box);
//		ReadEE(0x370,16,VRV_ip);
//		for(i=0;i<16;i++)
//		{
//			if((VRV_ip[i]<=0x4f)&&(VRV_ip[i]>0x0f))
//			/****************************************/
//			/*		VRV空调地址为1-00到4-15			*/	
//			/****************************************/
//				status_vrv_pointed = 1;
//		}

//	station_ip  = 1;
//	RTC_BUF[0] = 1;
//	ReadEE(0x351,1,RTC_BUF);
//  station_ip = RTC_BUF[0];
//	station_ip = 0x01;
//	svd_air_param[0][2] = 0xf0cc;
	    
	BUF_TRCE1[0] = 0xfa;
	BUF_TRCE1[1]=  DEVICE_CODE;
	BUF_TRCE1[2]=  box;
	BUF_TRCE1[3]=  0xfe;
	BUF_TRCE1[4]=  0xef;		//modbus 转换器
	BUF_TRCE1[5]=  0x11/*ver*/;
	BUF_TRCE1[6]=  0x80;
	trans_data1();
	
	/*********
	ver :	0x10 dakin modbus
	
	************/
	
	
	time20ms=10;
	
	second	=50;
		

	
	PCA0CN    = 0x40;
	
	PCA0L    = 0x00;               	// Set lower byte of PCA counter to 0  
	PCA0H    = 0x00;               	// Set higher byte of PCA counter to 0
	
	PCA0CPL4 = 0xFF;     // Write offset for the WDT 
	
//	PCA0MD |= 0x60;      // enable Watchdog Enable bit
		
	PCA0CPH4 = 0xff;
	
	COM0_length = 7;

//	initial_reg_len();
//	request_preset();
	PCA0MD |= 0x60;      // enable Watchdog Enable bit 

//01 03 01 00 00 08 45 F0
//01 03 10 00 08 82 00 00 15 00 01 16 82 96 00 00 00 00 02 99 98 
//	if(status_no_other_dakin_vrv ==0)
//	{
//		BUF_RECE[0] = station_ip;
//		BUF_RECE[1] = 0X06;
//		BUF_RECE[2] = 0X00;
//		BUF_RECE[3] = 0X00;
//		BUF_RECE[4] = 0X81;
//		BUF_RECE[5] = 0X0f;
//		CRC_result  = crc_chk(BUF_RECE,6);
//		BUF_RECE[6] = (uint8) CRC_result;
//		BUF_RECE[7] = (uint8) (CRC_result>>8);
//		trans_data(); 
//		while(!OE0){;}
//		
//		RTC_BUF[0] = 0XFF;
//		WriteEE(0x351,1,RTC_BUF);	
//	}
	roll_poling = 0;
	roll_flag = 1;
	               			
	do						   
	{ 
/*************************  added by jin ******************************/
/**************  超时，表示接收完成或是允许发送*******************/
		if(flag_1ms>=6)//数字代表毫秒
		{
			if(sreceive)
			{
				sreceiveend = 1;
				PermitTrans = 0;
				sreceive = 0;
				REN0=0;
			}	
			else 
			{
				sreceiveend = 0;
				PermitTrans = 1;
				REN0 = 1;				
			}
			flag_1ms=0;	
		}
/*********************超时，表示接收完成或是允许发送************************/


		if(sreceiveend)			  //串口0接收到了modbus命令
		{
			sreceiveend = OFF;
			/************	此处是LDS的checkSum modbus不用，屏蔽之	***************
			j=BUF_RECE[0];
			for(i=1;i<receive_length;i++)
			{
				j=j^BUF_RECE[i];
			}
			if(!j)	receive_data();
			/******************************/
/************************* added by jin **********************************/
/**********************************接收完成并做校验******************************/
			//海林的面板校验数据时低位先发送，高位后发送的
			CRC_result = (BUF_RECE[Modbus_Rcvd_Num-1]<<8)|BUF_RECE[Modbus_Rcvd_Num-2];

			if(CRC_result==crc_chk(BUF_RECE,Modbus_Rcvd_Num-2))
			{
//				if((BUF_RECE[2] == Modbus_Rcvd_Num-5)&&(BUF_RECE[2]%2 == 0))		 //接收到的数据长度是否符合
//				此处的if表示接收到的modubs数据长度是正确的，在这个if下面是处理程序
					process_received();
				REN0=1;
				
//				if(status_ctrl == 0)
				{
					interval_ms = 0;
				}
			}
		}		  

		//开始发送的程序
 		if((PermitTrans==1)&&(readySend==1))	 //发送标志位都为一时，开始发送
		{
			PermitTrans=0;
			readySend=0;
	  		trans_data();
			
		}

/**************************************开始发送的程序*****************************************/	
		
		if(sreceiveend_1)
		{
			sreceiveend_1 = OFF;
			receive_data1();
			SCON1 |=0x10;
		}

                if(receive_length==40)
                {
                	     	   	
                }		        
		
		
		while(TIME_1MS)
		{
			TIME_1MS = 0;
			
			PCA0CPH4 = 0xff;
			
			
			if(!(--time20ms_out))
			{

				time20ms_out = 10;
				
				
				
			}
				
			
			//PROC_RELAY_OUTPUT();	
			
			if(!(--time20ms))
			{

				time20ms = 10;
				if(tranready_1)
				{
					tranready_1=0;
					trans_data1();
				}
				
							
				
	/*service按钮处理程序,service按钮释放:servicesw=0,service按钮按下:servicesw=1,	*/		
				service_led =1;
				Delay(2);
				if(!service_led)
				{
					if(!servicesw)
					{	
						servicesw=1;
						while(!OE1);
						BUF_TRCE1[0] = 0xfa;
						BUF_TRCE1[1]=  DEVICE_CODE;
						BUF_TRCE1[2]=  box;
						BUF_TRCE1[3]=  0xfe;
						BUF_TRCE1[4]=  0;
						BUF_TRCE1[5]=  ver;
						BUF_TRCE1[6]=  0;
						set_tranready1();
					}	
				}
				else
				{
					servicesw=0;
					if(led_status)service_led =1;
					else 	service_led =0;
				}
				
				if(!second--)
				{
					second = 50;
					
					if(!setup_mode)clear_wdt();
					flag_1s^=1;
//					flag_1s = 1;

					if(flag_1s)	 
					{
						if(RQ_TIMES == 0)
						{
							RQ_TIMES = 4;
							air_box_index++;
							if(air_box_index>63)
								air_box_index=0;
							for(;air_box_index<64;i++)
							{
								if(air_box_num[air_box_index]!=0)
									break;
								else air_box_index++;
							}
						}	
						if(RQ_ST==0)
						{
							RQ_TIMES--;
							RQ_air_condi(air_box_index,1);
						}
						else
						{
							if(status_ctrl == 1)
							{
								status_ctrl=0;
								modbus_state = ST_onoff;
							}
							
							if((modbus_state==RQ_onoff)
								||(modbus_state==RQ_mode)
								||(modbus_state==RQ_temperature)
								||(modbus_state==RQ_speed)
								||(modbus_state==RQ_out_state) )
							{
								modbus_state = ST_onoff;
							}
							ST_air_condi(air_condi_num,1,Modbus_Reg);
						}


					}
					if((resend_msg)&&(OE0))
					{	
						if(!resend_msg_sec--)
						{	
						
							resend_msg_sec=10;	
							for(temp=0;temp < resend_msg_length;temp++)
							BUF_RECE[temp] = resend_msg_buf[temp] ;
							trans_data();
							if(!resend_msg_counter--) resend_msg = 0;
						
						}
					}
						
						
					if(TRFLAG)
					{
						TRFLAG = 0;
						//OE0 = 1;
						//REN0 =1;
						OE1 = 1;
						SCON1 |=0x10;
						EA = 1;
						
					}
					else	TRFLAG =1;
					if(serviceled)
					{	if(!netactivetime--) serviceled=0;
                                        }
					else	
					{
						if(led_status)
						{	
							service_led = 0;
							led_status=0;
						}
						else
						{
							service_led = 1;
							led_status=1;
						}		
					}
						
				}
				if(serviceled)
				{																	 
					if(second<25) 
					{	service_led =0;
						led_status=0;
					}
					else 
					{	service_led =1;
						led_status=1;
					}
				}
				
			}
			
		}
	}while(1);
	
}