#include "c8051F340.h"
#include "Main.h"
#include "cmd.h"

void Service_COM();
void Delay_tr();
void send_BUF_TRAN();

void receive_data();
void resend_msg_proc();




extern uchar netactivetime;

/****** added by jin  ****/
extern uint8 xdata flag_1ms; 
extern uint8 xdata Modbus_Rcvd_Num;
//extern uint8 xdata area[33];
extern uint8 xdata Modbus_Reg[85];
extern uint8 xdata	Reg_temp[50],air_box[16],VRV_ip[16],cmd_copy[20];
//extern uint16 xdata shift_addr[34];
//extern uint16 xdata svd_air_param[64][8];
extern uint16 xdata function_register[192];
extern uint16 xdata status_register[384];
extern uint8 xdata roll_poling,roll_flag,resend_times;
extern uint8 xdata station_ip,control_mode,actual_tempture[64];


uint8 xdata speed,mode,temperature;
/*************************/



void Service_COM() interrupt 4
{
	if( RI0)
	{
		RI0=0;
		flag_1ms = 0;
		if(!sreceive)
		{
//			if(((SBUF0>=0)&&(SBUF0<64))||(SBUF0==0xDC))	//	(SBUF0==0X01)	((SBUF0>=0)&&(SBUF0<64))||(SBUF0==0xDC)
			if(SBUF0 == station_ip)
			{
				sreceive=1;
				COM_buf_point = BUF_RECE;
				Modbus_Rcvd_Num = 0;
				receive_length =40;
			}
		}
//		else
		if(sreceive)
		{
			*COM_buf_point=SBUF0;
		   /********** added by jin  ************/
		   /*******    接收字符数    *******/
			Modbus_Rcvd_Num++;
		   /**********************/
			if(COM_buf_point==(BUF_RECE+receive_length-1))
			{
                sreceiveend=1;
				REN0=0;
				sreceive=0;
				
            }
            else COM_buf_point++;
		}
	}
	else
	{
		if( TI0)
		{
			TI0=0;
	/***********接收时不让REN=0? 	ADDED @2013-1-9***************/
			flag_1ms = 0;	 
	/***********接收时不让REN=0? 	ADDED @2013-1-9***************/
			if(COM_buf_point < (BUF_RECE+COM0_length))
			{
				COM_buf_point++;
			 /******* added by jin ********/
			 //Be used to add odd parity bit
				ACC = *COM_buf_point;
				TB80 = P;		//P为偶校验，~p为奇校验
			 /*************************/
				SBUF0=*COM_buf_point;

			}
			else
			{
				Delay_tr();
				OE0=1;
				REN0=1;
				sreceive=0;
			}
		}
	}
	receivetime=0;
	
}


void Service_COM1() interrupt 16
{
	unsigned char	data  i;
	if( SCON1&0x01 )
	{
		do
		{
			SCON1&=~(0x81);
			i= SBUF1;
			if(!sreceive_1)
			{
		//	COM1_buf_point = BUF_RECE1;
		//	*COM1_buf_point=SBUF1;
			
				if((i==0xfa)||(i==0xf5)||(i==0xfc))
				{
					sreceive_1=1;
					COM1_buf_point = BUF_RECE1;
				
				}
			  
			}
		
			
			if(sreceive_1)
			{
				*COM1_buf_point=i;

				if(COM1_buf_point==(BUF_RECE1+IN_LEN-1))
				{
                        		sreceiveend_1=1;
					SCON1&=~(0x10);;
					sreceive_1=0;
				
                       	 	}
                        	else COM1_buf_point++;
			}
			
		
		}while(	SCON1&0x01) ;
	}
	else
	{
		if( SCON1&0x02)
		{
			SCON1&=~(0x02);
			if(COM1_buf_trce_point < (BUF_TRCE1+IN_LEN-1))
			{
				COM1_buf_trce_point++;
				SBUF1=*COM1_buf_trce_point;

			}
			else
			{
				Delay_tr();
				OE1=1;
				SCON1 |=0x10;
				netactivetime = 6;
				serviceled = 1;
				

			}
		}
	}
	receivetime_1=0;
	TRFLAG=0;
}



void delay_tr()
{
	unsigned char j;
	{
		for(j=0;j<10;j++);

	 }
}

void trans_data1()
{
	unsigned char i,j;
	COM1_buf_trce_point = BUF_TRCE1;
	if((*COM1_buf_trce_point == 0xfa)||(*COM1_buf_trce_point == 0xf5)||(*COM1_buf_trce_point == 0xfc))
	{
		SCON1&=~(0x10);
		OE1 = 0;
                j=0;
		for(i=0;i<IN_LEN-1;i++)
			j=j+BUF_TRCE1[i];
		j = ~j;
		BUF_TRCE1[IN_LEN-1]=j+1;
		SBUF1 = *COM1_buf_trce_point;
		
	}
}

void set_tranready1()
{
	time20ms = time20ms+6;
	tranready_1 =1;
}

void trans_data()
{
	//unsigned char i;
	uint16 set_temp;
	COM_buf_point = BUF_RECE;
	REN0 = 0;
	RI0=0;
	OE0 = 0;
	ACC = *COM_buf_point;
	TB80 = P;			//P为偶校验，~p为奇校验
	SBUF0 = *COM_buf_point;
	
	/*
	if(*COM_buf_point == 0x32)
	{
		REN0 = 0;
		OE0 = 0;
        SBUF0 = *COM_buf_point;
	
	} */
}

void set_tranready()
{
	time20ms_out = time20ms_out+6;
	tranready =1;
}


void receive_data1()
{
	unsigned char data i,j,temp1,temp2,temp3;
        unsigned int data k;
        unsigned char 	 	*eeprom_point;
		uint16 temp_param;
	
	j=0;
	for(i=0;i<IN_LEN;i++)
		j=j+BUF_RECE1[i];
	if(!j)
	{
		netactivetime = 6;
		serviceled = 1;
		if((BUF_RECE1[0]==0xfa)/*&&(BUF_RECE1[1]==DEVICE_CODE)*/)
		{
			if((BUF_RECE1[2]==box)&&(BUF_RECE1[1]==DEVICE_CODE))
			{	j=BUF_RECE1[3];
			
				switch(j)
				{
					case 0xfb:		/* command = read EEPROM */
					{
						BUF_TRCE1[3]= 0xf9;
						if(BUF_RECE1[5]||BUF_RECE1[4])
						{
							ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,RTC_BUF);
							BUF_TRCE1[6]= RTC_BUF[0]; 
						}
						else BUF_TRCE1[6]= DEVICE_CODE;
						BUF_TRCE1[0]= BUF_RECE1[0];	
						BUF_TRCE1[1]= BUF_RECE1[1];	
						BUF_TRCE1[2]= BUF_RECE1[2];	
						BUF_TRCE1[4]= BUF_RECE1[4];
						BUF_TRCE1[5]= BUF_RECE1[5];	
						set_tranready1();
					}
					break;
					case 0xfa:		/* command = Write EEPROM */
					{
						BUF_TRCE1[3]= 0xf9;
						
						RTC_BUF[0]=BUF_RECE1[6];
						WriteEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,RTC_BUF);
						while(ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,BUF_RECE1));
						ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,RTC_BUF);
						BUF_TRCE1[6]= RTC_BUF[0];
						BUF_TRCE1[0]= 0xfa;	
						BUF_TRCE1[1]= BUF_RECE1[1];	
						BUF_TRCE1[2]= BUF_RECE1[2];	
						BUF_TRCE1[4]= BUF_RECE1[4];
						BUF_TRCE1[5]= BUF_RECE1[5];
						set_tranready1();
								
					}
					break;
					case 0xBA:		/* command = Request ver */
					{
						BUF_TRCE1[0]= BUF_RECE1[0];	
						BUF_TRCE1[1]= BUF_RECE1[1];	
						BUF_TRCE1[2]= BUF_RECE1[2];	
						BUF_TRCE1[4]= 0xf0;		//VRV air
						BUF_TRCE1[6]= BUF_RECE1[6];
						BUF_TRCE1[3]= 0xFE;
						BUF_TRCE1[5]= ver; 
						set_tranready1();
					}
					break;
					case 0xFC:		/* command = Reboot */
					{
								RSTSRC |=0x10;
					}
					break;
					case 0xFD:		/* command = Listen to me */
					{
					if(BUF_RECE1[5]) setup_mode =1;
								else setup_mode =0;
					}
					break;	
					case 0xDF:		/* command = enable Block read */
					{
						BUF_TRCE1[0]= BUF_RECE1[0];	
						BUF_TRCE1[1]= BUF_RECE1[1];	
						BUF_TRCE1[2]= BUF_RECE1[2];	
						BUF_TRCE1[4]= BUF_RECE1[4];
						BUF_TRCE1[5]= BUF_RECE1[5];
						BUF_TRCE1[6]= BUF_RECE1[6];
						BUF_TRCE1[3]=  0xDE;       	
						set_tranready1();
			    }
			    break;
					case 0xDE:		/* command = Block read */
					{
						BUF_TRCE1[0]=  0xFC;
						eeprom_point = &BUF_TRCE1[1];
						ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),6,eeprom_point);
						set_tranready1();
					}
					break;
					case 0xDD:		/* command = Block writer */
					{
			
						block_writer = 1;
						eerom_address = BUF_RECE1[4]*256 + BUF_RECE1[5];
						//RTC_BUF[0]=BUF_RECE[4];/* Block writer = high */
						//RTC_BUF[1]=BUF_RECE[5];/* Block writer = low */
						BUF_TRCE1[0]= BUF_RECE1[0];	
						BUF_TRCE1[1]= BUF_RECE1[1];	
						BUF_TRCE1[2]= BUF_RECE1[2];	
						BUF_TRCE1[4]= BUF_RECE1[4];
						BUF_TRCE1[5]= BUF_RECE1[5];
						BUF_TRCE1[6]= BUF_RECE1[6];
						BUF_TRCE1[3]=  0xDC;
						set_tranready1();
					}
					break;	

					case 0x8f:
					{
//						处理空调控制命令
						if(RQ_ST==0)
						{
							for(i=0;i<64;i++)
							{
								if(air_box_num[i] == BUF_RECE1[2])
									break;									
							}
							if(i>=64)
								return;
//							process_air_cmd();
							air_condi_num = i;
							BUF_TRCE1[0] = BUF_RECE1[0];
							BUF_TRCE1[1] = BUF_RECE1[1];
							BUF_TRCE1[2] = BUF_RECE1[2];
							BUF_TRCE1[3] = 0x9b;
							BUF_TRCE1[4] = BUF_RECE1[4];
							BUF_TRCE1[5] = BUF_RECE1[5];
							if((BUF_RECE1[4] == 0))
							{								
//								if(BUF_RECE1[5] != 0)
								{
									Modbus_Reg[0] = 0;//开关机状态
									air_power[i] = 0;
//								shadow_power[i] =1;
									Modbus_Reg[1] = 0; //模式
									air_mode[i] = 0;
									air_speed[i] = 0;
									Modbus_Reg[2] = 0; //温度
									Modbus_Reg[3] = 1;	//风速
									BUF_TRCE1[5] = (uint8) air_temperature[i];
									BUF_TRCE1[6] = (uint8) actual_temperature[i];
								}
							}
							else
							{
								Modbus_Reg[0] = 1; //开关机状态
								air_power[i] = 1;
								j=(BUF_RECE1[4]&0Xf0)>>4;
								switch(j)
								{
									case 1:
									{
										Modbus_Reg[3] = 5;	//风速低
										break;
									}
									case 2:
									{
										Modbus_Reg[3] = 4;	//风速中
										break;
									}									
									case 3:
									{
										Modbus_Reg[3] = 3;	//风速高
										break;
									}				
									default:
									{
										Modbus_Reg[3] = 2;	//风速自动
										break;
									}
								}
								air_speed[i] = 0;
								j = BUF_RECE1[4]&0x0f;
								switch(j)
								{
									case 1:
									{

										Modbus_Reg[1] = 4; //模式：送风
										air_mode[i] = 4;
										break;
									}
									case 2:
									{
										Modbus_Reg[1] = 2; //模式：制冷
										air_mode[i] = 2;
										break;									
									}
									case 3:
									{
										Modbus_Reg[1] = 1; //模式：制热
										air_mode[i] = 1	;
										break;									
									}
									default:
									{
										Modbus_Reg[1] = 5; //模式：自动
										air_mode[i] = 5;
									}										
								}
								BUF_RECE1[5] &= (~0XC0);
								if(BUF_RECE1[5]>30)
								{
//									break;
									Modbus_Reg[2] = 30;	//温度
									air_temperature[i] = 30;	 
								}
								else if(BUF_RECE1[5]<16)
								{
//									break;
									Modbus_Reg[2] = 16;	//温度
									air_temperature[i] = 16;
								}
								else
								{
									Modbus_Reg[2] = BUF_RECE1[5];
									air_temperature[i] =  Modbus_Reg[5];
								}
							BUF_TRCE1[6] = (uint8) actual_temperature[i];
							}
//							set_tranready1();
							if(BUF_RECE1[5]!=0)
							{
								status_ctrl =1;
								RQ_ST = 1;
							}
						}
						break;						
					}

					case 0x9c:
					{
//						处理空调查询命令，返回模式，温度，设置温度和实际温度
						for(i=0;i<64;i++)
						{
							if(air_box_num[i] == box)
								break;								
						}
						if(i>=64)
							return;
						BUF_TRCE1[0] = BUF_RECE1[0];
						BUF_TRCE1[1] = BUF_RECE1[1];
						BUF_TRCE1[2] = BUF_RECE1[2];
						BUF_TRCE1[3] = 0x9b;
						if(air_power[i] ==0)
							BUF_TRCE1[4] = 0;
						else
						{
							switch(air_mode[i])
							{
								case 0:
									BUF_TRCE1[4] = 0;
									break;
								case 1:
									BUF_TRCE1[4] |= 1;
									break;
								case 2:
									BUF_TRCE1[4] |= 2;
									break;
								case 3:
									BUF_TRCE1[4] |= 3;
									break;
								default:
									BUF_TRCE1[4] =0;
									break;										
							}
							BUF_TRCE1[4] |= (air_speed[i]<<4);
						}
						BUF_TRCE1[5] = (uint8) air_temperature[i];
						BUF_TRCE1[6] = (uint8) actual_temperature[i] ;
						set_tranready1();
						break;
					}
					default:break;
				
				}
			}
			else 
				if(BUF_RECE1[1]==0xb8/*DEVICE_CODE*/)
				{
//					处理空调命令
					if(BUF_RECE1[3] == 0X8F)
					{
						if(RQ_ST==0)
						{
							for(i=0;i<64;i++)
							{
								if(air_box_num[i] == BUF_RECE1[2])
									break;									
							}
							if(i>=64)
								return;
//							process_air_cmd();
							air_condi_num = i;
							BUF_TRCE1[0] = BUF_RECE1[0];
							BUF_TRCE1[1] = BUF_RECE1[1];
							BUF_TRCE1[2] = BUF_RECE1[2];
							BUF_TRCE1[3] = 0x9b;
							BUF_TRCE1[4] = BUF_RECE1[4];
							BUF_TRCE1[5] = BUF_RECE1[5];
							if(BUF_RECE1[4] == 0)
							{								
								Modbus_Reg[0] = 0;//开关机状态
								air_power[i] = 0;
//								shadow_power[i] =1;
								Modbus_Reg[1] = 0; //模式
								air_mode[i] = 0;
								air_speed[i] = 0;
								Modbus_Reg[2] = 0; //温度
								Modbus_Reg[3] = 1;	//风速
								BUF_TRCE1[5] = (uint8) air_temperature[i];
								BUF_TRCE1[6] = (uint8) actual_temperature[i];
							}
							else
							{
								Modbus_Reg[0] = 1; //开关机状态
								air_power[i] = 1;
								j=(BUF_RECE1[4]&0Xf0)>>4;
								switch(j)
								{
									case 1:
									{
										Modbus_Reg[3] = 5;	//风速低
										break;
									}
									case 2:
									{
										Modbus_Reg[3] = 4;	//风速中
										break;
									}									
									case 3:
									{
										Modbus_Reg[3] = 3;	//风速低
										break;
									}				
									default:
									{
										Modbus_Reg[3] = 2;	//风速自动
										break;
									}
								}
								air_speed[i] = 0;
								j = BUF_RECE1[4]&0x0f;
								switch(j)
								{
									case 1:
									{

										Modbus_Reg[1] = 4; //模式：送风
										air_mode[i] = 4;
										break;
									}
									case 2:
									{
										Modbus_Reg[1] = 2; //模式：制冷
										air_mode[i] = 2;
										break;									
									}
									case 3:
									{
										Modbus_Reg[1] = 1; //模式：制热
										air_mode[i] = 1	;
										break;									
									}
									default:
									{
										Modbus_Reg[1] = 5; //模式：自动
										air_mode[i] = 5;
									}
										
								}
								BUF_RECE1[5] &= (~0X80);
								if(BUF_RECE1[5]>30)
								{
									Modbus_Reg[2] = 30;	//温度
									air_temperature[i] = 30;	 
								}
								else if(BUF_RECE1[5]<16)
								{
									Modbus_Reg[2] = 16;	//温度
									air_temperature[i] = 16;
								}
								else
								{
									Modbus_Reg[2] = BUF_RECE1[5];
									air_temperature[i] =  Modbus_Reg[5];
								}
							//	BUF_TRCE1[5] = BUF_RECE1[5];
								BUF_TRCE1[6] = (uint8) actual_temperature[i];
							}
//							set_tranready1();
							if(BUF_RECE1[5] !=0)
							{
								status_ctrl =1;
								RQ_ST = 1;
							}
						}
					}
					else
					{
//						应答空调查询命令
						if(BUF_RECE1[3] == 0X9C)
						{
							for(i=0;i<64;i++)
							{
								if(air_box_num[i] == BUF_RECE1[2])
									break;								
							}
							if(i>=64)
								return;
							BUF_TRCE1[0] = BUF_RECE1[0];
							BUF_TRCE1[1] = BUF_RECE1[1];
							BUF_TRCE1[2] = BUF_RECE1[2];
							BUF_TRCE1[3] = 0x9b;
							if(air_power[i] ==0)
								BUF_TRCE1[4] = 0;
							else
							{
								switch(air_mode[i])
								{
									case 0:
										BUF_TRCE1[4] = 0;
										break;
									case 1:
										BUF_TRCE1[4] |= 1;
										break;
									case 2:
										BUF_TRCE1[4] |= 2;
										break;
									case 3:
										BUF_TRCE1[4] |= 3;
										break;
									default:
										BUF_TRCE1[4] =0;
										break;										
								}
								BUF_TRCE1[4] |= (air_speed[i]<<4);
							}
							BUF_TRCE1[5] = (uint8) air_temperature[i];
							BUF_TRCE1[6] = (uint8) actual_temperature[i] ;
							set_tranready1();
						}
					}
				}

		}
		else
		{	
			if((BUF_RECE1[0]==0xf5)&&(BUF_RECE1[3]==0xFE))
			{
		
			}	
			else
			{
				if((BUF_RECE1[0]== 0xfc)&& block_writer)
				{	
					eeprom_point = &BUF_RECE1[1];
					i = eerom_address % 256;
					j = i-(i >> 4)*16+5;
					if(j<16)
			         		WriteEE(eerom_address,6,eeprom_point);
			         	else
			         	{
			         		i= j-15;
			         		j = 6-i;
			         		WriteEE(eerom_address,j,eeprom_point);
			         			
						while(ReadEE(eerom_address,1,BUF_RECE1));
                                                
			         		k = eerom_address +j;
			         		eeprom_point =eeprom_point +j;
			         		WriteEE(k,i,eeprom_point);
			         			
					
			         	}		
					BUF_TRCE1[0]= 0xFA;
					BUF_TRCE1[1]= DEVICE_CODE;
					BUF_TRCE1[2]= box;
					BUF_TRCE1[3]=0xdc;
					BUF_TRCE1[4]=eerom_address/256;
					BUF_TRCE1[5]=eerom_address%256;
					BUF_TRCE1[6]= 0;	
					eerom_address = eerom_address +6;
					set_tranready1();
				}
					
			}
		}


	}
}

/*
void receive_data()
{
	unsigned char data i;
        
	if((BUF_RECE[1]==0x34)&&(BUF_RECE[2]==0x30)&&(BUF_RECE[3]==0x30))
	{	//400 返回命令
		if(resend_msg_buf[4]==2) i=2;
		else 	if(resend_msg_buf[4]==1) i=1;
			else  i=0;
		
		returne_air_mode[i] = BUF_RECE[7]-0x30;	//0=fan 1= hot 2=cool
		returne_set_temperature[i] = (BUF_RECE[23]-0x30)*10+BUF_RECE[24]-0x30;
		returne_temperature[i] 	=(BUF_RECE[26]-0x30)*10+BUF_RECE[27]-0x30;
		returne_fan1_speed[i] 	=BUF_RECE[28]-0x30;
		returne_fan2_speed[i]	=BUF_RECE[29]-0x30;		
	
	}
        else 
	{	if((BUF_RECE[1]==0x41)&&(BUF_RECE[2]==0x54)&&(BUF_RECE[4]==0x21))
		{	//ATX 命令
			if(BUF_RECE[2]==0x30)
			{

			}
			else
			{
			  if(BUF_RECE[2]==0x31)
				{

				}
				else
				{
				       	if(BUF_RECE[2]==0x32)
					{

					}
					else
					{

					}
				}
			}
		}
                else  
		{	if((BUF_RECE[1]==0x36)&&(BUF_RECE[3]==0x30)&&(BUF_RECE[4]==0x03))
                	{ 	if(BUF_RECE[2]==0x30)
				{	//600 返回命令
					if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x30)&&(resend_msg_buf[3]==0x30))
						resend_msg =0;
				}
                	     	else  
				{	if(BUF_RECE[2]==0x31)
					{	//610 返回命令
						if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x31)&&(resend_msg_buf[3]==0x30))
							resend_msg =0;
					}
                	     	   	else
					{	if(BUF_RECE[2]==0x32)
						{	//620 返回命令
							if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x32)&&(resend_msg_buf[3]==0x30))
								resend_msg =0;
						}
                	     	   	      	else 
						{	if(BUF_RECE[2]==0x33)
							{	//630 返回命令
								if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x33)&&(resend_msg_buf[3]==0x30))
									resend_msg =0;

							}
                	     	   	      	   	else 
							{	if(BUF_RECE[2]==0x34)
								{	//640 返回命令
									if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x34)&&(resend_msg_buf[3]==0x30))
										resend_msg =0;
								}
							}
						}
					}
				}
			}
		}
	}
}
*/
/*
void send_BUF_TRAN()
{	
//	uchar data i,j;
	if(OE1)
	{
	/*	for( i=0; i< keynumber_all; i++)
		{
			j = BUF_TRAN[ i* 7];
			if((j == 0xfa)||(j == 0xf5)||(j == 0xfc))
			{
				BUF_RECE1[0] = j;
				BUF_TRAN[ i* 7] = 0xff;
				for( j = 1;j < 7; j++) 
					BUF_RECE1[j] = BUF_TRAN[ i* 7+j];
				set_tranready1();
				i = keynumber_all + 2;
			}
		}
                	
	}
				
}
*/


void resend_msg_proc()
{
	resend_msg_sec = 10;
	resend_msg_counter = 2;
	resend_msg = 1;
	for(temp=0;temp < 7;temp++)
		resend_msg_buf[temp] = BUF_RECE[temp];
	set_tranready();
}





				

			