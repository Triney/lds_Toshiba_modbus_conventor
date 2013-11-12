#include "c8051F340.h"
#include "Main.h"

void Service_COM();
void Delay_tr();
void send_BUF_TRAN();

void receive_data();
void resend_msg_proc();




extern  uchar netactivetime;




void Service_COM() interrupt 4
{
	if( RI0)
	{
		RI0=0;
		if(!sreceive)
		{
			if(SBUF0==0x02)
			{
				sreceive=1;
				COM_buf_point = BUF_RECE;
				receive_length =40;
			}
		}
		if(sreceive)
		{
			*COM_buf_point=SBUF0;

			if(COM_buf_point==receive_length-1)
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
			if(COM_buf_point < (BUF_RECE+COM0_length))
			{
				COM_buf_point++;
				SBUF0=*COM_buf_point;

			}
			else
			{
				Delay_tr();
				OE0=1;
				REN0=1;
			}
		}
	}
	receivetime=0;
	
}


void Service_COM1() interrupt 16
{
	if( SCON1&0x01 )
	{
		SCON1&=~(0x01);
		if(!sreceive_1)
		{
			if((SBUF1==0xfa)||(SBUF1==0xf5)||(SBUF1==0xfc))
			{
				sreceive_1=1;
				COM1_buf_point = BUF_RECE1;
				
			}
		}
		if(sreceive_1)
		{
			*COM1_buf_point=SBUF1;

			if(COM1_buf_point==BUF_RECE1+IN_LEN-1)
			{
                        	sreceiveend_1=1;
				SCON1&=~(0x10);;
				sreceive_1=0;
				
                        }
                        else COM1_buf_point++;
		}
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
	COM_buf_point = BUF_RECE;
	if(*COM_buf_point == 0x32)
	{
		REN0 = 0;
		OE0 = 0;
                SBUF0 = *COM_buf_point;
	
	}
}

void set_tranready()
{
	time20ms_out = time20ms_out+6;
	tranready =1;
}


void receive_data1()
{
	unsigned char data i,j;
        unsigned int data k;
        unsigned char 	 	*eeprom_point;
	
	j=0;
	for(i=0;i<IN_LEN;i++)
		j=j+BUF_RECE1[i];
	if(!j)
	{
		netactivetime = 6;
		serviceled = 1;
		if((BUF_RECE1[0]==0xfa)&&(BUF_RECE1[1]==DEVICE_CODE)&&(BUF_RECE1[2]==box))
		{
			j=BUF_RECE1[3];
			
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
		                      	BUF_RECE1[3]= 0xf9;
		                      	set_tranready1();
		                      	RTC_BUF[0]=BUF_RECE1[6];
		                      	WriteEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,RTC_BUF);
		                      	while(ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,BUF_RECE1));
		                      	ReadEE((BUF_RECE1[4]*256+BUF_RECE1[5]),1,RTC_BUF);
		                      	BUF_RECE1[6]= RTC_BUF[0];
		                      	BUF_TRCE1[0]= 0xfa;	
		                      	BUF_TRCE1[1]= BUF_RECE1[1];	
		                      	BUF_TRCE1[2]= BUF_RECE1[2];	
		                      	BUF_TRCE1[4]= BUF_RECE1[4];
		                      	BUF_TRCE1[5]= BUF_RECE1[5];
                              	}
                              	break;
                              	case 0xBA:		/* command = Request ver */
		              	{
		                      	BUF_TRCE1[0]= BUF_RECE1[0];	
		                      	BUF_TRCE1[1]= BUF_RECE1[1];	
		                      	BUF_TRCE1[2]= BUF_RECE1[2];	
		                      	BUF_TRCE1[4]= BUF_RECE1[4];
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
				default:
					
				break;
				
			}
		}
		else
		{
			if(BUF_RECE1[0]==0xf5)
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


void receive_data()
{
	//unsigned char data i,j;
        
	if((BUF_RECE[1]==0x34)&&(BUF_RECE[2]==0x30)&&(BUF_RECE[3]==0x30))
	{	//400 ·µ»ØÃüÁî
	
	
	}
        else 
	{	if((BUF_RECE[1]==0x41)&&(BUF_RECE[2]==0x54)&&(BUF_RECE[4]==0x21))
		{	//ATX ÃüÁî
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
				{	//600 ·µ»ØÃüÁî
					if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x30)&&(resend_msg_buf[3]==0x30))
						resend_msg =0;
				}
                	     	else  
				{	if(BUF_RECE[2]==0x31)
					{	//610 ·µ»ØÃüÁî
						if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x31)&&(resend_msg_buf[3]==0x30))
							resend_msg =0;
					}
                	     	   	else
					{	if(BUF_RECE[2]==0x32)
						{	//620 ·µ»ØÃüÁî
							if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x32)&&(resend_msg_buf[3]==0x30))
								resend_msg =0;
						}
                	     	   	      	else 
						{	if(BUF_RECE[2]==0x33)
							{	//630 ·µ»ØÃüÁî
								if((resend_msg_buf[1]==0x36)&&(resend_msg_buf[2]==0x33)&&(resend_msg_buf[3]==0x30))
									resend_msg =0;

							}
                	     	   	      	   	else 
							{	if(BUF_RECE[2]==0x34)
								{	//640 ·µ»ØÃüÁî
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
                */	
	}
				
}



void resend_msg_proc()
{
	resend_msg_sec = 10;
	resend_msg_counter = 2;
	resend_msg = 1;
	for(temp=0;temp < 7;temp++)
		resend_msg_buf[temp] = BUF_RECE[temp];
	set_tranready();
}





				

			