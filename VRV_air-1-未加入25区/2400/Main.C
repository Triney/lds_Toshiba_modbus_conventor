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
sbit TIME_2MS		=Flag3^1;



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


unsigned char 	xdata	BUF_RECE[64];
unsigned char 	data 	BUF_RECE1[IN_LEN];
unsigned char 	data 	BUF_TRCE1[IN_LEN];

unsigned char 	resend_msg_length=0;
unsigned char 	xdata	resend_msg_buf[64];
unsigned char 	resend_msg_sec=0;
unsigned char 	resend_msg_counter=0;
unsigned char 	temp=0;
unsigned char 	xdata	receive_length=0;
unsigned char 	xdata	receivetime =0;

void Timer_Init()
{
	TCON=0x10;
	TMOD=0x21;
	CKCON=000;
	TL0=0xA0;
	TH0=0xA0;
	TH1=0x30;
}

void UART_Init()
{
	SCON0	=0x10;
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

/*定时器0 中断服务程序,用于2MS */
void Service_Timer0() interrupt 1	using 1
{
	TL0 = TL0_VAL;	
	TH0 = TH0_VAL;
	TIME_2MS = 1;
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





void main()
{
	unsigned char 	data  i,j;
	//struct	Task_d   xdata *Task_point;

        PCA0MD &= ~(0x40);      // clear Watchdog Enable bit
	Init_Device();

        
	TR1	= ON;
	TR0	= ON;
	EA	= ON;
	OE0	= ON;
	RI0	= ON;
        OE1	= ON;
        SCON1  |=0x01;
        
        for(box=0;box<255;box++)
        {
        	Delay_ms(60);
        }
        
        ReadEE(0x01,1,RTC_BUF);
        box = RTC_BUF[0];
        
        
	BUF_TRCE1[0] = 0xfa;
	BUF_TRCE1[1]=  DEVICE_CODE;
	BUF_TRCE1[2]=  box;
	BUF_TRCE1[3]=  0xfe;
	BUF_TRCE1[4]=  0;
	BUF_TRCE1[5]=  ver+1;
	BUF_TRCE1[6]=  0x80;
	trans_data1();
	
	
	
	time20ms=10;
	
	second	=50;
		

	
	PCA0CN    = 0x40;
	
	PCA0L    = 0x00;               	// Set lower byte of PCA counter to 0  
	PCA0H    = 0x00;               	// Set higher byte of PCA counter to 0
	
	PCA0CPL4 = 0xFF;     // Write offset for the WDT 
	
	PCA0MD |= 0x60;      // enable Watchdog Enable bit
		
	PCA0CPH4 = 0xff;
	
	COM0_length = 4;
                			BUF_RECE[0]=0x32;
                			BUF_RECE[1]=0xec;
                			//BUF_RECE[1]=0xee;
                			BUF_RECE[2]=0xaa;
                			BUF_RECE[3]=0x80;
                			BUF_RECE[4]= BUF_RECE[1] +BUF_RECE[2]+BUF_RECE[3];
                			trans_data();
                			while(!OE0)
                			
	do
	{
		if(sreceiveend)
		{
			sreceiveend = OFF;
			j=BUF_RECE[0];
			for(i=1;i<receive_length;i++)
			{
				j=j^BUF_RECE[i];
			}
			if(!j)	receive_data();
				
			REN0=1;;
		}
		
		
		
		if(sreceiveend_1)
		{
			sreceiveend_1 = OFF;
			receive_data1();
			SCON1 |=0x10;
		}

                if(receive_length==40)
                {
                	if((BUF_RECE[1]==0x34)&&(BUF_RECE[2]==0x30)&&(BUF_RECE[3]==0x30))receive_length = 32;//400 返回命令
                	else if((BUF_RECE[1]==0x41)&&(BUF_RECE[2]==0x54)&&(BUF_RECE[4]==0x21))receive_length = 33;//ATX 命令
                	     else  if((BUF_RECE[1]==0x36)&&(BUF_RECE[3]==0x30))
                	     	   { 	if((BUF_RECE[2]==0x30))receive_length = 6;//600 返回命令
                	     	     	else  if(BUF_RECE[2]==0x31)receive_length = 6;//610 返回命令
                	     	   	      else if(BUF_RECE[2]==0x32)receive_length = 6;//620 返回命令
                	     	   	      	   else if(BUF_RECE[2]==0x33)receive_length = 6;//630 返回命令
                	     	   	      	   	else if(BUF_RECE[2]==0x34)receive_length = 6;//640 返回命令	
                	     	   }
                	     	   	
                }		        
		
		
		while(TIME_2MS)
		{
			TIME_2MS = 0;
			
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
						BUF_RECE1[0] = 0xfa;
						BUF_RECE1[1]=  DEVICE_CODE;
						BUF_RECE1[2]=  box;
						BUF_RECE1[3]=  0xfe;
						BUF_RECE1[4]=  0;
						BUF_RECE1[5]=  ver;
						BUF_RECE1[6]=  0;
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