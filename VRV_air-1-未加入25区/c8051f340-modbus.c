//modbusͨ��Э��ʵ��
//

#include <c8051f340.h>            
#include <intrins.h>
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
#define BaudRate       9600//9600,14400,19200,38400,56000��ѡ        
#define SYSCLK         12000000    
//#define BAUDRATE     9600         
//#define SYSCLK       11059200     
sbit GREEN = P2^5;
sbit RED   = P2^4;
sbit PF    = P2^1;
//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

#define uint16 unsigned int
#define uint8 unsigned char
#define   MAXNByte  15   //����ֽ���
//#define  time0      30000
#define  MAXREG     100   //���Ĵ�������

//void SYSCLK_Init (void);
void PORT_Init (void);   //IO�ڳ�ʼ��
void UART0_Init (void);   //uart0��ʼ��

void readRegisters(void) ;//���Ĵ���,������03
void beginSend(void) ;    //�����ӳ���
void presetMultipleRegisters(void);//���ö���Ĵ���,������16
void FLASH_ByteWrite(uint16 addr, uint8 byte);//flashд
void FLASH_PageErase (uint16 addr);        //flashҳ��д
//void Receive_timeout(void);             //��ʱ�ӳ���
uint16 crc_chk(uint8 *dat,uint8 length);         //CRCУ��
uint8 FLASH_ByteRead (uint16 addr);           //flash��
//void presetSingleRegisters(void);       //���õ����Ĵ���,������06
void send_err(uint8 err,uint8 err_code);       //���ʹ���
void Timer0_Init(void);                  //��ʱ��0��ʼ��
void PCA_Init();
uint16 flag;
uint8 temp_crc;
uint8 count_receiveNByte;//�����ֽ���
uint8 xdata mod_buf[20];       //modubs ���ݽ��ջ�����
uint8 xdata sendBuf[20];       //���ͻ�����
uint8 sendCount;         //�����ֽ���
uint8 localAddr = 3;     //��Ƭ�����ư�ĵ�ַ
//uint16 Receivetimeout;    //��ʱ��ֵ
//bit ReceiveBit;    

//unsigned int  code  baud_code[8] = {};
/*?????*/
/*void delay(unsigned int m)
{
 unsigned int n;
 n=0;
 while(n < m)
 {n++;}
 return;
}  */
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

/*
void main(void)
{
   uint16 dat;
   uint16 tempData;
   uint16 crcData;
   PCA0MD    &= ~0x40;                    
   OSCICN    |= 0x03; 

   //SYSCLK_Init ();                     
    PORT_Init ();                       
    UART0_Init (); 
    PCA_Init();                    
// 	Timer0_Init();
    EA=1;
    ES0=1;
	PS0=1;
	GREEN=1;     
    RED=0;
 while(1){
  if (mod_buf[0]==localAddr)
  { 
     if(count_receiveNByte>4) 
	{


    switch(mod_buf[1])
	{
       case 0x03:
	                    if(count_receiveNByte>=8)
	      		 	    { 
				          crcData = crc_chk(mod_buf,6);  //CRCУ��
                          dat=mod_buf[7];
                          if(crcData==mod_buf[6]+(dat<<8)) //CRCУ�����Ƚ�
                        
                              readRegisters();         // ���������ȷ����ô�ͻ�Ӧ����
							  count_receiveNByte=0;     //��������ָ�����   
						 
                         }
                    break;
                    
                 
/*    	case 0x06:
               if(count_receiveNByte>=8)
	      		 	{ 
				     crcData = crc_chk(mod_buf,6);  //CRCУ��
					 dat=mod_buf[7];
                   if(crcData==mod_buf[6]+(dat<<8)) //CRCУ�����Ƚ�
                     
 		            presetSingleRegisters();
                    count_receiveNByte=0;     //��������ָ�����  
 			       }
                   break;*/
/*	    case 0x10:
		           dat=mod_buf[4];
                   tempData = (dat<<8) + mod_buf[5];
                   tempData = tempData * 2; //���ݸ���
                   tempData += 9;
                if(count_receiveNByte >= tempData)
                 {
                  
                     crcData = crc_chk(mod_buf,tempData-2);
                     dat=mod_buf[tempData-1];
                    if(crcData == (dat<<8)+ mod_buf[tempData-2])
                      
		                presetMultipleRegisters();
						count_receiveNByte=0;     //��������ָ�����   
					
				 }
				  break; 
		 default:
                  break; 
	 }
				

   }
   
         
  }
  else
    count_receiveNByte=0;     //��������ָ�����
  _nop_();
//    Receive_timeout();
   PCA0CPH4 = 0x00;
 }
}

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// This routine initializes the system clock to use an 22.1184MHz crystal
// as its clock source.
//
//void SYSCLK_Init (void)
//{
//   int i;                              

//   OSCXCN = 0x67;                     
 //  for (i=0; i < 256; i++) ;         

//   while (!(OSCXCN & 0x80)) ;          

//   OSCICN = 0x88;                     
//}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Configure the Crossbar and GPIO ports
//
 */
void PCA_Init()
{
    PCA0CN     =  0x40;                // PCA counter enable
    PCA0MD    &= ~0x40 ;               // Watchdog timer disabled-clearing bit 6
    PCA0MD    &=  0xF1;                // timebase selected - System clock / 12
    PCA0CPL4   =  0xFF;                // Offset value
    PCA0MD  |= 0x40;
    PCA0L    = 0x00;               	   // Set lower byte of PCA counter to 0  
    PCA0H    = 0x00; 
}

void PORT_Init (void)
{
   XBR0    = 0x01;                      //UART TX0, RX0�����˿�����P0.4��P0.5
   XBR1    = 0x40;
   XBR2    = 0x00;                     //UART1��I/O�������˿���?
   P0MDOUT |= 0x01;                    //TX0���Ϊ���췽ʽ
   P2MDOUT |= 0x3f;                   
   P1MDOUT |= 0xff;
//   P0SKIP  |= 0xc0;
}

//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Configure the UART0 using Timer1, for <baudrate> and 8-N-1.
//
void UART0_Init (void)
{
    SCON0   = 0x50;                    
    TMOD    = 0x20;                    
	if(BaudRate > 38400)                 
	{
		CKCON  = 0x08;                      //��ʱ��ʹ��ϵͳʱ��
		TH1    = 256-SYSCLK/2/BaudRate;
	}
	if(BaudRate <= 38400) 
	{
		CKCON  = 0x01;                      //��ʱ��ʱ����ϵͳʱ�ӵ�4��Ƶ
		TH1    = 256-SYSCLK/4/2/BaudRate;
	}
   TL1=TH1;
   TR1    = 1;                        
   CKCON |= 0x01;                      //��ʱ��ʱ����ϵͳʱ�ӵ�4��Ƶ
   PF=0;
}

/*void Timer0_Init(void)
{
 TMOD |= 0x01;        // ��ʱ��0: ��ʽ1,16λ��ʱ��
 TF0=0;        
 TR0=1;// TR0
 CKCON |= 0x10;        // 
 TH0 = 0xf1; //256 -((60*SYSCLK/1000/12)/256);
 TL0 = 0xc2;//256 -((60*SYSCLK/1000/12)%256);
 ET0=1;
}

void timer0() interrupt 1 using 2 //��ʱ���ж�
{   
    TF0=0;
    TR0=0;
    TH0=0xf1;    //3.646ms interrupt
    TL0=0xc2;
    flag++;
	if(flag==15)
	 {
	   flag=0;
       if(count_receiveNByte<=2)    //��ʱ�������ջ������������ж�Ϊ�յ�һ֡
       {
              count_receiveNByte=0;     //��������ָ�����   ;
       }
	  }
	TR0=1;
}
/***********************************************************
CRCУ��
***************************************************************/
uint16 crc_chk(uint8 *pData,uint8 nLen)
{
uint16 temp_crc=0xffff,temp1,i,j;
 for(i=0;i<nLen;i++)
    {
    temp_crc^=*(pData+i);
	for(j=0;j<8;j++)
		{
		temp1=temp_crc;
		temp_crc>>=1;
		if(temp1&0x0001)
			temp_crc^=0xa001;
        }
    }
return(temp_crc);
}

/*������03��*/
void readRegisters(void)
{
   uint16 addr;
   uint16 tempAddr;
// uint16 result;
   uint16 crcData;
   uint8 readCount;
   uint8 byteCount;

// uint8  finsh; //
   uint8 i;
   uint16 tempData = 0; 
   tempData=mod_buf[2];
   addr = (tempData<<8) + mod_buf[3];
   tempAddr = addr ;//& 0xfff; 
   //addr = mod_buf[3];
   //tempAddr = addr;

 //readCount = (receBuf[4]<<8) + receBuf[5]; 
   readCount  = mod_buf[5];
   if(readCount<MAXREG)
   {
       byteCount = readCount* 2 ;//;
 
      for(i=0;i<byteCount;i++,tempAddr++)
      {
          tempData=FLASH_ByteRead(tempAddr);
          //getRegisterVal(tempAddr,&tempData);    
          sendBuf[i+3] = tempData;// & 0xff; 
  
       }
 
       sendBuf[0] = localAddr;
       sendBuf[1] = 3;
       sendBuf[2] = byteCount;
       byteCount += 3;
       crcData = crc_chk(sendBuf,byteCount);
       sendBuf[byteCount] = crcData & 0xff;
       byteCount++;
       sendBuf[byteCount] =  crcData >> 8;

       sendCount = byteCount + 1;
       beginSend();
   }
   else
	{

	    send_err(0x83,0x03); 
    }
}//void readRegisters(void)

/*****************************************************************
������6 ���õ����Ĵ���
******************************************************************
void presetSingleRegisters(void)
{
 uint16 addr;
 uint16 tempAddr;


 uint16 crcData;
 uint16 tempData;
// uint8  finsh; //Ϊ1ʱ��� Ϊ0ʱ����

 tempData=mod_buf[2];
 addr = (tempData<<8) + mod_buf[3];
  tempAddr = addr;// & 0xfff;
  //addr = mod_buf[3];
  //tempAddr = addr & 0xff;

  //setCount = (receBuf[4]<<8) + receBuf[5];
  //setCount = mod_buf[5];
  //byteCount = mod_buf[6]; 
 
     FLASH_PageErase (tempAddr);

//    tempData = (modbuf[i*2+7]<<8) + modbuf[i*2+8];
      tempData=mod_buf[4];
//    setRegisterVal(tempAddr,tempData); 
      FLASH_ByteWrite (tempAddr, tempData);
	  sendBuf[4] = tempData ; 
	  tempData=mod_buf[5];
	  FLASH_ByteWrite (tempAddr+1, tempData);         
      sendBuf[5] = tempData;
 

 sendBuf[0] = localAddr;
 sendBuf[1] = 6;
 sendBuf[2] = addr >> 8;
 sendBuf[3] = addr & 0xff;
 crcData = crc_chk(sendBuf,6);
 sendBuf[6] = crcData & 0xff;
 sendBuf[7] = crcData >> 8;
 sendCount = 8;
 beginSend(); 
}

/****************************************************************
������16�����ö�����ܼĴ���
******************************************************************/
void presetMultipleRegisters(void)
{

 uint16 addr;
 uint16 tempAddr;
 uint8 byteCount;
 uint8 setCount;
 uint16 crcData;
 uint16 tempData;
// uint8  finsh; //Ϊ1ʱ��� Ϊ0ʱ����
 uint8 i;
 
 //addr = mod_buf[3];
 tempData=mod_buf[2];
 addr = (tempData<<8) + mod_buf[3];
 tempAddr = addr;// & 0xfff;
 
 //tempAddr = addr & 0xff;

 //setCount = (receBuf[4]<<8) + receBuf[5];
 setCount = mod_buf[5];
 if(setCount<=0x78)
{
  byteCount = mod_buf[6]; 
 

 sendBuf[0] = localAddr;
 sendBuf[1] = 16;
 sendBuf[2] = addr >> 8;
 sendBuf[3] = addr & 0xff;
 sendBuf[4] = setCount >> 8;
 sendBuf[5] = setCount & 0xff;
 crcData = crc_chk(sendBuf,6);
 sendBuf[6] = crcData & 0xff;
 sendBuf[7] = crcData >> 8;
 sendCount = 8;
 beginSend();
 
    FLASH_PageErase (0x7c00);

   for(i=0;i<byteCount;i++,tempAddr++)
   {
//    tempData = (modbuf[i*2+7]<<8) + modbuf[i*2+8];
      tempData=mod_buf[7+i];
//    setRegisterVal(tempAddr,tempData); 

      FLASH_ByteWrite (tempAddr, tempData);
   }

 }
 else//�Ĵ�������>120
   {

     send_err(0x90,0x03);
 
   }
 
}//void presetMultipleRegisters(void)




/*UART0�жϴ���*/
/*
void UART0_ISR (void) interrupt 4
{
 ES0=0;
 if(!TI0)              
   {
//    Receivetimeout=time0;
    RI0 = 0;  
    mod_buf[count_receiveNByte] = SBUF0; //��ȡ���� 
   count_receiveNByte++;       //�����ֽ���Ŀ
//   Receivetimeout=350;  
  if(count_receiveNByte >= MAXNByte)   //���������
   count_receiveNByte = 0;     //������ ��0  
   
   }
  TI0 = 0;    
  ES0=1;
}			 */
//�����ӳ���
void beginSend(void) 
{	uint8 i;
        ES0=0;
        GREEN=0;
		RED=1;
		PF=1;          //485������
        for(i=0;i<sendCount;i++)
       {
		//��żУ��
	    ACC=sendBuf[i] ;
		TB80=P;
        SBUF0 = sendBuf[i];
        while(TI0==0);
        TI0=0;
       }
	   PF=0;           //485�������
	   GREEN=1;
	   RED=0;
	   ES0=1;
}

void send_err(uint8 err,uint8 err_code)//���ʹ���
{
  uint16 crc_tmp;
	
  sendBuf[0]=localAddr;
  sendBuf[1]=err;
  sendBuf[2]=err_code;
  crc_tmp=crc_chk(mod_buf,3);
  sendBuf[3]=crc_tmp;
  sendBuf[4]=crc_tmp>>8;
  sendCount=5;
  beginSend();	
  
}
/*****************************************************
���ճ�ʱ�ж�
����������
 ((Receivetime_flag=1)&&(count_receiveNByte>0))  1ms��ʱʱ�䵽
���� �н��յ����ַ������ô�ģ�飬�ж��Ƿ���ճ�ʱ
*****************************************************/
 /*
void Receive_timeout(void)
{
 Receivetimeout--;
 if((Receivetimeout==0)&&(count_receiveNByte>0)) //˵�����ճ�ʱ
  {
   count_receiveNByte=0;     //��������ָ�����
   SCON0|=0x10;       //����UART0���գ�REN0=1;
  }
}
 */

void FLASH_PageErase (uint16 addr)
{
   bit EA_SAVE = EA;                   // Preserve EA
   char xdata * data pwrite;           // FLASH write pointer

   EA = 0;                             // Disable interrupts
   // change clock speed to slow, then restore later

   VDM0CN = 0x80;                      // Enable VDD monitor
	
   RSTSRC |= 0x02;                      // enable VDD monitor as a reset source

  
   FLKEY  = 0xA5;                      // Key Sequence 1
   FLKEY  = 0xF1;                      // Key Sequence 2
   PSCTL |= 0x03;                      // PSWE = 1; PSEE = 1


   VDM0CN = 0x80;                      // Enable VDD monitor

   RSTSRC |= 0x02;                      // Enable VDD monitor as a reset source

   pwrite = (char xdata *) addr;

   VDM0CN = 0x80;                      // Enable VDD monitor
	
   *pwrite = 0;                        // Initiate page erase

   PSCTL &= ~0x03;                     // PSWE = 0; PSEE = 0

   EA = EA_SAVE;                       // Restore interrupts
}
/********************************************************************
���Ĵ���ֵ
********************************************************************/
uint8 FLASH_ByteRead (uint16 addr)
{
   bit EA_SAVE = EA;                   // Preserve EA
   char code * data pread;             // FLASH read pointer
   unsigned char byte;

   EA = 0;                             // Disable interrupts

   pread = (char code *) addr;

   byte = *pread;                      // Read the byte

   EA = EA_SAVE;                       // Restore interrupts

   return byte;
}
/************************************************************
Flashд
************************************************************/
void FLASH_ByteWrite (uint16 addr, uint8 byte)
{
   bit EA_SAVE = EA;                   // Preserve EA
   char xdata * data pwrite;           // FLASH write pointer

   EA = 0;                             // Disable interrupts

   // change clock speed to slow, then restore later

   VDM0CN = 0x80;                      // Enable VDD monitor

   RSTSRC |= 0x02;                      // Enable VDD monitor as a reset source

   pwrite = (char xdata *) addr;

   PFE0CN &= 0XFE;		//�趨���ֽ�д��FLASH��ʽ

   PSCTL |= 0x01;                      // PSWE = 1
   FLKEY  = 0xA5;                      // Key Sequence 1
   FLKEY  = 0xF1;                      // Key Sequence 2

   VDM0CN = 0x80;                      // Enable VDD monitor


   RSTSRC |= 0x02;                      // Enable VDD monitor as a reset source

   VDM0CN = 0x80;                      // Enable VDD monitor

   *pwrite = byte;                     // Write the byte

   PSCTL &= ~0x01;                     // PSWE = 0

   EA = EA_SAVE;                       // Restore interrupts
}

