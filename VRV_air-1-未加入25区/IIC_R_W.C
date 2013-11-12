#include "c8051F340.h"
#include "Main.h"

int WriteEE(unsigned int SubAdr,unsigned char Num,unsigned char *Wbuf);
int ReadEE(unsigned int SubAdr,unsigned char Num,unsigned char *Rbuf);

int ReadCCR(unsigned char SubAdr,unsigned char Num,unsigned char *Rbuf);
int ReadCCR(unsigned char SubAdr,unsigned char Num,unsigned char *Rbuf);

unsigned char ReadByte();
int SendByte(unsigned char);
void Start(void);
void Stop(void);
void SendACK(void);
void SendNoACK(void);
void WDFeed(void);



/*页写存储器,Num<=8*/
int WriteEE(unsigned int SubAdr,unsigned char Num,unsigned char *Wbuf)
{
	unsigned char data i;
	if(SubAdr < 0x2000 )
	{
		Start(); /*发送I2C 总线起始条件*/
		if(SendByte(0xa0)!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
		
		if(SendByte(SubAdr/256)!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
				
		if(SendByte((SubAdr%256))!=0)
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
			 
		for(i=0;i<Num;i++) /*重复操作直到发送完最后一个数据*/
		if(SendByte(*(Wbuf+i))!=0) return -1;
		Stop(); /*发送I2C 总线停止条件*/
		Delay(200); /*延时10ms,等待数据写完*/
		return 0;
	}
	else 	return -1;
}

/*读多个字节数据 */
int ReadEE(unsigned int SubAdr,unsigned char Num,unsigned char *Rbuf)
{
	unsigned char data i;
	if(SubAdr < 0x2000 )
	{
		Start();				/* 发送I2C 总线起始条件*/
		if(SendByte(0xa0)!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
			
		if(SendByte(SubAdr/256)!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
				
		if(SendByte((SubAdr%256))!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
			
		Start(); 				/*发送I2C 总线重复起始条件*/
		if(SendByte(0xa1)!=0) 
		{	
			Stop(); /*发送I2C 总线停止条件*/
			return -1; /* 发送被控器地址*/
		}
			
		if(Num!=1)
			{
				for(i=0;i<Num-1;i++)
				{
				*(Rbuf+i)=ReadByte();
				SendACK(); /*发送应答信号*/
				}
				*(Rbuf+Num-1)=ReadByte();
			}
		else
			*Rbuf=ReadByte();
		SendNoACK(); /*最后一个字节,发送非应答信号*/
		Stop(); /*发送I2C 总线停止条件*/
		return 0;
	}
	else 	return -1;
}

/*字节数据传送子程序发送一个字节数据或地址给被控器*/
int SendByte(unsigned char a)
{
	unsigned char data i,j;
	unsigned char bdata Sin;
	Sin=a;
	for(i=0;i<8;i++)
		{
			if((Sin & 0x80)==0)
				SDA=0;
			else
				SDA=1;
			for(j=0;j<80;j++);
			SCL=1; /*置时钟线为高通知被控器开始*/
			for(j=0;j<160;j++);/*保证时钟高周期大于4 s*/
			SCL=0;
			Sin=Sin<<1;
		}
	for(j=0;j<80;j++);
		SDA=1;
	for(j=0;j<80;j++);
		SCL=1;
	for(j=0;j<80;j++);
	if (SDA==0) {SCL=0;return 0;} /*成功,返回0*/
	for(j=0;j<80;j++);
	SCL=0;
	return -1; /*未收到应答,返回-1*/
}


/*数据接收子程序从被控器接收一个字节数据*/
unsigned char ReadByte()
{
	unsigned char data i,j;
	unsigned char bdata Sin;
	SDA=1;
	for (i=0;i<7;i++)
		{
			SCL=0;
			for(j=0;j<80;j++);
			SCL=1;
			if(SDA==1)
				Sin|=0x01;
			else
				Sin&=0xfe;
			for(j=0;j<80;j++);
			Sin<<=1;
		}
	SCL=0;
	for(j=0;j<80;j++);
		SCL=1;
	if(SDA==1)
		Sin|=0x01;
	else
		Sin&=0xfe;
	for(j=0;j<80;j++);
	return(Sin);
}

/*发送应答位*/
void SendACK()
{
	unsigned char data j;
	SCL=0;
	for(j=0;j<80;j++);
	SDA=0;
	for(j=0;j<80;j++);
	SCL=1;
	for(j=0;j<160;j++);
	SCL=0;
}

/*发送非应答位*/
void SendNoACK()
{
	unsigned char data j;
	SCL=0;
	for(j=0;j<80;j++);
	SDA=1;
	for(j=0;j<80;j++);
	SCL=1;
	for(j=0;j<160;j++);
	SCL=0;
}

/*START 启动I2C 总线子程序发送I2C 起始条件*/
void Start(void)
{
	unsigned char data i;
	SDA=1; /* 发送起始条件的数据信号*/
	for (i=0;i<80;i++);
	SCL=1; /*发送起始条件的时钟信号*/
	for(i=0;i<160;i++) ; /*起始条件建立时间大于4.7 s*/
	SDA=0; /*发送起始信号*/
	for(i=0;i<160;i++); /*起始条件锁定时间大于4 s*/
	SCL=0; /*钳住I2C 总线准备发送或接收数据*/
}

/*STOP 停止I2C 总线子程序发送I2C 总线停止条件*/
void Stop(void)
{
	unsigned char data i;
	SDA=0;
	for(i=0;i<80;i++) ;
	SCL=1;
	for(i=0;i<160;i++) ;
	SDA=1;
	for(i=0;i<160;i++);
}



