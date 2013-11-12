void PORT_Init (void);   //IO口初始化
void UART0_Init (void);   //uart0初始化

void readRegisters(void) ;//读寄存器,功能码03
void beginSend(void) ;    //发送子程序
void presetMultipleRegisters(void);//设置多个寄存器,功能码16
void FLASH_ByteWrite(uint16 addr, uint8 byte);//flash写
void FLASH_PageErase (uint16 addr);        //flash页擦写
//void Receive_timeout(void);             //超时子程序
uint16 crc_chk(uint8 *dat,uint8 length);         //CRC校验
uint8 FLASH_ByteRead (uint16 addr);           //flash读
//void presetSingleRegisters(void);       //设置单个寄存器,功能码06
void send_err(uint8 err,uint8 err_code);       //发送错误
void Timer0_Init(void);                  //定时器0初始化
void PCA_Init();