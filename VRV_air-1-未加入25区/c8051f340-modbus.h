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