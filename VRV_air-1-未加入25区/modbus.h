
extern uint8	sendCount;	
extern uint8	receCount;	
extern uint8	sendPosi;


void beginSend(void);
void checkComm0Modbus(void);
void readCoil(void);
void readRegisters(void);
void forceSingleCoil(void);
void presetSingleRegister(void);
void presetMultipleRegisters(void);
void forceMultipleCoils(void);
uint16 getRegisterVal(uint16 addr,uint16 *tempData);
uint16 setRegisterVal(uint16 addr,uint16 tempData);
uint16 getCoilVal(uint16 addr,uint16 *tempData);
uint16 setCoilVal(uint16 addr,uint16 tempData);
