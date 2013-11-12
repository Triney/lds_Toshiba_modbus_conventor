;------------------------------------------------------------------------------
;  Copyright (c) KEIL ELEKTRONIK GmbH and Franklin Software, Inc., 1987-1992
;------------------------------------------------------------------------------
;  Register Declarations for the 8051 Processor
;

extern	unsigned char bdata relay_state1;

keyaction	BIT	00H			;byte 20h
keyprese	BIT	keyaction+1
keyrelase	BIT	keyprese+1
Sreceive	BIT	keyrelase+1
Sreceiveend	BIT	Sreceive+1
tranbuffull	bit	Sreceiveend+1
tranready	bit	tranbuffull+1
hostwrite	bit	tranready+1
signal		bit	hostwrite+1		;byte 21h
panelsetup	bit	signal+1
blockwrite	bit	panelsetup+1
blockpageover	bit	blockwrite+1
keyenable	BIT	blockpageover+1
toggle70	bit	keyenable+1
flag20ms	bit	toggle70+1
receivetime	bit	flag20ms+1		;byte 22h
ledon		bit	receivetime+1
TIME_2MS	BIT	ledon+1
state_0c	bit	TIME_2MS+1
DISABLE_tran	bit	state_0c+1
BUF_TRAN_BUSY	BIT	tranbuffull
TASK_ENABLE     bit     DISABLE_tran+1
AUTORUN_TASK    bit     TASK_ENABLE+1
TASK_WAIT_TR    bit     AUTORUN_TASK+1

SMB_BUSY	BIT	TASK_WAIT_TR+1	; Software flag to indicate when the
                                        ; EEPROM_ByteRead() or 
                                        ; EEPROM_ByteWrite()
                                        ; functions have claimed the SMBus
SMB_ACKPOLL	BIT	SMB_BUSY+1	; When set, this flag causes the ISR
                                        ; to send a repeated START until the
                                        ; slave has acknowledged its address
SMB_RANDOMREAD  BIT     SMB_ACKPOLL+1   ; When set, this flag causes the ISR
                                        ; to send a START signal after sending
                                        ; the word address.
SMB_RW		BIT	SMB_RANDOMREAD+1; Software flag to indicate the
                                        ; direction of the current transfer
SEND_START	BIT	SMB_RW+1	; Send a start
SMB_SENDWORDADDR BIT	SEND_START+1    ; When set, this flag causes the ISR
                                        ; to send the 8-bit <WORD_ADDR>
                                        ; after sending the slave address.
NEED_SEND_wordadd	BIT	   SMB_SENDWORDADDR+1 
tranbuffull_1	BIT	NEED_SEND_wordadd+1
sreceive_1	BIT	tranbuffull_1+1
sreceiveend_1	BIT	sreceive_1+1
receivetime_1	BIT	sreceiveend_1+1 
DISABLE_tran_1	BIT	receivetime_1+1
LED_SW		BIT	DISABLE_tran_1+1
TRFLAG		BIT	LED_SW+1
serviceled	BIT	TRFLAG+1
servicesw	BIT	serviceled+1
tranready_1	BIT     servicesw+1 
MASTER_MUTE	BIT	tranready_1+1
RECEIVED_DATA_FROM_SLAVE	BIT      MASTER_MUTE+1
MASTER_NEED_SLAVE_DATA	BIT	RECEIVED_DATA_FROM_SLAVE+1  
CLEAR_LAMP_ON	bit	MASTER_NEED_SLAVE_DATA+1
MASTER_OFF	BIT	CLEAR_LAMP_ON+1		;当MASTER OFF键按下时，置1,其他按键按下时清0。
ROMM_IN_MAN	BIT	MASTER_OFF+1		;ROMM_IN_MAN=1 有人进入,ROMM_IN_MAN=0,没有人
ALL_SCENE_ON_FLAG	BIT	ROMM_IN_MAN+1	;=1 总关区域有场景操作  =0 总关区域有场景12



ASM_CARRY	BIT	ASM_PSW.0
ASM_ZERO	BIT	ASM_CARRY+1
ASM_PLUS	BIT	ASM_ZERO+1
ASM_MINUS	BIT	ASM_PLUS+1

TH1_VAL		EQU	0F4H			;定时器1的重载值
TL1_VAL		EQU	0F4H			;9600,7.3728,smod=0
TH0_VAL		EQU	0E8h			;7.3728mHZ
TL0_VAL		EQU	090h			;2MS定时值

PRESEPTR	EQU	0109H
key_PRESEPTR	EQU	00f9H
RELASEPTR	EQU	0259H
key_RELASEPTR	equ	0249H

AIR_contrl_area_EE	EQU	0228H

AIR_WATER_IS_HOT_EE	EQU	022AH		; BIT  0 DOUBLE tube CONTROL 1=YES 0=NO
						; BIT  1  HOT = 00 COLD=1
MAN_OUT_DELAY_EE	EQU	022BH
DEVICE_SUB_BOX_EE	EQU	022CH
MARST_SEND_MODE_EE	EQU	022DH
SUB_DIM_NO_EE		EQU	022EH


SET_temperatureEE_H	EQU	02H
SET_temperatureEE_L	EQU	2FH

DELAY_TIME		EQU	5H

onetouch_TIME	EQU	050			;1 SEC
SIGNON_TIME	EQU	150			;3 SEC
DEVICE_CODE	EQU	0ACH
DEVICE_CODE_SUB	EQU	0B8H			;TEMP USE DR602 CODE  温度控制器
DEVICE_CODE1_SUB	EQU	0C4H		;插排开关设备代码

config_state	equ	08H
START_TASK_POINT	EQU	12H
STARTUP1_TASK_POINT	EQU	16H
TASK_ADDEREE		EQU	20H


EEPROM		EQU	0A0h
CCR             EQU     0D0H

; Status vector - top 4 bits only
SMB_MTSTA	EQU     0xE0                 ; (MT) start transmitted
SMB_MTDB	EQU     0xC0                 ; (MT) data byte transmitted
SMB_MRDB	EQU	0x80                 ; (MR) data byte received
; End status vector definition

;EEPROM			EQU	0A0h
keynumber_all		equ	16
//SDA		BIT	P16
//SCL		BIT	P17
OE0		BIT	P2.1
OE1		BIT	P2.2
;oe		bit	p2.7
service_led	bit	p2.3


RELAY_0		BIT	P1.2
RELAY_1		BIT	P1.3
HYEN		BIT	P1.4		;插牌取电
need_clear_lamp		BIT	P1.5	;请打扫灯
do_not_disturb_lamp	bit	p1.6		;请勿打扰灯

keyareaPRESEEE		EQU	40H
keyareaRELEASEEE	EQU	90H

srelase		data	10h
channelpoint    data    srelase+1
togglefad	data	channelpoint+1			;60H
keyarea		data	togglefad+1
keylednumber	data	keyarea+1
keyjoin		data	keylednumber+1
keyjoinnew	data	keyjoin+1
keyareatemp	data	keyjoinnew+1
keynumbertemp	data	keyareatemp+1
keyjointemp	data	keynumbertemp+1
keyjoinnewtemp	data	keyjointemp+1
P3mirror	data	keyjoinnewtemp+1
P3buffer	data	P3mirror+1
P4mirror	data	P3buffer+1
P4buffer	data	P4mirror+1
time20ms	DATA	P4buffer+1
	

RELAY_OUT	DATA	26H

ASM_ACC		DATA	28H		;BIT 41 ~ 47
ASM_X		DATA	ASM_ACC+1	;
ASM_PSW		DATA	ASM_X+1	;
SECOND		DATA	ASM_PSW+1


BUF_RECE	data	030H			;接受通讯缓冲区
BUF_TRAN	data	BUF_RECE+8		;发送通讯缓冲区

BUF_RECE1	data	BUF_TRAN+8		;接受通讯缓冲区  外部
BUF_TRAN1	data	BUF_RECE1+8		;发送通讯缓冲区
bwriteaddhi	data	BUF_TRAN1+8
bwriteaddlow	data	bwriteaddhi+1
PR_LEVEL	data	bwriteaddlow+1
RTC_BUF		data	PR_LEVEL+1		;48h时间缓冲区
temp		data	RTC_BUF+3
keyprocpoint	DATA	temp
keynumber	data	temp+1
;receivetime	data	keynumber+1
SDATAPTR        data    keynumber+1		;接受通讯缓冲区指针
;SDATACONTER	DATA	SDATAPTR+1		;接受通讯缓冲区计数器
BYTECNT		DATA	SDATAPTR+1		;EEPROM计数器
CHIP_ADDRE	DATA	BYTECNT+1		;AXH
wordadd1	data	CHIP_ADDRE+1		;00000xxxB
SLVADR		DATA	wordadd1
SUBADR		DATA	wordadd1+1		;0~255
pSMB_DATA_IN	DATA	SUBADR+1	;
pSMB_DATA_OUT	DATA	pSMB_DATA_IN
buf_tranptr	data	pSMB_DATA_OUT+1
boxnumber	data	buf_tranptr+1
keypoint	data	boxnumber+1
DELE12NOP	DATA	keypoint+1
;ISP_COUNTER	DATA	DELE12NOP+1
RECE_time_delay	data	DELE12NOP+1		;当接收到数据后,设置延迟时间10MS,经过延迟
							;后,才能发送数据
KEY_READ_BUFF	DATA	RECE_time_delay+1	;8 BYTE BUFFER TEMP
CLOCK_4D	data	KEY_READ_BUFF
comp_buff	data	KEY_READ_BUFF
TASK_100MS_DELAY        data   KEY_READ_BUFF+8
ASM_KEY_MAP	DATA	TASK_100MS_DELAY+1	;01 ~ 16 KEY PRESE  81~96 KEY RELEASE
;P3mirror	data	ASM_KEY_MAP+1
;P3buffer	data	P3mirror+1
lastarea	data	ASM_KEY_MAP+1
lastjoin	data	lastarea+1
buf_tranptr1	DATA	lastjoin+1
sdataptr_1	DATA	buf_tranptr1+1
RECE_time_delay_1	DATA	sdataptr_1+1
netactivetime	DATA	RECE_time_delay_1+1
true_temperature	DATA	netactivetime+1		;SAVE temperature
DEVICE_SUB_BOX	DATA	true_temperature+1
ALL_ON_AREA	DATA	DEVICE_SUB_BOX+1
ALL_ON_CHANNEL_CONTER	DATA	ALL_ON_AREA+1
key1_OFF_delay	data	ALL_ON_CHANNEL_CONTER+1
key1_ON_delay	DATA	key1_OFF_delay+2
P0BUFFER	DATA	key1_ON_delay+2
P0mirror	DATA	P0BUFFER+1
RALAY_ON_TO_OFF_IS_CHANGE	DATA	P0mirror+1		;RELAY 正在翻转，要等状态稳定才能处理。
RALAY_OFF_TO_ON__CHANGE	DATA	RALAY_ON_TO_OFF_IS_CHANGE+1

onetouch_BUF	data	80h
keyenable_buf	data	onetouch_BUF		;D7:键PRESEenable=0XXXXXXXb,键PRESEdisable=1xxxxxxxb
						;D6:键RELASEenable=x0XXXXXXb,键RELASEdisable=x1xxxxxxb
						;D5:键PRESE状态time,PRESE=xx1xxxxxB,need time+1
						;D4:键onetouch状态,onetouch=xxx1xxxx
						;D3:键DDSED状态,1=DDSEND,0=NORMAOL
						;D2:键需要发送状态,1=发送,0=不发送
						;D1:键RELASE状态,1=RELASEY有效
						;D0:单键ON-OFF状态，ON=xxxxxxx1，OFF=xxxxxxx0（16字节）

keytimebuf	data	onetouch_BUF+16



T0_7_COUNT	DATA	keytimebuf+16			;0080H
;###################################################################################
;	T0_7_COUNT  1--8,TASK1~8,	9:STARTUP COUNT
;###################################################################################
T0_7_VECTOR	DATA	T0_7_COUNT+2*9			;00BDH	HIGH ADDRE,LOW ADDRE
;###################################################################################
;	T0_7_VECTOR  1--8,TASK1~8,	9:STARTUP VECTOR
;###################################################################################

ASM_USR_RAM	DATA	T0_7_VECTOR+2*9

STACK_POINTER	data	ASM_USR_RAM+32;0dfh			;堆栈指针 E0 ~ FF  32 byte

key_tran_buf	XDATA	00h			;按键传送缓冲区；7 byte * 16 =112

ASM_TASK_ACC	XDATA	key_tran_buf+7*keynumber_all

LED_40H_XRAM	XDATA	ASM_TASK_ACC+3*9

SUM_LED_40H_XRAM	XDATA	LED_40H_XRAM+4*16

PRESEPTR_XRAM	XDATA	SUM_LED_40H_XRAM+1

SUM_RESEPTR_XRAM	XDATA	PRESEPTR_XRAM+7*16

RELASEPTR_XRAM	XDATA	SUM_RESEPTR_XRAM+1

SUM_RELASEPTR_XRAM	XDATA	RELASEPTR_XRAM+7*16

set_temperature	xdata	SUM_RELASEPTR_XRAM+1		;eerom 	temperature

mode		xdata	set_temperature+1
fan_speed	xdata	mode+1

minute		xdata	fan_speed+1
MAN_OUT_DELAY	XDATA	minute+1	;人离开后延时  (秒)
last_man_mode	xdata	MAN_OUT_DELAY+1
last_man_temperature	xdata	last_man_mode+1
last_man_card_no_h	xdata	last_man_temperature+1
lsat_man_cade_no_low	xdata	last_man_card_no_h+1
man_in_status	xdata	lsat_man_cade_no_low+1;人进入时,状态保存到此

;######################################################################
;	0C BYTE
;	从0C位置读入,判ACC.6=1? 是按键prese release 跟随外部信号禁止,
;			    =0 	按键prese release 跟随外部信号使能.
;	acc.7
;	acc.6       save slider state
;	KEY  12 =master key
;	KEY  13	=插排开关
;	KEY  14	=门口按钮
;;	KEY  15	=SOS按钮
;;		fan_speed from 0 STOP 1 SLOW   2  MID   3  FAST
;		mode_flag  (=0   stop )  (=1  fan)  (=2  cold)   (=3  hot)
;		FAN SPEED HIGH HALF BYTE,MODE LOW HALF BYTE
;P2.6  P2.5  00	FAN HIGH         P2.3 P2.4   00   COLD PROC
;	     01 FAN MID			     01   HOT  PROC
;	     10 FAN LOW
;	     11 FAN STOP		     11   STOP

;	23CH	门口按钮关场景号
;	23DH	门口按钮关区号
;	23EH	门口按钮
;	340H起403H的数值
;
;	351H,352H为门口按钮

;	353H,总开
;	355H,门口按钮关按钮
;	fa 69 ff 8e 01 ff ff 11  Guest in 
;	fa 69 ff 8d 01 ff ff cf  Guest out
;	fa ac ff 9b 12 16 1e	12 模式  24度 31度  