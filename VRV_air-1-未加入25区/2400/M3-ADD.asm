;------------------------------------
;-  Generated Initialization File  --
;------------------------------------

$include (C8051F340.inc)
$include (HOTEL_ROOM.inc)

		

		ORG	0000H
		jmp	MAIN_PRO		;转入主运行程序入口
		
		org	000bh
		ajmp	time0int

		ORG	0023H
		JMP	RI_TISUB0
		
		ORG	033H
		JMP	SPIO_PROC
		
		ORG	003BH		;SMB0 INTERRUPT
		JMP	I2CPROC
		
		ORG	0083H
		JMP	RI_TISUB1
		
		ORG	100H
SPIO_PROC:		
		;PUSH	ACC
		;PUSH	PSW
		;JBC	SPIO_MOV_HIGH,SPIO_PROC_1
		;MOV	temperature_h,SPI0DAT
		;SETB	SPIO_MOV_HIGH
	;	MOV	SPI0DAT,#55H
		;JMP	SPIO_PROC_2
		
SPIO_PROC_1:		
		;MOV	A,SPI0DAT
		;RLC	A
		;MOV	A,temperature_h
		;RLC	A
	;	MOV	temperature_L,A
		mov  SPI0CN,    #00Dh
		
SPIO_PROC_2:		
		CLR	SPIF
	;	POP	PSW
		;POP	ACC
		RETI
		
time0int:
		PUSH	ACC
		PUSH	PSW
		clr	tr0
		mov	th0,#TH0_VAL
		mov	tl0,#TL0_VAL
		setb	tr0
		SETB	TIME_2MS
		JNB	DISABLE_tran,time0int_B
		DJNZ	RECE_time_delay,time0int_B
		CLR	DISABLE_tran
time0int_B:
		JNB	DISABLE_tran_1,time0int_C
		DJNZ	RECE_time_delay_1,time0int_C
		CLR	DISABLE_tran_1
time0int_C:
		djnz	time20ms,time0int_a
		setb	flag20ms
		mov	time20ms,#10
time0int_a:
		;sjmp	time0intend
		jnb	Sreceive,time0intend
		jb	receivetime,receiover
		setb	receivetime
		ajmp	time0intend
receiover:	clr	sreceive
		clr	receivetime
		;clr	OE0
		SETB	OE0
		setb	ren0
		clr	ti0

time0intend:
		pop	psw
		pop	acc
		reti
		
;************************************************************************
;*名称:	SVC_SCI								*
;*用途: 串行接口中断服务子程序						*
;*说明:	此程序串口的发送和接收						*
;************************************************************************
RI_TISUB0:
		clr	ea
		push	psw
		push	acc
		mov	a,r0
		push	acc
		mov	a,r1
		push	acc
		jb	ti0,	tisub
		jnb	ri0,ri_tisubend
		clr	ri0
		jb	sreceive,ri1
		mov	a,sbuf0
		cjne	a,#0FAh,check_sbuf1
		jmp	first_receive
check_sbuf1:
		cjne	a,#0F5h,check_sbuf2
		jmp	first_receive
check_sbuf2:
		cjne	a,#0FCh,ri_tisubend
first_receive:
		SETB	DISABLE_tran
		MOV	RECE_time_delay,#06H	;接受数据10MS后,才能发送数据
		setb	sreceive		;first receive
		mov	a,#buf_rece
		mov	sdataptr,a
		clr	receivetime
ri1:		mov	r0,sdataptr
		mov	a,sbuf0
		mov	@r0,a
		clr	receivetime
		cjne	r0,#buf_rece+7,recon
		setb	sreceiveend
		clr	sreceive
		clr	ren0
		ajmp	ri_tisubend

recon:
		inc	r0
		mov	sdataptr,r0
		ajmp	ri_tisubend
tisub:
		clr	ti0
		mov	r0,buf_tranptr
		mov	a,@r0
		cjne	r0,#buf_tran+8,ticon
		call	trandelay
		SETB	DISABLE_tran
		MOV	RECE_time_delay,#06H	;发送数据10MS后,才能发送数据
		SETB	OE0
		;clr	OE0
		CLR	MASTER_MUTE
		setb	ren0
		setb	serviceled
		mov	netactivetime,#06h
		clr	tranbuffull
		jb	panelsetup,ri_tisubend
		mov	r1,#buf_rece
		mov	r0,#buf_tran
		mov	buf_tranptr,#08
copyt_rbuf1:	mov	a,@r0
		mov	@r1,a
		inc	r0
		inc	r1
		djnz	buf_tranptr,copyt_rbuf1
		clr	ren0
		setb	sreceiveend
		clr	sreceive
		ajmp	ri_tisubend

ticon:
		inc	buf_tranptr
		mov	sbuf0,a

ri_tisubend:
		pop	acc
		mov	r1,acc
		pop	acc
		mov	r0,acc
		pop	acc
		pop	psw
		CLR	TRFLAG
		setb	ea
		RETI	
;************************************************************************
;*名称:	RI_TISUB1								*
;*用途: 串行接口_1中断服务子程序						*
;*说明:	此程序串口的发送和接收						*
;************************************************************************
		
RI_TISUB1:		
		;PUSH	ACC
		;MOV	A,SCON1
		;ANL	A,#11111100B
		;MOV	SCON1,A
		;POP	ACC
		clr	ea
		push	psw
		push	acc
		mov	a,r0
		push	acc
		mov	a,r1
		push	acc
		MOV	A,SCON1
		JBC	ACC.1,ti1_sub
		JBC	ACC.0,RI1_SUB
		ANL	A,#00111100B
		MOV	SCON1,A
		JMP	ri_tisubend_1
RI1_SUB:		
		MOV	SCON1,A
		jb	sreceive_1,ri1_1
		mov	a,sbuf1
		cjne	a,#0FAh,check_sbuf1_1
		jmp	first_receive_1
check_sbuf1_1:
		cjne	a,#0F5h,check_sbuf2_1
		jmp	first_receive_1
check_sbuf2_1:
		cjne	a,#0FCh,ri_tisubend_1
first_receive_1:
		SETB	DISABLE_tran_1
		MOV	RECE_time_delay_1,#06H	;接受数据10MS后,才能发送数据
		setb	sreceive_1		;first receive
		mov	sdataptr_1,#buf_rece1
		clr	receivetime_1
ri1_1:		mov	r0,sdataptr_1
		mov	a,sbuf1
		mov	@r0,a
		clr	receivetime_1
		cjne	r0,#buf_rece1+7,recon_1
		setb	sreceiveend_1
		clr	sreceive_1
		MOV     A,SCON1
                CLR    ACC.0
                MOV     SCON1,A
		jmp	ri_tisubend_1

recon_1:
		INC	sdataptr_1
		jmp	ri_tisubend_1
ti1_sub:
		MOV	SCON1,A
		mov	r0,buf_tranptr1
		mov	a,@r0
		cjne	r0,#buf_tran1+8,ticon1
		call	trandelay
		SETB	DISABLE_tran_1
		MOV	RECE_time_delay_1,#06H	;发送数据10MS后,才能发送数据
		SETB	OE1
		;clr	OE1
		;setb	ren1
                MOV     A,SCON1
                SETB    ACC.0
                MOV     SCON1,A
		clr	tranbuffull_1
		setb	serviceled
		mov	netactivetime,#06h
		
		;jb	panelsetup,ri_tisubend1
		;mov	r1,#buf_rece
		;mov	r0,#buf_tran
		;mov	buf_tranptr,#08
;copyt_rbuf1:	;mov	a,@r0
		;mov	@r1,a
		;inc	r0
		;inc	r1
		;	buf_tranptr,copyt_rbuf1
		;clr	ren0
		;setb	sreceiveend
		;clr	sreceive
		ajmp	ri_tisubend_1

ticon1:
		inc	buf_tranptr1
		mov	sbuf1,a

ri_tisubend_1:
		pop	acc
		mov	r1,acc
		pop	acc
		mov	r0,acc
		pop	acc
		pop	psw
		CLR	TRFLAG
		setb	ea
		RETI
			
				
;#############################################################################
;		SMBUS (I2C INTERRUPT) PROC
;
;#############################################################################	   


I2CPROC:
;------------------------------------------------------------------------------------
; SMBus Interrupt Service Routine (ISR)
;------------------------------------------------------------------------------------
;
; SMBus ISR state machine
; - Master only implementation - no slave or arbitration states defined
; - All incoming data is written starting at the global pointer <pSMB_DATA_IN>
; - All outgoing data is read from the global pointer <pSMB_DATA_OUT>
;
   		PUSH	PSW
		CLR	F0                   ; Used by the ISR to flag failed
                                             ; transfers

   			                     ; Used by the ISR to count the
                                             ; number of data bytes sent or
                                             ; received

   		;CLR	SEND_START           ; Send a start
		PUSH	ACC
		MOV	ACC,R0
		PUSH	ACC
		MOV	A,SMB0CN
		ANL	A,#0F0H
		CJNE	A,#SMB_MTSTA,TEST_SMB_MTDB	;0E0H
; Master Transmitter/Receiver: START condition transmitted.		
		MOV	A,CHIP_ADDRE
		JNB	SMB_RW,Master_WRITE
		ORL	A,#01H		;READ
		CLR	NEED_SEND_wordadd
Master_WRITE:	MOV	SMB0DAT,A
		CLR	STA		; Manually clear START bit
		JMP	I2CPROC_END
		
		
TEST_SMB_MTDB:	CJNE	A,#SMB_MTDB,TEST_SMB_MRDB
; Master Transmitter: Data byte (or Slave Address) transmitted
		JNB	ACK,slave_NACK
; Slave Address or Data Byte
; Acknowledged?	
		JNB	SEND_START,TEST_SENDWORDADDR
		CLR	SEND_START
		NOP
		NOP
		NOP
		NOP
		SETB	STA
		JMP	I2CPROC_END
TEST_SENDWORDADDR:		
		JNB	NEED_SEND_wordadd,CHECK_SEND_NEXT_BYTE
		CLR	NEED_SEND_wordadd
		MOV	SMB0DAT,wordadd1		; send word address_HIGH
		JMP	I2CPROC_END_1
CHECK_SEND_NEXT_BYTE:		
		JNB	SMB_SENDWORDADDR,TEST_SMB_RW
; Are we sending the word address		
		CLR	SMB_SENDWORDADDR	; Clear flag
		MOV	SMB0DAT,SUBADR		; send word address
		JNB	SMB_RANDOMREAD,NO_RANDOMREAD
		SETB	SEND_START	; send a START after the next ACK cycle
		SETB	SMB_RW
		
NO_RANDOMREAD:		
		JMP	I2CPROC_END

TEST_SMB_RW:	
; Is this transfer a WRITE?	
		JB	SMB_RW,SMB_RW_R
		MOV	A,BYTECNT
		JZ	SEND_STO		;BYTE COUNTER=0,
		MOV	R0,pSMB_DATA_OUT
		MOV	A,@R0
		MOV	SMB0DAT,A
		INC	pSMB_DATA_OUT
		DEC	BYTECNT
		JMP	I2CPROC_END
SEND_STO:		
		SETB	STO                  ; set STO to terminte transfer
                CLR	SMB_BUSY             ; clear software busy flag
SMB_RW_R:		
		JMP	I2CPROC_END
		
slave_NACK:
		JB 	SMB_ACKPOLL,slave_NACK_1
		SETB	F0		; Indicate failed transfer
                                        ; and handle at end of ISR
		JMP	I2CPROC_END
slave_NACK_1:
                NOP
		NOP
		NOP
		NOP
		SETB	STA             ; Restart transfer
 		JMP	I2CPROC_END

TEST_SMB_MRDB:	
; Master Receiver: byte received
		SETB	F0
		CJNE	A,#SMB_MRDB,I2CPROC_END
		CLR	F0
      		MOV	R0,pSMB_DATA_IN
      		DJNZ	BYTECNT,RCV_DATA_NEXT
      		JMP	RCV_DATA_LAST
RCV_DATA_NEXT:      		
		MOV	A,SMB0DAT
		MOV	@R0,A		     ; Store received byte
		INC	pSMB_DATA_IN	     ; Increment data in pointer
		SETB	ACK                  ; Set ACK bit (may be cleared later
                                             ; in the code)

		JMP	I2CPROC_END
RCV_DATA_LAST:	; This is the last byte
		MOV	A,SMB0DAT
		MOV	@R0,A
		CLR	SMB_BUSY             ; Free SMBus interface
            	CLR	ACK                  ; Send NACK to indicate last byte
                                             ; of this transfer
            	SETB	STO                  ; Send STOP to terminate transfer
      
I2CPROC_END:
   		JNB	F0, I2CPROC_END_1    ; If the transfer failed,
		MOV	A,SMB0CN 	     ; Reset communication
		ANL	A,#10111111B
                ORL	A,#40H
                MOV	SMB0CN,A
      		CLR	SMB_BUSY             ; Free SMBus
   
   
I2CPROC_END_1:		
		
		POP	ACC
		MOV	R0,ACC
		POP	ACC
		POP	PSW
		CLR	SI
		RETI		


;####################################################################################### 		
;		发送数据至X1243处理程序
;		SlvAdr保存器件头地址（Slave Address）	AF(读EEPROM）AE（写EEPROM）
;							DF(读CCR）   DE（写CCR）
;		wordadd1 	保存高字节地址，	00000A10A9A8  （EEPROM）00000000  （CCR）
;		SubAdr		保存低字节地址，	A7A6A5A4A3A2A1A0（EEPROM）（00~3F）（CCR）
;		ByteCnt		写字节长度计数器
;		R0		写字节指针
;
;#######################################################################################
SendData_EEPROM:
		MOV	CHIP_ADDRE,#EEPROM
		SETB	NEED_SEND_wordadd
 		
SendData:
		mov	a,wordadd1
		jz	SendData1
		anl	a,#11100000b
		jnz	SendReturn
		sjmp	SendData2
SendData1:
		mov	a,SubAdr
		cjne	a,#01h,SendData2
		mov	R1,pSMB_DATA_OUT
		CJNE	@R1,#00,SendData2
		JMP	SendReturn		;box=0 jump
SendData2:		
		
		JB	SMB_BUSY,$
		SETB	SMB_BUSY
;		Set SMBus ISR parameters
    		CLR	SMB_RW               ; Mark next transfer as a write
   		SETB	SMB_SENDWORDADDR     ; Send Word Address after Slave Address
   		CLR	SMB_RANDOMREAD	     ; Do not send a START signal after
                                             ; the word address
   		SETB	SMB_ACKPOLL          ; Enable Acknowledge Polling (The ISR
                                             ; will automatically restart the 
                                             ; transfer if the slave does not 
                                             ; acknoledge its address.

;		Initiate SMBus Transfer
   		SETB	STA 
		JB	SMB_BUSY,$           ; Wait until data is write
SendReturn:
		RET

;######################################################################################
;		从X1243接受数据处理程序
;		SlvAdr保存器件头地址（Slave Address）	AF(读EEPROM）AE（写EEPROM）
;							DF(读CCR）   DE（写CCR）
;		wordadd1 	保存高字节地址，	00000A10A9A8  （EEPROM）00000000  （CCR）
;		SubAdr		保存低字节地址，	A7A6A5A4A3A2A1A0（EEPROM）（00~3F）（CCR）
;		ByteCnt		读字节长度计数器
;		R1		读字节指针
;		R0		use
;
;###################################################################################### 
rcvdata_EEPROM:
		MOV	CHIP_ADDRE,#EEPROM
		SETB	NEED_SEND_wordadd
		
RcvData:	
		mov	a,wordadd1
		jz	RcvData1
		anl	a,#11100000b
		jnz	SendReturn
RcvData1:		
		JB	SMB_BUSY,$           ; Wait for SMBus to be free.
   		SETB	SMB_BUSY             ; Claim SMBus (set to busy)
; Set SMBus ISR parameters
   		CLR	SMB_RW		     ; A random read starts as a write
                                             ; then changes to a read after
                                             ; the repeated start is sent. The
                                             ; ISR handles this switchover if
                                             ; the <SMB_RANDOMREAD> bit is set.
   		SETB	SMB_SENDWORDADDR     ; Send Word Address after Slave Address
   		SETB	SMB_RANDOMREAD       ; Send a START after the word address
   		SETB	SMB_ACKPOLL          ; Enable Acknowledge Polling


; Initiate SMBus Transfer
   		SETB	STA 
   		JB	SMB_BUSY,$           ; Wait until data is read

		RET		

trandelay:	mov	acc,#80h
		djnz	acc,$
		ret

;##################################################################
;
;		wait write end
;
;##################################################################		
waitwriteend:		
	;	clr	EA
	;	mov	wfeed1,#0A5h
	;	mov	wfeed2,#05Ah
	;	setb	ea
		PUSH	TEMP
		mov	BYTECNT,#01H
		mov	pSMB_DATA_IN,#temp
		call	rcvdata_EEPROM
		POP	TEMP
		RET
		
						
READONEBYTE:
 		mov	bytecnt,#01
		sjmp	READMOREBYTE

;#######################################################################################
;
READTWOBYTE:
		mov	bytecnt,#02
READMOREBYTE:
 		CLR	A
		mov	wordadd1,A
		call	rcvdata_EEPROM
		RET

;command_sense:
		;DB	00h
		;db	0feh,0d2h,0fdh,0d6h		

transbuf_1:
		mov	r1,#buf_tran1
		mov	a,@r1
		cjne	a,#0F5h,transbuf3_1	;FIRST CODE = 00 OR FF JUMP
;##############################################################################
;		check sence
;##############################################################################
		;mov	r7,buf_tran+3
		;mov	r2,#05			;command_sense 计数
		;mov	r3,BUF_RECE+2
		;mov	dptr,#command_sense
;find_sense:	;mov	a,r2
		;movc	a,@a+dptr
		;xrl	a,r7
		;jz	find_sense_1			;find sense
		;djnz	r2,find_sense
		;jmp	transbuf3_A
;find_sense_1:
		;mov	lastarea,buf_tran+1
		;mov	lastjoin,buf_tran+2
		jmp	transbuf3_A_1
transbuf3_1:
		cjne	a,#0FAh,transbuf3_b_1	;FIRST CODE = 00 OR FF JUMP
		jmp	transbuf3_A_1
transbuf3_b_1:
		cjne	a,#0FCh,transbuf3_c_1
		jmp	transbuf3_A
transbuf3_c_1:
		clr	tranbuffull_1
		ajmp	transbuf4_1
transbuf3_A_1:
		mov	r2,#07h
		clr	a
transbuf1_1:
		add	a,@r1
		inc	r1
		djnz	r2,transbuf1_1
		cpl	a
		inc	a
		mov	buf_tran1+7,a
		mov	buf_tranptr1,#buf_tran1
		;CLR	OE0

		CLR	OE1
		;setb	oe1			;tran
		call	trandelay
		clr	ea
                MOV	A,SCON1
		SETB    ACC.1
		MOV	SCON1,A
		;setb	TI1
		setb	EA
transbuf4_1:
		ret
		

command_sense:
		DB	00h
		db	0feh,0d2h,0fdh,0d6h

transbuf:
		mov	r1,#buf_tran
		mov	a,@r1
		cjne	a,#0F5h,transbuf3	;FIRST CODE = 00 OR FF JUMP
;##############################################################################
;		check sence
;##############################################################################
		mov	r7,buf_tran+3
		mov	r2,#05			;command_sense 计数
		;mov	r3,BUF_RECE+2
		mov	dptr,#command_sense
find_sense:	mov	a,r2
		movc	a,@a+dptr
		xrl	a,r7
		jz	find_sense_1			;find sense
		djnz	r2,find_sense
		jmp	transbuf3_A
find_sense_1:
		mov	lastarea,buf_tran+1
		mov	lastjoin,buf_tran+2
		jmp	transbuf3_A
transbuf3:
		cjne	a,#0FAh,transbuf3_b	;FIRST CODE = 00 OR FF JUMP
		jmp	transbuf3_A
transbuf3_b:
		cjne	a,#0FCh,transbuf3_c
		jmp	transbuf3_A
transbuf3_c:
		clr	tranbuffull
		ajmp	transbuf4
transbuf3_A:
		mov	r2,#07h
		clr	a
transbuf1:
		add	a,@r1
		inc	r1
		djnz	r2,transbuf1
		cpl	a
		inc	a
		mov	buf_tran+7,a
		mov	buf_tranptr,#buf_tran
		JBC	RECEIVED_DATA_FROM_SLAVE,transbuf1_SLAVE
		JBC	MASTER_MUTE,MASTER_NO_SEND
		;CLR	OE
transbuf1_7B_7A:
		mov	buf_tranptr,#buf_tran
		CLR	OE0
		;setb	oe0			;tran
		call	trandelay
MASTER_NO_SEND:		
		clr	ea
		setb	TI0
		setb	EA
transbuf4:
		ret		
		
transbuf1_SLAVE:		
;##################################################################
;		MASTER NEED SEND DATA TO SLAVE
;##################################################################
		MOV	R2,#08H
		MOV	R0,#buf_tran
		MOV	R1,#buf_tran1
COPY_MASTER_TO_SLAVE_BUF_TRAN:		
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	R2,COPY_MASTER_TO_SLAVE_BUF_TRAN
		mov	buf_tranptr1,#buf_tran1
		;CLR	OE0

		CLR	OE1
		;setb	oe1			;tran
		call	trandelay
		clr	ea
                MOV	A,SCON1
		SETB    ACC.1
		MOV	SCON1,A
		;setb	TI1
		CLR	tranbuffull
		setb	EA
		RET		


; Peripheral specific initialization functions,
; Called from the Init_Device label
Timer_Init:
    mov  TCON,      #010h			;
    mov  TMOD,      #021h
    mov  CKCON,     #001h
    mov  TL0,       #0A0h
    mov  TH0,       #0A0h
    mov  TH1,       #064h
    ret

UART_Init:
    mov  SCON0,     #010h
    mov  SBRLL1,    #08Fh
    mov  SBRLH1,    #0FDh
    mov  SCON1,     #010h
    mov  SBCON1,    #043h
    CLR	RECEIVED_DATA_FROM_SLAVE
    CLR	MASTER_NEED_SLAVE_DATA
    ret

SMBus_Init:
    mov  SMB0CF,    #081h
    ret

SPI_Init:
    mov  SPI0CFG,   #070h
    mov  SPI0CN,    #00Dh
    MOV		SPI0CKR,#60
    ret

Port_IO_Init:
    ; P0.0  -  SCK  (SPI0), Open-Drain, Digital
    ; P0.1  -  MISO (SPI0), Open-Drain, Digital
    ; P0.2  -  MOSI (SPI0), Open-Drain, Digital
    ; P0.3  -  NSS  (SPI0), Open-Drain, Digital
    ; P0.4  -  TX0 (UART0), Open-Drain, Digital
    ; P0.5  -  RX0 (UART0), Open-Drain, Digital
    ; P0.6  -  SDA (SMBus), Open-Drain, Digital
    ; P0.7  -  SCL (SMBus), Open-Drain, Digital

    ; P1.0  -  TX1 (UART1), Open-Drain, Digital
    ; P1.1  -  RX1 (UART1), Open-Drain, Digital
    ; P1.2  -  Unassigned,  Open-Drain, Digital
    ; P1.3  -  Unassigned,  Open-Drain, Digital
    ; P1.4  -  Unassigned,  Open-Drain, Digital
    ; P1.5  -  Unassigned,  Open-Drain, Digital
    ; P1.6  -  Unassigned,  Open-Drain, Digital
    ; P1.7  -  Unassigned,  Open-Drain, Digital

    ; P2.0  -  Unassigned,  Open-Drain, Digital
    ; P2.1  -  Unassigned,  Open-Drain, Digital
    ; P2.2  -  Unassigned,  Open-Drain, Digital
    ; P2.3  -  Unassigned,  Open-Drain, Digital
    ; P2.4  -  Unassigned,  Open-Drain, Digital
    ; P2.5  -  Unassigned,  Open-Drain, Digital
    ; P2.6  -  Unassigned,  Open-Drain, Digital
    ; P2.7  -  Unassigned,  Open-Drain, Digital

    ; P3.0  -  Unassigned,  Open-Drain, Digital
    ; P3.1  -  Unassigned,  Open-Drain, Digital
    ; P3.2  -  Unassigned,  Open-Drain, Digital
    ; P3.3  -  Unassigned,  Open-Drain, Digital
    ; P3.4  -  Unassigned,  Open-Drain, Digital
    ; P3.5  -  Unassigned,  Open-Drain, Digital
    ; P3.6  -  Unassigned,  Open-Drain, Digital
    ; P3.7  -  Unassigned,  Open-Drain, Digital

    ;mov  P1SKIP,    #020h
    mov  XBR0,      #07h
    mov  XBR1,      #040h
    mov  XBR2,      #001h
    mov		p1mdout,#11111100b
    mov		p1,#03h
    ret

Oscillator_Init:
    mov  OSCICN,    #083h
    mov  CLKMUL,    #080h
    clr  A                     ; Wait 5us for initialization
    djnz ACC,       $
    orl  CLKMUL,    #0C0h
Osc_Wait2:
    mov  A,         CLKMUL
    jnb  ACC.5,     Osc_Wait2
    ret

Interrupts_Init:
    mov  EIE1,      #001h
    mov  EIE2,      #002h
    mov  IE,        #0D2h
    ret

; Initialization function for device,
; Call Init_Device from your main program
Init_Device:
    lcall Timer_Init
    lcall UART_Init
    lcall SMBus_Init
    lcall SPI_Init
    lcall Port_IO_Init
    lcall Oscillator_Init
    lcall Interrupts_Init
    ret
    
    
		

		
;#######################################################################
;		READ   	EEPROM 40H  TO XRAM
;		comp_buff  8 BYTE
;#######################################################################
READ_EEPROM_40H:
		MOV	DPTR,#LED_40H_XRAM
		MOV	R2,#40H
		MOV	R3,#0
		MOV	R6,#08H
READ_EEPROM_40H_1:
		mov	wordadd1,#000h
		mov	subadr,R2
		mov	bytecnt,#08
		mov	pSMB_DATA_IN,#comp_buff
		call	rcvdata_EEPROM
		MOV	R0,#comp_buff
		MOV	R1,#08H
READ_EEPROM_40H_2:
		MOV	A,@R0
		MOVX	@DPTR,A
		INC	DPTR
		INC	R2
		INC	R0
		ADD	A,R3
		MOV	R3,A
		DJNZ	R1,READ_EEPROM_40H_2
		DJNZ	R6,READ_EEPROM_40H_1
		MOV	A,R3
		CPL	A
		INC	A
		MOVX	@DPTR,A
		RET



;#######################################################################
;		CHECK  EEPROM 40H SUM
;
;#######################################################################
CHECK_EEPROM_40H:
		MOV	DPTR,#LED_40H_XRAM
		MOV	R3,#0
		MOV	R6,#4*16+1
CHECK_EEPROM_40H_1:
		MOVX	A,@DPTR
		INC	DPTR
		ADD	A,R3
		MOV	R3,A
		DJNZ	R6,CHECK_EEPROM_40H_1
		JZ	CHECK_EEPROM_40H_2
		CALL	READ_EEPROM_40H
CHECK_EEPROM_40H_2:
		RET

;#######################################################################
;		READ   	EEPROM PRESEPTR  TO XRAM
;		comp_buff  7 BYTE
;#######################################################################
READ_EEPROM_PRESEPTR:
		MOV	DPTR,#PRESEPTR_XRAM
		MOV	R4,#01H		;PRESEPTR_HIGH
		MOV	R2,#09H		;PRESEPTR_LOW
		MOV	R3,#0
		CALL	READ_EEPROM_PRESEPTR_RELASEPTR
		RET

;#######################################################################
;		READ   	EEPROM RELASEPTR  TO XRAM
;		comp_buff  7 BYTE
;#######################################################################
READ_EEPROM_RELASEPTR:
		MOV	DPTR,#RELASEPTR_XRAM
		MOV	R4,#02H		;RELASEPTR_HIGH
		MOV	R2,#49H		;RELASEPTR_LOW
		MOV	R3,#0
		CALL	READ_EEPROM_PRESEPTR_RELASEPTR
		RET

READ_EEPROM_PRESEPTR_RELASEPTR:
		MOV	R6,#16
READ_EEPROM_PRESEPTR_RELASEPTR_1:
		mov	wordadd1,R4
		mov	subadr,R2
		MOV	A,R2
		ADD	A,#10H
		MOV	R2,A
		JNC	READ_EEPROM_PRESEPTR_RELASEPTR_2
		INC	R4
READ_EEPROM_PRESEPTR_RELASEPTR_2:
		mov	bytecnt,#07
		mov	pSMB_DATA_IN,#comp_buff
		call	rcvdata_EEPROM
		MOV	R0,#comp_buff
		MOV	R1,#07H
READ_EEPROM_PRESEPTR_RELASEPTR_3:
		MOV	A,@R0
		MOVX	@DPTR,A
		INC	DPTR
		INC	R0
		ADD	A,R3
		MOV	R3,A
		DJNZ	R1,READ_EEPROM_PRESEPTR_RELASEPTR_3
		DJNZ	R6,READ_EEPROM_PRESEPTR_RELASEPTR_1
		MOV	A,R3
		CPL	A
		INC	A
		MOVX	@DPTR,A
		RET

;#######################################################################
;		CHECK  EEPROM PRESEPTR SUM
;
;#######################################################################
CHECK_EEPROM_PRESEPTR:
		MOV	DPTR,#PRESEPTR_XRAM
		MOV	R3,#0
		MOV	R6,#7*16+1
CHECK_EEPROM_PRESEPTR_1:
		MOVX	A,@DPTR
		INC	DPTR
		ADD	A,R3
		MOV	R3,A
		DJNZ	R6,CHECK_EEPROM_PRESEPTR_1
		JZ	CHECK_EEPROM_PRESEPTR_2
		CALL	READ_EEPROM_PRESEPTR
CHECK_EEPROM_PRESEPTR_2:
		RET


;#######################################################################
;		CHECK  EEPROM RELASEPTR SUM
;
;#######################################################################
CHECK_EEPROM_RELASEPTR:
		MOV	DPTR,#RELASEPTR_XRAM
		MOV	R3,#0
		MOV	R6,#7*16+1
CHECK_EEPROM_RELASEPTR_1:
		MOVX	A,@DPTR
		INC	DPTR
		ADD	A,R3
		MOV	R3,A
		DJNZ	R6,CHECK_EEPROM_RELASEPTR_1
		JZ	CHECK_EEPROM_RELASEPTR_2
		CALL	READ_EEPROM_RELASEPTR
CHECK_EEPROM_RELASEPTR_2:
		RET
;#######################################################################

;#######################################################################
;	r7 = 计数器，R4=JOIN指针，TEMP=NEW JOIN
;#######################################################################
;comp_data:
		;mov	acc,r7
		;mov	b,#10h
		;mul	ab
		;add	a,dpl
		;mov	dpl,a
		;jnc	comp_data_1
		;inc	dph
;comp_data_1:
		;mov	a,b
		;jz	comp_data_2
		;inc	dph
;comp_data_2:					;new point
		;mov	slvadr,dph
		;mov	subadr,dpl
		;mov	bytecnt,#07h
		;mov	r1,#comp_buff		;28h
		;lcall	rcvdata_EEPROM			;get panel save data
comp_data:
		mov	acc,r7
		MOV	B,#07
		mul	ab
		add	a,dpl
		mov	dpl,a
		jnc	comp_data_1
		inc	dph
comp_data_1:
		MOV	B,#07H
		mov	r1,#comp_buff
comp_data_2:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	B,comp_data_2

		mov	r1,#comp_buff
		MOV	R0,#BUF_RECE
		CLR	F0
		MOV	A,@R0			;1 BYTE COMP
		XRL	A,@R1
		JNZ	comp_data_END
		INC	R0
		INC	R1
		MOV	A,@R0
		JZ	comp_data_AREA0
		XRL	A,@R1
		JNZ	comp_data_END
;####################################################
;		AERA 相等
;####################################################
comp_data_AREA0:
		inc	r0
                inc	r1
		mov	a,@r0			;join AND <>0
		anl	a,srelase+1
		jz	comp_data_END
		inc	r0
		inc	r1
		MOV	A,@R0			;4 BYTE COMP
		XRL	A,@R1			;command byte
		JNZ	comp_data_END
		inc	r0
		inc	r1
		MOV	A,@R0			;SCENE BYTE COMP
		XRL	A,@R1
		JNZ	comp_data_END
		setb	f0
comp_data_END:
		ret

comp_data_channel:
		;mov	acc,r7
		;mov	b,#10h
		;mul	ab
		;add	a,dpl
		;mov	dpl,a
		;jnc	comp_data_1_channel
		;inc	dph
;comp_data_1_channel:
		;mov	a,b
		;jz	comp_data_2_channel
		;inc	dph
;comp_data_2_channel:					;new point
		;mov	slvadr,dph
		;mov	subadr,dpl
		;mov	bytecnt,#07h
		;mov	r1,#comp_buff		;28h
		;lcall	rcvdata_EEPROM			;get panel save data
		mov	acc,r7
		mov	b,#7h
		mul	ab
		add	a,dpl
		mov	dpl,a
		jnc	comp_data_1_channel
		inc	dph
comp_data_1_channel:
		MOV	B,#07H
		mov	r1,#comp_buff
comp_data_2_channel:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	B,comp_data_2_channel

		mov	r1,#comp_buff
		MOV	R0,#BUF_RECE
		CLR	F0
		MOV	A,@R0			;1 BYTE COMP
		XRL	A,@R1
		JNZ	comp_data_END_channel
		INC	R0
		INC	R1
		MOV	A,@R0
		JZ	comp_data_AREA0_channel
		XRL	A,@R1
		JNZ	comp_data_END_channel
;####################################################
;		AERA 相等
;####################################################
comp_data_AREA0_channel:
		inc	r0
		inc	r1
		mov	a,@r0			;join AND <>0
		anl	a,srelase+1
		jz	comp_data_END_channel
		inc	r0
		inc	r1
		MOV	A,@R0			;4 BYTE COMP
		XRL	A,@R1			;command byte
		JNZ	comp_data_END_channel
		inc	r0			;5 BYTE COMP
		inc	r1
		inc	r0
		inc	r1
		MOV	A,@R0			;6 BYTE COMP
		XRL	A,@R1			;channel byte
		JNZ	comp_data_END_channel
		inc	r0			;7 BYTE COMP
		inc	r1
		MOV	A,@R0			;channel LEVEL BYTE COMP
		XRL	A,@R1
		JNZ	comp_data_END_channel
		setb	f0
comp_data_END_channel:
		ret

onetouch_keyrelase:
		mov	a,#keytimebuf
		add	a,keynumber
		mov	r0,a
		cjne	@r0,#onetouch_TIME,onetouch_keyrelase_x
onetouch_keyrelase_x:
		jc	onetouch_relase_a	;按键时间短,跳转onetouch_relase_a
;###########################################################################
;		原key_tran_buf中有发送开的命令
;		改一个命令字节,停止渐变
;###########################################################################
		mov	a,#07
		mov	b,keynumber
		mul	ab
		ADD	A,#key_tran_buf
		add	a,#3			; 指向命令字节
		MOV	R1,A			;获得keynumber发送缓冲地址
		MOVX	a,@r1
		MOV	R0,A
		MOV	A,#0DAH
		MOVX	@r1,A			;发set tocurrent preset stop

no_71:
		call	set_tran_flag
                ;CALL    LEDPROC
		ret
onetouch_relase_a:
		mov	a,#onetouch_BUF
		add	a,keynumber
		mov	r0,a
		mov	ACC,@r0
		jbc	acc.0,onetouch_relase_b
                SETB    LEDON                   ;20060704
		setb	acc.0
		mov	@r0,acc			;save to onetouch_BUF
		mov	dptr,#preseptr
		call	COPY_EEROM_TO_key_tran_buf
		clr	c
		mov	a,dpl
		subb	a,#07
		mov	subadr,a
		mov	bytecnt,#04h
		mov	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM			;获得 KEY 的状态
		mov	a,RTC_BUF+1
		JB	ACC.0,OLD_PROC
;###################################################################################
;		FADE PRESET TO PRESET FIRST PRSET
;###################################################################################
		MOV	A,R3			;R3=BUF TRAN ADDRE
		ADD	A,#4
		MOV	R1,A
		MOV	A,RTC_BUF+2
		MOVX	@R1,A			;SAVE BANK
		;setb	ledon


OLD_PROC:
		call	set_tran_flag
                ;CALL    LEDPROC
		ret
;##########################################################################
;		原key_tran_buf中有发送开的命令
;		改一个命令字节,渐变到关
;##########################################################################

onetouch_relase_b:
		mov	@r0,acc
		CLR	LEDON			;LED OFF
		mov	dptr,#preseptr-6
		mov	a,#10h
		mov	b,keynumber
		mul	ab
		mov	r0,a
		mov	a,dpl
		add	a,r0
		mov	dpl,a
		jnc	onetouch_relase_b_A_ADD
		inc	dph
onetouch_relase_b_A_ADD:
		mov	subadr,a
		mov	a,dph
		mov	wordadd1,a
		mov	bytecnt,#03h
		mov	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM			;获得 KEY 的状态
		mov	a,RTC_BUF
		JB	ACC.0,OLD_PROC_relase_b
;########################################################################
;		FADE T0 PRSET TO PRESET
;########################################################################
		mov	a,#07
		mov	b,keynumber
		mul	ab
		ADD	A,#key_tran_buf+4
		MOV	R1,A			;获得keynumber发送缓冲地址
		MOV	A,RTC_BUF+2
		MOVX	@R1,A
		JMP	yes_76h_a
OLD_PROC_relase_b:
		mov	a,#07
		mov	b,keynumber
		mul	ab
		ADD	A,#key_tran_buf+3
		;add	a,#3			; 指向命令字节
		MOV	R1,A			;获得keynumber发送缓冲地址
		MOV	A,#0DCH
		movX	@r1,A			;发 Fade channel to off
yes_76h_a:
		call	set_tran_flag
		ret
		;ljmp	REPET
;########################################################################
;		根据keynumber,置位发送标志
;		use ACC,R0,出口 R0=标志字节的地址
;#########################################################################
set_tran_flag:
		MOV	A,keynumber
		MOV	B,#07H
		MUL	AB
		ADD	A,#key_tran_buf
		MOV	R0,A
		MOVX	A,@R0
		CJNE	A,#0F5H,set_tran_flag_NO1C
		JMP	set_tran_flag_SEND
set_tran_flag_NO1C:
		CJNE	A,#0FAH,set_tran_flag_NO5C
		JMP	set_tran_flag_SEND
set_tran_flag_NO5C:
		CJNE	A,#0FCH,set_tran_flag_END
set_tran_flag_SEND:
		mov	a,#onetouch_BUF
		add	a,keynumber
		mov	r0,a
		mov	a,#00000100b
		orl	a,@r0			;置位发送标志
		mov	@r0,a
set_tran_flag_END:
		RET


;########################################################################
;		根据keynumber,DPTR 将EEROM的数据
;		COPY TO key_tran_buf
;		use ACC,R0,,R1,B,R3出口 R3,R1=keynumber发送缓冲地址
;#########################################################################
COPY_EEROM_TO_key_tran_buf:
		mov	a,#07
		mov	b,keynumber
		mul	ab
		ADD	A,#key_tran_buf
		MOV	pSMB_DATA_IN,#KEY_READ_BUFF	;获得keynumber发送缓冲地址
		MOV	R3,A			;SAVE 发送缓冲地址
		mov	a,#10h
		mov	b,keynumber
		mul	ab
		mov	r0,a
		mov	a,dpl
		add	a,r0
		mov	dpl,a
		jnc	COPY_EEROM_TO_key_tran_buf_A
		inc	dph
COPY_EEROM_TO_key_tran_buf_A:
		mov	subadr,a
		mov	a,dph
		ADD	A,B
		mov	wordadd1,a
		mov	bytecnt,#07h
		call	rcvdata_EEPROM			;KEY DATA TO KEY_READ_BUFF
		mov     a,KEY_READ_BUFF
                cjne    a,#0F5h,jump_change_join
                MOV     R0,#KEY_READ_BUFF+2     ;R0=JOIN
                MOV     A,keyjoinnew
                JZ	jump_change_join
                MOV     @R0,A                   ;SET NEW JOIN
jump_change_join:
		MOV	A,R3
		MOV	R0,A			;keynumber发送缓冲地址
		MOV	R1,#KEY_READ_BUFF
		MOV	B,#07H
COPY_KEY_READ_BUFF_TO_key_tran_buf:
		MOV	A,@R1
		MOVX	@R0,A
		INC	R0
		INC	R1
		DJNZ	B,COPY_KEY_READ_BUFF_TO_key_tran_buf
		MOV	A,R3
		MOV	R1,A
		RET

onetouch_long_prese:
;onrtouchlong:
		mov	dptr,#preseptr
		mov	TEMP,R0			;SAVE R0
		call	COPY_EEROM_TO_key_tran_buf
		mov	acc,r1
		ADD	A,#03H
		mov	R1,ACC
		MOV	R0,TEMP
                MOV     RTC_BUF+2,R1
                MOV	A,SUBADR
                CLR	C
                SUBB	A,#03H
                MOV     RTC_BUF+1,R0
                MOV	SUBADR,A
                 mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		MOV	R1,RTC_BUF+2
		INC	R1
		MOV	A,RTC_BUF
		MOVX	@R1,A
		DEC	R1
                MOV     R0,RTC_BUF+1
		MOV	A,@R0
		mov	r2,a
		anl	a,#01h
		JNZ	onetouch_OFF		;down
		mov	a,r2
		orl	a,#01h
		MOV	@R0,a			;up
                MOV     A,#0DBH
		MOVX	@R1,A
		AJMP	presetime3
onetouch_OFF:					;down
		mov	a,r2
		anl	a,#0feh
		MOV	@R0,a			;CLEAR
                MOV     A,#0DCH
		MOVX	@R1,A
		;inc	r1
		;mov	a,@R1
		;mov	a,BUF_TRAN+4
		;mov	b,#05h
		;div	ab
		;inc	r1
		;mov	@R1,ACC
		;mov	BUF_TRAN+5,a
		clr	ledon
		;mov	a,dpl
		;clr	c
		;subb	a,#2
		;mov	subadr,a
		;mov	bytecnt,#01h
		;DEC	R1
		;mov	r1,#buf_tran+4
		;call	rcvdata_EEPROM

presetime3:
		call	set_tran_flag
		MOV	R0,TEMP
		ret

key_buf_tran:
		mov	a,@r0
		jnb	acc.2,key_buf_tran_end	;不需要发送跳转
		setb	tranbuffull
;#############################################################
;		从key_tran_buf COPY TO BUF_TRAN
;#############################################################
		MOV	TEMP,R0
		mov	a,r7
		dec	a
		mov	b,#07			;mov	b,r5
		;mov	b,keynumber
		mul	ab
		ADD	A,#key_tran_buf
		MOV	R0,A			;获得keynumber发送缓冲地址
		MOV	R1,#BUF_TRAN
		MOV	R6,#07
COPY_key_tran_buf_TO_BUF_TRAN:
		MOVX	a,@R0
		MOV	@R1,ACC
		INC	R0
		INC	R1
		DJNZ	R6,COPY_key_tran_buf_TO_BUF_TRAN
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		MOV	R0,TEMP
		SETB	MASTER_MUTE
		SETB	tranready
		
;#############################################################
;		检查是否要发送二次
;
;#############################################################
		MOV	ACC,@R0
		JBC	ACC.3,TWO_key_buf_tran
CLEAR_key_buf_tran:
		CLR	ACC.2
		MOV	@R0,ACC
key_buf_tran_end:
		ret
TWO_key_buf_tran:
		MOV	@R0,ACC
		ret
		
COPY_BUF_TRAN_TO_BUF_TRAN1:		
;#############################################################
;		COPY BUF_TRAN TO BUF_TRAN1
;#############################################################
		MOV	R6,#07
		MOV	R0,#BUF_TRAN
		MOV	R1,#BUF_TRAN1
COPY_BUF_TRAN_TO_BUF_TRAN1_1:		
		MOV	a,@R0
		MOV	@R1,ACC
		INC	R0
		INC	R1
		DJNZ	R6,COPY_BUF_TRAN_TO_BUF_TRAN1_1
		SETB	tranready_1
		RET		

;##########################################################################
;		IN ACC=TASK NUMBER,USE R1,R0,R7
;##########################################################################
READ_TASK_POINT_T0_7_VECTOR:
		RL	A			;TASK NUMBER * 2
		MOV	R7,A			;SAVE OFFSET
		ADD	A,#TASK_ADDEREE - 2
;##########################################################################
;		READ TASK POINT TO T0_7_VECTOR
;##########################################################################
		MOV	SUBADR,A
		MOV	A,R7
		ADD	A,#T0_7_COUNT - 2
		MOV	R1,A
		CLR	A
		MOV	@R1,A
		INC	R1
		MOV	@R1,A			;T0_7_COUNT = 0
		MOV	A,R7
		ADD	A,#T0_7_VECTOR-2
		MOV	pSMB_DATA_IN,A
		CALL	READTWOBYTE
		RET
;############################################################################
;		GET TASK NUMBER  A=TASK BIT RETURN R1=TASK N0. 1~8
;############################################################################
GET_TASK_NO:
		MOV	R1,#00H
		CLR	C
GET_TASK_NO_1:
		INC	R1
		RRC	A
		JNC	GET_TASK_NO_1
		RET
;############################################################################
;		T0_7_COUNT CLEAR ACC=NUMBER
;############################################################################
T0_7_COUNT_CLEAR:
		RL	A			;TASK NUMBER * 2
		ADD	A,#T0_7_COUNT - 2
		MOV	R1,A
		CLR	A
		MOV	@R1,A
		INC	R1
		MOV	@R1,A			;TIME CONTER CLEAR
		RET
;##########################################################################
;		TASK PROC
;
;##########################################################################
TASK_PROC:
		;INC	AUXR1			;CHANGE DPTR

;##########################################################################
;		GET_COMMAND
;##########################################################################
GET_COMMAND:
		MOV	wordadd1,DPH
		mov	BYTECNT,#1	;读
		mov	pSMB_DATA_IN,#temp	;存储在80h
		mov	SUBADR,DPL
		call	rcvdata_EEPROM
		MOV	A,TEMP
		JZ	TASK_END_DELAY
		CJNE	A,#0FFH,TASK_PROC_04
;#################################################################
;		READ COMMAND = 0 OR 0FFH END STOP TASK
;#################################################################
		JMP	TASK_END_DELAY
TASK_PROC_04:
		CJNE	A,#04H,TASK_PROC_NO_04
;##########################################################################
;		DELAY	PROC  04 COMMAND
;##########################################################################
		MOV     A,R4
                MOV	pSMB_DATA_IN,A		;DELAY BUFFER POINT
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		call	rcvdata_EEPROM
		INC	DPTR
		INC	DPTR
TASK_END_DELAY:
		CLR	TASK_ENABLE
TASK_PROC_END:

		RET
TASK_PROC_NO_04:
		CJNE	A,#05H,TASK_PROC_NO_05
;##########################################################################
;		CANCEL TASK PROC  05 COMMAND
;##########################################################################
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		INC	DPTR
		INC	DPTR			;NEXT POINT
		;SETB	TASK_ENABLE
		MOV	A,RTC_BUF
		CPL	A
		ANL	PR_LEVEL,A		;PR_LEVEL TASK BIT = 0
		MOV	A,RTC_BUF
		CALL	GET_TASK_NO
		MOV	A,R1			;R1 = TASK NUMBER
		CALL	T0_7_COUNT_CLEAR
		SJMP	TASK_PROC_END
TASK_PROC_NO_05:
		CJNE	A,#0CH,TASK_PROC_NO_0C
;##########################################################################
;		START TASK PROC  0C COMMAND D7=TASK8,D6=TASK5 ...D0=TASK1
;##########################################################################
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		INC	DPTR
		INC	DPTR			;NEXT POINT
		MOV	A,(RTC_BUF)
		JZ	TASK_PROC_END
		SETB	TASK_ENABLE
		ORL	PR_LEVEL,A		;PR_LEVEL TASK BIT = 1
		CALL	GET_TASK_NO
		MOV	A,R1			;R1 = TASK NUMBER
		CALL	READ_TASK_POINT_T0_7_VECTOR

		RET
TASK_PROC_NO_0C:
		CJNE	A,#06H,TASK_PROC_NO_06
;##########################################################################
;		JUMP PROC  06 COMMAND
;##########################################################################
		SETB	TASK_ENABLE
		MOV	pSMB_DATA_IN,#RTC_BUF		;JUMP DATA BUFFER POINT
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		call	rcvdata_EEPROM
		MOV	R1,#RTC_BUF
		MOV	A,@R1			;GET NEW DPTR HIGH BYTE
		MOV	DPH,A
		INC	R1
		MOV	A,@R1			;GET NEW DPTR LOW BYTE
		MOV	DPL,A
		RET
TASK_PROC_NO_06:
		CJNE	A,#03H,TASK_PROC_NO_03
;##########################################################################
;		NewVector PROC  03h COMMAND
;		change task first point
;##########################################################################
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
;#######################################################################
;	get new vector  to RTC_BUF save to task begin point
;#######################################################################
		INC	DPTR
		INC	DPTR			;NEXT POINT
		MOV	A,R3
		RL	A			;A=A*2
		ADD	A,#TASK_ADDEREE-2
		MOV	wordadd1,#00H
		mov	SUBADR,A
		mov	BYTECNT,#2
		MOV	pSMB_DATA_OUT,#RTC_BUF
		call	SendData_EEPROM
		call	waitwriteend
                CLR	TASK_ENABLE
		RET
TASK_PROC_NO_03:
		;SETB	TASK_ENABLE
		JMP	TASK_PROC_ASM
TASK_PROC_NO_03_ADD:
		SETB    TASK_WAIT_TR
		JB	tranbuffull,NO_ADD_DPH
                CLR     TASK_WAIT_TR
		MOV	wordadd1,DPH
		mov	BYTECNT,#7		;读
		mov	pSMB_DATA_IN,#BUF_TRAN		;存储在BUF_TRAN
		mov	SUBADR,DPL
		call	rcvdata_EEPROM
		mov	a,BUF_TRAN
		cjne	a,#0ffh,TASK_PROC_NO_03_ADD_1
		CLR	TASK_ENABLE
		ret
TASK_PROC_NO_03_ADD_1:
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		SETB	MASTER_MUTE
		setb	tranready
		;setb	rereceive
		SETB	tranbuffull
		MOV	A,DPL
		ADD	A,#07H
		MOV	DPL,A
		JNC	NO_ADD_DPH
		INC	DPH
NO_ADD_DPH:
		RET

TASK_PROC_ASM:

		CJNE	A,#030H,TASK_PROC_ASM_NO_LDA
;###########################################################################
;		ASM	LDA PROC
;###########################################################################
ASM_LDA_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,TEMP
		MOV	ASM_ACC,A		;DATA TO ASM_ACC
		CLR	ASM_ZERO
		JNZ	ASM_LDA_PROC_END
		SETB	ASM_ZERO
ASM_LDA_PROC_END:
		RET
TASK_PROC_ASM_NO_LDA:
		CJNE	A,#031H,TASK_PROC_ASM_NO_STA
;###########################################################################
;		ASM	STA PROC
;###########################################################################
ASM_STA_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
;############################################################################
;		RTF  INDECATE  DATA SAVE TO
;############################################################################
		MOV	A,RTC_BUF
		CJNE	A,#050H,SAVE_TO_NO_BUF_RECE
		MOV	A,ASM_ACC
		MOV	@R1,A
		RET
SAVE_TO_NO_BUF_RECE:
		ANL     A,#0E0H
		CJNE	A,#80H,SAVE_TO_NO_USER_RAM
		MOV	A,ASM_ACC
		MOV	@R1,A
		RET
SAVE_TO_NO_USER_RAM:
		CJNE	A,#0C0H,SAVE_TO_NO_USER_RAM_WITH_X
		MOV	A,ASM_ACC
		MOV	@R1,A
		RET
SAVE_TO_NO_USER_RAM_WITH_X:
		CJNE	A,#020H,SAVE_TO_NO_BUF_RECE_WITH_X
		MOV	A,ASM_ACC
		MOV	@R1,A

SAVE_TO_NO_BUF_RECE_WITH_X:
		RET
		;MOV	A,ASM_ACC
		;MOV	@R1,A
ASM_STA_PROC_END:
		RET
TASK_PROC_ASM_NO_STA:
		CJNE	A,#032H,TASK_PROC_ASM_NO_ADD
;###########################################################################
;		ASM	ADD PROC
;###########################################################################
ASM_ADD_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		ADD	A,TEMP
		MOV	ASM_ACC,A		;SAVE ACC TO ASM_ACC
		MOV	ASM_CARRY,C
		CLR	ASM_ZERO
		JNZ	ASM_ADD_PROC_END
		SETB	ASM_ZERO
ASM_ADD_PROC_END:
		RET
TASK_PROC_ASM_NO_ADD:
		CJNE	A,#033H,TASK_PROC_ASM_NO_SUB
;###########################################################################
;		ASM	SUB PROC
;###########################################################################
ASM_SUB_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		CLR	C
		SUBB	A,TEMP
		MOV	ASM_ACC,A		;SAVE ACC TO ASM_ACC
		MOV	ASM_CARRY,C
		CLR	ASM_ZERO
		JNZ	ASM_SUB_PROC_END
		SETB	ASM_ZERO
ASM_SUB_PROC_END:
		RET
TASK_PROC_ASM_NO_SUB:
		CJNE	A,#034H,TASK_PROC_ASM_NO_MUL
;###########################################################################
;		ASM	MUL PROC A*B B=HIGH A =LOW
;###########################################################################
ASM_MUL_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		MOV	B,ASM_X
		MUL	AB
		MOV	ASM_X,B
		MOV	ASM_ACC,A
		CLR	ASM_CARRY
		CLR	ASM_ZERO
		JNZ	ASM_MUL_PROC_END
		SETB	ASM_ZERO
ASM_MUL_PROC_END:
		RET
TASK_PROC_ASM_NO_MUL:
		CJNE	A,#035H,TASK_PROC_ASM_NO_DIV
;###########################################################################
;		ASM	DIV PROC
;###########################################################################
ASM_DIV_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		MOV	B,ASM_X
		DIV	AB
		MOV	ASM_X,B
		MOV	ASM_ACC,A
		CLR	ASM_CARRY
		CLR	ASM_ZERO
		JNZ	ASM_DIV_PROC_END
		SETB	ASM_ZERO
ASM_DIV_PROC_END:
		RET
TASK_PROC_ASM_NO_DIV:
		CJNE	A,#036H,TASK_PROC_ASM_NO_INC
;###########################################################################
;		ASM	INC PROC
;###########################################################################
ASM_INC_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		ADD	A,#1
		MOV	ASM_ACC,A
		MOV	ASM_CARRY,C
		CLR	ASM_ZERO
		JNZ	ASM_INC_PROC_END
		SETB	ASM_ZERO
ASM_INC_PROC_END:
		RET
TASK_PROC_ASM_NO_INC:
		CJNE	A,#037H,TASK_PROC_ASM_NO_DEC
;###########################################################################
;		ASM	DEC PROC
;###########################################################################
ASM_DEC_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		CLR	C
		SUBB	A,#1
		MOV	ASM_ACC,A
		MOV	ASM_CARRY,C
		CLR	ASM_ZERO
		JNZ	ASM_DEC_PROC_END
		SETB	ASM_ZERO
ASM_DEC_PROC_END:
		RET
TASK_PROC_ASM_NO_DEC:
		CJNE	A,#44H,TASK_PROC_ASM_NO_SHL
;###########################################################################
;		ASM	SHL PROC
;###########################################################################
ASM_SHL_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		RL	A
		MOV	ASM_ACC,A
		RET
TASK_PROC_ASM_NO_SHL:
		CJNE	A,#45H,TASK_PROC_ASM_NO_SHR
;###########################################################################
;		ASM	SHR PROC
;###########################################################################
ASM_SHR_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		RR	A
		MOV	ASM_ACC,A
		RET
TASK_PROC_ASM_NO_SHR:
		CJNE	A,#46H,TASK_PROC_ASM_NO_ROL
;###########################################################################
;		ASM	ROL PROC  ROTATE LEFT THROUGH CARRY
;###########################################################################
ASM_ROL_PROC:
		INC	DPTR
		MOV	C,ASM_CARRY
		MOV	A,ASM_ACC
		RLC	A
		MOV	ASM_CARRY,C
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_ROL_PROC_END
		SETB	ASM_ZERO
ASM_ROL_PROC_END:
		RET
TASK_PROC_ASM_NO_ROL:
		CJNE	A,#47H,TASK_PROC_ASM_NO_ROR
;###########################################################################
;		ASM	ROR PROC  ROTATE RIGHT THROUGH CARRY
;###########################################################################
ASM_ROR_PROC:
		INC	DPTR
		MOV	C,ASM_CARRY
		MOV	A,ASM_ACC
		RRC	A
		MOV	ASM_CARRY,C
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_ROR_PROC_END
		SETB	ASM_ZERO
ASM_ROR_PROC_END:
		RET
TASK_PROC_ASM_NO_ROR:
		CJNE	A,#40H,TASK_PROC_ASM_NO_AND
;###########################################################################
;		ASM	AND PROC
;###########################################################################
ASM_AND_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		ANL	A,TEMP
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_AND_PROC_END
		SETB	ASM_ZERO
ASM_AND_PROC_END:
		RET
TASK_PROC_ASM_NO_AND:
		CJNE	A,#41H,TASK_PROC_ASM_NO_OR
;###########################################################################
;		ASM	OR PROC
;###########################################################################
ASM_OR_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		ORL	A,TEMP
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_OR_PROC_END
		SETB	ASM_ZERO
ASM_OR_PROC_END:
		RET
TASK_PROC_ASM_NO_OR:
		CJNE	A,#43H,TASK_PROC_ASM_NO_XOR
;###########################################################################
;		ASM	XOR PROC
;###########################################################################
ASM_XOR_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		XRL	A,TEMP
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_XOR_PROC_END
		SETB	ASM_ZERO
ASM_XOR_PROC_END:
		RET
TASK_PROC_ASM_NO_XOR:
		CJNE	A,#070H,TASK_PROC_ASM_NO_LDX
;###########################################################################
;		ASM	LDX PROC
;###########################################################################
ASM_LDX_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,TEMP
		MOV	ASM_X,A		;DATA TO ASM_X
		RET
TASK_PROC_ASM_NO_LDX:
		CJNE	A,#071H,TASK_PROC_ASM_NO_STX
;###########################################################################
;		ASM	STX PROC
;###########################################################################
ASM_STX_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_X
		MOV	@R1,A
ASM_STX_PROC_END:
		RET
TASK_PROC_ASM_NO_STX:
		CJNE	A,#076H,TASK_PROC_ASM_NO_INCX
;###########################################################################
;		ASM	INCX PROC
;###########################################################################
ASM_INCX_PROC:
		INC	DPTR
		MOV	A,ASM_X
		INC     A
		MOV	ASM_X,A
		RET
TASK_PROC_ASM_NO_INCX:
		CJNE	A,#077H,TASK_PROC_ASM_NO_DECX
;###########################################################################
;		ASM	DECX PROC
;###########################################################################
ASM_DECX_PROC:
		INC	DPTR
		MOV	A,ASM_X
		DEC     A
		MOV	ASM_X,A
		RET
TASK_PROC_ASM_NO_DECX:
		CJNE	A,#079H,TASK_PROC_ASM_NO_TAX
;###########################################################################
;		ASM	TAX PROC
;###########################################################################
ASM_TAX_PROC:
		INC	DPTR
		MOV	A,ASM_ACC
		MOV	ASM_X,A
		RET
TASK_PROC_ASM_NO_TAX:
		CJNE	A,#07AH,TASK_PROC_ASM_NO_TXA
;###########################################################################
;		ASM	TXA PROC
;###########################################################################
ASM_TXA_PROC:
		INC	DPTR
		MOV	A,ASM_X
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_TXA_PROC_END
		SETB	ASM_ZERO
ASM_TXA_PROC_END:
		RET
TASK_PROC_ASM_NO_TXA:
		CJNE	A,#074H,TASK_PROC_ASM_NO_SWAP
;###########################################################################
;		ASM	SWAP PROC
;###########################################################################
ASM_SWAP_PROC:
		INC	DPTR
		MOV	TEMP,ASM_ACC
		MOV	A,ASM_X
		MOV	ASM_X,TEMP
		MOV	ASM_ACC,A
		CLR	ASM_ZERO
		JNZ	ASM_SWAP_PROC_END
		SETB	ASM_ZERO
ASM_SWAP_PROC_END:
		RET
TASK_PROC_ASM_NO_SWAP:
		CJNE	A,#38H,TASK_PROC_ASM_NO_CMP
;###########################################################################
;		ASM	CMP PROC
;###########################################################################
ASM_CMP_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_ACC
		CJNE	A,TEMP,ASM_CMP_NO_EQU
ASM_CMP_EQU:
		SETB	ASM_ZERO
		CLR	ASM_CARRY
		CLR	ASM_PLUS
		CLR	ASM_MINUS
		RET
ASM_CMP_NO_EQU:
		JC	ASM_CMP_NO_MINUS
ASM_CMP_PLUS:
		CLR	ASM_ZERO
		CLR	ASM_CARRY
		SETB	ASM_PLUS
		CLR	ASM_MINUS
		RET

ASM_CMP_NO_MINUS:
		CLR	ASM_ZERO
		SETB	ASM_CARRY
		SETB	ASM_MINUS
		CLR	ASM_PLUS
		RET
TASK_PROC_ASM_NO_CMP:
		CJNE	A,#78H,TASK_PROC_ASM_NO_CMPX
;###########################################################################
;		ASM	CMPX PROC
;###########################################################################
ASM_CMPX_PROC:
		CALL	GET_ASM_COMMAND
		CALL	ASM_GET_DATA
		MOV	A,ASM_X
		CJNE	A,TEMP,ASM_CMPX_NO_EQU
ASM_CMPX_EQU:
		SETB	ASM_ZERO
		CLR	ASM_CARRY
		CLR	ASM_PLUS
		CLR	ASM_MINUS
		RET
ASM_CMPX_NO_EQU:
		JC	ASM_CMPX_NO_MINUS
ASM_CMPX_PLUS:
		CLR	ASM_ZERO
		CLR	ASM_CARRY
		SETB	ASM_PLUS
		CLR	ASM_MINUS
		RET

ASM_CMPX_NO_MINUS:
		CLR	ASM_ZERO
		SETB	ASM_CARRY
		SETB	ASM_MINUS
		CLR	ASM_PLUS
		RET
TASK_PROC_ASM_NO_CMPX:
		CJNE	A,#50H,TASK_PROC_ASM_NO_BRZ
;###########################################################################
;		ASM	BRZ PROC
;###########################################################################
ASM_BRZ_PROC:
		CALL	GET_BR_ADDRESS
		JNB	ASM_ZERO,ASM_BRZ_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BRZ_PROC_END:
		RET

TASK_PROC_ASM_NO_BRZ:
		CJNE	A,#51H,TASK_PROC_ASM_NO_BPL
;###########################################################################
;		ASM	BPL PROC
;###########################################################################
ASM_BPL_PROC:
		CALL	GET_BR_ADDRESS
		JNB	ASM_PLUS,ASM_BPL_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BPL_PROC_END:
		RET
TASK_PROC_ASM_NO_BPL:
		CJNE	A,#52H,TASK_PROC_ASM_NO_BMI
;###########################################################################
;		ASM	BMI PROC
;###########################################################################
ASM_BMI_PROC:
		CALL	GET_BR_ADDRESS
		JNB	ASM_MINUS,ASM_BMI_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BMI_PROC_END:
		RET
TASK_PROC_ASM_NO_BMI:
		CJNE	A,#53H,TASK_PROC_ASM_NO_BRC
;###########################################################################
;		ASM	BRC PROC
;###########################################################################
ASM_BRC_PROC:
		CALL	GET_BR_ADDRESS
		JNB	ASM_CARRY,ASM_BRC_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BRC_PROC_END:
		RET
TASK_PROC_ASM_NO_BRC:
		CJNE	A,#54H,TASK_PROC_ASM_NO_BRA
;###########################################################################
;		ASM	BRA PROC
;###########################################################################
ASM_BRA_PROC:
		CALL	GET_BR_ADDRESS
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BRA_PROC_END:
		RET
TASK_PROC_ASM_NO_BRA:
		CJNE	A,#55H,TASK_PROC_ASM_NO_BNE
;###########################################################################
;		ASM	BNE PROC
;###########################################################################
ASM_BNE_PROC:
		CALL	GET_BR_ADDRESS
		JB	ASM_ZERO,ASM_BNE_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BNE_PROC_END:
		RET
TASK_PROC_ASM_NO_BNE:
		CJNE	A,#56H,TASK_PROC_ASM_NO_BCC
;###########################################################################
;		ASM	BCC PROC
;###########################################################################
ASM_BCC_PROC:
		CALL	GET_BR_ADDRESS
		JB	ASM_CARRY,ASM_BCC_PROC_END
		MOV	DPH,RTC_BUF+1
		MOV	DPL,RTC_BUF+2		;TEMP
ASM_BCC_PROC_END:
		RET
TASK_PROC_ASM_NO_BCC:
		CJNE	A,#65H,TASK_PROC_ASM_NO_COPY
;###########################################################################
;		ASM	COPY PROC
;###########################################################################
ASM_COPY_PROC:
		CALL	GET_BR_ADDRESS
		MOV	A,RTC_BUF
		ANL	A,#0F0H
		CJNE	A,#50H,ASM_COPY_PROC_A
;##################################################################3
;               copy from @ to ~
;###################################################################
		MOV	A,RTC_BUF
		ANL	A,#07H
		ADD	A,#BUF_RECE
		MOV	R0,A			;SOUSE
		JMP	ASM_COPY_PROC_B
ASM_COPY_PROC_A:
		CJNE	A,#60H,ASM_COPY_PROC_D
;##############################################################
;		COPY FROM EEROM T0 @ OR USER RAM
;##############################################################
		MOV	RTC_BUF,RTC_BUF+1
		mov	RTC_BUF+1,RTC_BUF+2
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#2
		MOV	pSMB_DATA_IN,#RTC_BUF+2
		call	rcvdata_EEPROM
		INC	DPTR
		INC	DPTR			;NEXT POINT
		MOV	wordadd1,RTC_BUF
		mov	SUBADR,RTC_BUF+1
		mov	BYTECNT,TEMP
		MOV	A,RTC_BUF+2
		ANL	A,#0F0H
		CJNE	A,#50H,ASM_COPY_PROC_E
;################################################################
;		COPY TO @0~@7
;################################################################
		MOV	A,RTC_BUF+2
		ANL	A,#07H
		ADD	A,#BUF_RECE
		JMP	ASM_COPY_PROC_F
ASM_COPY_PROC_E:
;################################################################
;		COPY TO USER RAM
;################################################################
		MOV	A,RTC_BUF+2
		ANL	A,#1FH			;GET ADDRE
                ADD     A,#ASM_USR_RAM
ASM_COPY_PROC_F:
		MOV	pSMB_DATA_IN,A
		CALL	rcvdata_EEPROM
                RET
ASM_COPY_PROC_D:
		MOV	A,RTC_BUF
		ANL	A,#01FH
                ADD     A,#ASM_USR_RAM
		MOV	R0,A
ASM_COPY_PROC_B:
		MOV	A,RTC_BUF+1
		ANL	A,#1FH
                ADD     A,#ASM_USR_RAM
		MOV	R1,A
ASM_COPY_PROC_C:
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	RTC_BUF+2,ASM_COPY_PROC_C
		RET
TASK_PROC_ASM_NO_COPY:
		CJNE	A,#064H,TASK_PROC_ASM_NO_CANTX
;###########################################################################
;		ASM	CANTX PROC
;###########################################################################
ASM_CANTX_PROC:
		INC	DPTR
		INC	DPTR
		MOV	ASM_ACC,#00H
                SETB    ASM_ZERO
		JB	BUF_TRAN_BUSY,ASM_CANTX_PROC_END
		MOV	ASM_ACC,#0FFH
                CLR     ASM_ZERO
ASM_CANTX_PROC_END:
		RET
TASK_PROC_ASM_NO_CANTX:
		CJNE	A,#0AH,TASK_PROC_ASM_NO_PAUSETASK
;###########################################################################
;		ASM	PAUSETASK() PROC
;###########################################################################
ASM_PAUSETASK_PROC:
		CALL	GET_ASM_COMMAND
		INC	DPTR
		MOV	A,RTC_BUF
		CPL	A
		ANL	A,PR_LEVEL
		MOV	PR_LEVEL,A
		MOV	A,RTC_BUF
		CALL	GET_TASK_NO
		MOV	A,R1
		RL	A
		ADD	A,#T0_7_COUNT-2
		MOV	R1,A
		CLR	A
		MOV	@R1,A
		INC	R1
		MOV	@R1,A
		RET
TASK_PROC_ASM_NO_PAUSETASK:
		CJNE	A,#0BH,TASK_PROC_ASM_NO_RESTARTTASK
;###########################################################################
;		ASM	RESTARTTASK() PROC
;###########################################################################
ASM_RESTARTTASK_PROC:
		CALL	GET_ASM_COMMAND
		INC	DPTR
		MOV	A,RTC_BUF
		ORL	PR_LEVEL,A
		RET
TASK_PROC_ASM_NO_RESTARTTASK:
		CJNE	A,#60H,TASK_PROC_ASM_NO_TX
;###########################################################################
;		ASM	TX() PROC
;###########################################################################
ASM_TX_PROC:
		SETB    TASK_WAIT_TR
		JB	tranbuffull,ASM_TX_PROC_COPY_A_add
		clr	TASK_WAIT_TR
		jb	BUF_TRAN_BUSY,TASK_PROC_ASM_NO_TX
		CALL	GET_BR_ADDRESS
		MOV	A,RTC_BUF
		ANL	A,#0F0H
		CJNE	A,#50H,ASM_TX_PROC_ASM_RAM
                MOV	A,RTC_BUF
		ANL	A,#00001111B
                ADD     A,#BUF_RECE
		MOV	R0,A
                JMP	ASM_TX_PROC_COPY
ASM_TX_PROC_ASM_RAM:
		MOV	A,RTC_BUF
		ANL	A,#00011111B
                ADD     A,#ASM_USR_RAM
		MOV	R0,A
ASM_TX_PROC_COPY:
		MOV	R1,#BUF_TRAN
		SETB	BUF_TRAN_BUSY
ASM_TX_PROC_COPY_A:
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	RTC_BUF+2,ASM_TX_PROC_COPY_A
		SETB	tranready
ASM_TX_PROC_COPY_A_add:
		RET
TASK_PROC_ASM_NO_TX:


		JMP	TASK_PROC_NO_03_ADD


;############################################################################
;		GET ASM COMMAND 3 BYTE BUT DPTR ONLY +2
;		RETURN   COMMAND SAVE TO RTC_BUF
;############################################################################
GET_ASM_COMMAND:
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#3
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		INC	DPTR
		RET


;##########################################################################
;		DATA SAVE TEMP
;##########################################################################
ASM_GET_DATA:
		MOV	A,RTC_BUF
		ANL	A,#11110000B
		CJNE	A,#70H,ASM_DATA_NO_70
;##########################################################################
;		#  读入立即数	LDA	#
;##########################################################################
		MOV	TEMP,RTC_BUF+1
		INC	DPTR
		RET
ASM_DATA_NO_70:
		CJNE	A,#50H,ASM_DATA_NO_50
;##########################################################################
;		@  读入RS485输入缓冲数据	;LDA	@
;##########################################################################
		MOV	A,RTC_BUF
		ANL	A,#07H
		MOV	R1,#BUF_RECE
		ADD	A,R1
		MOV	R1,A
		MOV	TEMP,@R1
		RET
ASM_DATA_NO_50:
		CJNE	A,#40H,ASM_DATA_NO_40
;##########################################################################
;		^  读入接口数据	;LDA	^            10  keyboard
;##########################################################################
		MOV	A,RTC_BUF+1
		MOV	TEMP,#00H
		CJNE	A,#010,ASM_DATA_NO_KEYPORT
		MOV	TEMP,ASM_KEY_MAP		;LDA ^10,0
		JMP	ASM_GET_DATA_END
ASM_DATA_NO_KEYPORT:
		;CJNE	A,#017,ASM_GET_DATA_END
		;MOV	TEMP,ASM_KEY_SWITCH_MAP		;LDA ^17,0
ASM_GET_DATA_END:
		INC	DPTR
		INC	DPTR
		RET
ASM_DATA_NO_40:
		CJNE	A,#30H,ASM_DATA_NO_30
;#####################################################################
;		Load x register from EE		LDX	LAB/NUM,X	@@@@
;#####################################################################
		PUSH	wordadd1
		PUSH	SUBADR
		MOV	A,RTC_BUF+2		;LOW	ADDRE
		ADD	A,ASM_X			;x register
		MOV	SUBADR,A
		JNC	NO_HIGH_INC
		INC	RTC_BUF+1
NO_HIGH_INC:
		MOV	wordadd1,RTC_BUF+1
		mov	BYTECNT,#1
		MOV	pSMB_DATA_IN,#RTC_BUF+2
		call	rcvdata_EEPROM
		MOV	TEMP,RTC_BUF+2
		INC	DPTR
		INC	DPTR
		POP	SUBADR
		POP	wordadd1
		RET
ASM_DATA_NO_30:
		CJNE	A,#60H,ASM_DATA_NO_60
;#####################################################################
;		GET LABEL DATA					@@@@
;#####################################################################
		PUSH	wordadd1
		PUSH	SUBADR
		;mov	slvadr,#EEPROM
		MOV	wordadd1,RTC_BUF+1
		mov	SUBADR,RTC_BUF+2
		mov	BYTECNT,#1
		MOV	pSMB_DATA_IN,#RTC_BUF+2
		call	rcvdata_EEPROM
		MOV	TEMP,RTC_BUF+2
		INC	DPTR
		INC	DPTR
		POP	SUBADR
		POP	wordadd1
		RET
ASM_DATA_NO_60:

		ANL	A,#0E0H
		CJNE	A,#0C0H,ASM_DATA_NO_C0
;#####################################################################
;		Load x register from USER RAM				@@@@
;#####################################################################
		MOV	A,RTC_BUF			;~0 --~31
		ANL	A,#1FH
		ADD	A,ASM_X
		ADD	A,#ASM_USR_RAM
		MOV	R1,A
		MOV	A,@R1
		MOV	TEMP,A
		RET
ASM_DATA_NO_C0:
		CJNE	A,#020H,ASM_DATA_NO_20
;#####################################################################
;		Load x register from @				@@@@
;#####################################################################
		MOV	A,RTC_BUF			;@0 --@7
		ANL	A,#07H
		ADD	A,ASM_X
		ADD	A,#BUF_RECE
		MOV	R1,A
		MOV	A,@R1
		MOV	TEMP,A
		RET

ASM_DATA_NO_20:
		CJNE	A,#010H,ASM_DATA_NO_10
;#####################################################################
;		Load ^0,0,X				@@@@
;#####################################################################
		INC	DPTR
		INC	DPTR
		RET

ASM_DATA_NO_10:
;#####################################################################
;		Load USER RAM				@@@@
;#####################################################################

		MOV	A,RTC_BUF			;~0 --~31
		ANL	A,#1FH
		ADD	A,#ASM_USR_RAM
		MOV	R1,A
		MOV	A,@R1
		MOV	TEMP,A
		RET
;##################################################################
;
;##################################################################
GET_BR_ADDRESS:
		INC	DPTR
		MOV	wordadd1,DPH
		mov	SUBADR,DPL
		mov	BYTECNT,#3
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		INC	DPTR
		INC	DPTR
		INC	DPTR
		RET
;##########################################################################
;		100 MS DELAY PROC
;
;##########################################################################
TASK_100MS_PROC:
		MOV	R2,#00000001B
		MOV	R1,#T0_7_COUNT
		MOV	R3,#09H
TASK_100MS_PROC_1:

		MOV	A,@R1
		INC	R1
		ORL	A,@R1
		JZ	TASK_100MS_PROC_NEXT
		MOV	A,@R1
		JZ	TASK_100MS_PROC_HIGHT
		DEC	A
		MOV	@R1,A
		JNZ	TASK_100MS_PROC_NEXT
		DEC	R1
		MOV	A,@R1
		INC	R1
		JNZ	TASK_100MS_PROC_NEXT

;###############################################################################
;		DELAY TIME IS END
;###############################################################################
		CJNE	R3,#01H,TASK_1_8_FLAG
;##############################################################################
;		START UP TASK FLAG
;##############################################################################
		SETB	AUTORUN_TASK
		JMP	TASK_100MS_PROC_END
TASK_1_8_FLAG:
		MOV	A,R2
		ORL	PR_LEVEL,A
		JMP	TASK_100MS_PROC_NEXT
TASK_100MS_PROC_HIGHT:
		DEC	R1
		DEC	@R1
		INC	R1
		MOV	@R1,#0FFH
TASK_100MS_PROC_NEXT:
		INC	R1
		MOV	A,R2
		RL	A
		MOV	R2,A
		DJNZ	R3,TASK_100MS_PROC_1
TASK_100MS_PROC_END:
		RET

;############################################################################
;
;
;############################################################################
BASE_ACC_START_STAK:
		ANL	A,#07H			;TASK 1~8
		INC	A
		MOV	R3,A			;R1 = TASK NUMBER 1=1TASK,8=8TASK
		MOV	R1,A
		MOV	A,#10000000B
FIND_task:
		RL	A
		DJNZ	R1,FIND_task
		ORL	PR_LEVEL,A		;TASK BIT =1
;###################################################################################
;		BASE TASK NUMBER R3 GET TASK ADDRE
;###################################################################################
		MOV	A,R3
		CALL	READ_TASK_POINT_T0_7_VECTOR
		RET    
    
MAIN_PRO:
		anl   	PCA0MD, #NOT(040h)      ; clear Watchdog Enable bit
		CALL	Init_Device
		
                mov	r0,#0FEh			;所有内存数据清零
		CLR	A
be1:
		mov	@r0,A
		MOVX	@R0,A
		djnz	r0,be1
                
                
                MOV	SECOND,#50
                SETB    TR1
		SETB	TR0
		SETB	EA
		mov	time20ms,#10
		mov	sp,#STACK_POINTER
		clr     SMB_BUSY
		SETB	OE0
		;CLR	OE0
                SETB    RI0
                SETB	OE1
                ;CLR	OE1
                
                mov	slvadr,#000h
		mov	subadr,#config_state
		mov	pSMB_DATA_IN,#boxnumber
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		mov	a,boxnumber
		jnb	acc.2,save_slider
		setb	state_0c
save_slider:
		mov	slvadr,#000h
		mov	subadr,#01h
		mov	pSMB_DATA_IN,#boxnumber
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		
		mov	slvadr,#HIGH(DEVICE_SUB_BOX_EE)
		mov	subadr,#LOW(DEVICE_SUB_BOX_EE)
		mov	pSMB_DATA_IN,#RTC_BUF
		mov	bytecnt,#04
		call	rcvdata_EEPROM
		mov	dptr,#set_temperature
		mov	a,RTC_BUF+3
		movx	@dptr,a				;save set_temperature
		MOV	dptr,#minute
		mov	a,#30				;30 s
		movx	@dptr,a
		MOV	DEVICE_SUB_BOX,RTC_BUF
		
		
		lcall	checkarea
		mov	bytecnt,#04			;初始化keyarea 4字节
		mov	slvadr,#00h
		mov	a,#keyareaPRESEEE
		mov	subadr,a
		mov	pSMB_DATA_IN,#keyarea
		call	rcvdata_EEPROM
		mov	a,keyjoinnew
		jnz	keyjoinnew_a
		mov	keyjoinnew,keyjoin
keyjoinnew_a:
		;setb	ec1
		mov	a,sbuf0
		clr	sreceive
		mov	time20ms,#10
		mov	ACC,#0ffh
		mov	P3mirror,a
		mov	P4mirror,a
		mov	P3buffer,a
		mov	P4buffer,a
		
		mov	dptr,#resetcode
		lcall	trcode
;######################################################################
;		RESET CODE NEED SEND MASTER SLAVE
;######################################################################	
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1	
		mov	subadr,#02h
		mov	pSMB_DATA_IN,#RTC_BUF
		CALL	READTWOBYTE				;get ver
		MOV	A,RTC_BUF
		CJNE	A,BUF_TRAN+4,WRITE_VER_TO_EEROM
		MOV	A,RTC_BUF+1
		CJNE	A,BUF_TRAN+5,WRITE_VER_TO_EEROM
		JMP	WRITE_VER_TO_EEROM_OK
;##############################################################
;		WRITE VER TO EEROM
;##############################################################
WRITE_VER_TO_EEROM:
		CLR	A
		mov	wordadd1,a
		mov	BYTECNT,#2
		mov	pSMB_DATA_OUT,#BUF_TRAN+4
		mov	SUBADR,#02
		lcall	SENDdata_EEPROM
		call	waitwriteend
WRITE_VER_TO_EEROM_OK:
		clr	panelsetup
		MOV	TASK_100MS_DELAY,#05H
		;MOV	LED1,#0FEH

		CALL	READ_EEPROM_40H

		CALL	READ_EEPROM_PRESEPTR

		CALL	READ_EEPROM_RELASEPTR

		MOV	DPTR,#MAN_OUT_DELAY
		CLR	A
		MOVX	@DPTR,A
		
		clr	tranready
		lcall	transbuf
		jb	tranbuffull,$
		jb	DISABLE_tran,$
		
;##########################################################################
;		总关参数初始设定
;##########################################################################		
		CLR	ALL_SCENE_ON_FLAG
		MOV	ALL_ON_CHANNEL_CONTER,#00H
		mov	wordadd1,#02H
		mov	subadr,#20h
		mov	pSMB_DATA_IN,#ALL_ON_AREA
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		

		
;###########################################################################
;		CHECK AUTORUN TASK
;
;###########################################################################
		mov	subadr,#STARTUP1_TASK_POINT	;1AH
		mov	pSMB_DATA_IN,#RTC_BUF
		call	READTWOBYTE
		mov	wordadd1,RTC_BUF
		mov	BYTECNT,#2
		mov	SUBADR,RTC_BUF+1
		mov	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		MOV	A,RTC_BUF
		JZ	REPET				;NO AUTORUN TASK
		CJNE    A,#0FFH,GET_AUTORUN_1
                JMP     REPET
GET_AUTORUN_1:
                MOV	R0,#T0_7_VECTOR+2*8
		MOV	@R0,wordadd1
		INC	R0
		MOV	@R0,SUBADR
		SETB	AUTORUN_TASK
		
REPET:		
		JNB	panelsetup,REPET_ADD_A
                JMP     REPET_ADD
REPET_ADD_A:
		MOV	channelpoint,#80H
		MOV	R3,#08H
		;INC	AUXR1			;CHANGE DPTR
PROC_TASK_1_8:
		MOV	A,PR_LEVEL
		ANL	A,channelpoint
		JZ	PROC_TASK_1_8_NEXT
;############################################################
;		FIND NEED PROC TASK
;		NUMBER  =  CLOCK_4D
;############################################################
NEXT_STEP_PROC:
		MOV	A,R3
		RL	A			;A=A*2
		MOV	R1,A
		ADD	A,R3
		ADD	A,#ASM_TASK_ACC-3
		MOV	R0,A
		MOVX	A,@R0
		MOV	ASM_ACC,A
		INC	R0
		MOVX	A,@R0
		MOV	ASM_X,A
		INC	R0
		MOVX	A,@R0
		MOV	ASM_PSW,A
		MOV	A,R1
		ADD	A,#T0_7_COUNT-2
		MOV	R4,A			;R4 SAVE T0_7_COUNT
		MOV	A,#T0_7_VECTOR-2
		ADD	A,R1
		MOV	R1,A
		MOV	R2,A			;R2 SAVE T0_7_VECTOR
		SETB	TASK_ENABLE
		MOV	DPH,@R1
		INC	R1
		MOV	DPL,@R1
		CALL	TASK_PROC
		MOV	A,R2
		MOV	R1,A
		MOV	@R1,DPH
		INC	R1
		MOV	@R1,DPL
		MOV	A,R3
		RL	A			;A=A*2
		ADD	A,R3
		ADD	A,#ASM_TASK_ACC-3
		MOV	R0,A
		MOV	A,ASM_ACC
		MOVX	@R0,A
		INC	R0
		MOV	A,ASM_X
		MOVX	@R0,A
		INC	R0
		MOV	A,ASM_PSW
		MOVX	@R0,A
		JBC     TASK_WAIT_TR,PROC_TASK_1_8_NEXT
		JB	TASK_ENABLE,NEXT_STEP_PROC
		;JB	TASK_ENABLE,PROC_TASK_1_8_NEXT
		MOV	A,channelpoint
		CPL	A
		ANL	PR_LEVEL,A		;STOP TASK PROC
PROC_TASK_1_8_NEXT:
		MOV	A,channelpoint
		RR	A
		MOV	channelpoint,A
		DJNZ	R3,PROC_TASK_1_8
;#################################################################################
;		PROC	AUTORUN TASK
;
;#################################################################################
		JNB	AUTORUN_TASK,TASK_1_8_AUTO_END
AUTORUN_NEXT_STEP_PROC:
		MOV	R0,#ASM_TASK_ACC+3*8
		MOVX	A,@R0
		MOV	ASM_ACC,A
		INC	R0
		MOVX	A,@R0
		MOV	ASM_X,A
		INC	R0
		MOVX	A,@R0
		MOV	ASM_PSW,A
		MOV	R4,#T0_7_COUNT+2*8
		MOV	A,#T0_7_VECTOR+2*8
		MOV	R1,A
		MOV	R2,A			;R2 SAVE T0_7_VECTOR
		SETB	TASK_ENABLE
		MOV	DPH,@R1
		INC	R1
		MOV	DPL,@R1
		CALL	TASK_PROC
		MOV	A,R2
		MOV	R1,A
		MOV	@R1,DPH
		INC	R1
		MOV	@R1,DPL
		MOV	R0,#ASM_TASK_ACC+3*8
		MOV	A,ASM_ACC
		MOVX	@R0,A
		INC	R0
		MOV	A,ASM_X
		MOVX	@R0,A
		INC	R0
		MOV	A,ASM_PSW
		MOVX	@R0,A
		JBC     TASK_WAIT_TR,TASK_1_8_AUTO_END
		JB	TASK_ENABLE,AUTORUN_NEXT_STEP_PROC
		;JB	TASK_ENABLE,TASK_1_8_AUTO_END
		CLR	AUTORUN_TASK
TASK_1_8_AUTO_END:
		;INC	AUXR1			;CHANGE DPTR
REPET_ADD:
		jnb	sreceiveend,man2
		clr	ren0
		lcall	receivedata
		setb	ren0
man2:
		jnb	sreceiveend_1,man2_ADD
		CLR	sreceiveend_1
		MOV	A,SCON1
		CLR	ACC.4			;ren1
		MOV	SCON1,A
		lcall	receivedata_1
		MOV	A,SCON1
		SETB	ACC.4
		MOV	SCON1,A			;ren1
		;setb	ren
man2_ADD:		
;##################################################################
;		get true_temperature from sub device 
;##################################################################
		jb	panelsetup,man4_ADD
		mov	a,true_temperature
		jnz	man4_ADD
		jb	tranready_1,fast_send_data
		jb	tranbuffull_1,man4_ADD
		jb	DISABLE_tran_1,man4_ADD
		;mov	BUF_TRAN1,#0fah
		;mov	BUF_TRAN1+1,#DEVICE_CODE_SUB
		;mov	BUF_TRAN1+2,DEVICE_SUB_BOX			;dr602 temperature box no
		;mov	BUF_TRAN1+3,#09Ch
		;mov	BUF_TRAN1+4,#00h
		;mov	BUF_TRAN1+5,#00h
		;mov	BUF_TRAN1+6,#00h
		;setb	tranbuffull_1
		;call	transbuf_1
		;jb	tranbuffull_1,$
		;jb	DISABLE_tran_1,$
		;mov	time20ms,#100
		;clr	flag20ms
		;jnb	flag20ms,$
		;JNB	sreceiveend_1,man4_ADD				;一定有反馈!
		;CLR	sreceiveend_1
		;MOV	A,SCON1
		;CLR	ACC.4
		;MOV	SCON1,A
		;lcall	receivedata_1
		;MOV	A,SCON1
		;SETB	ACC.4
		;MOV	SCON1,A
man4_ADD:		
		jnb	ROMM_IN_MAN,man4_ADD_2
		setb	HYEN
		jmp	man4_ADD_1	
man4_ADD_2:		
		clr	HYEN
man4_ADD_1:		
		JNB	TIME_2MS,man2_A
		CLR	TIME_2MS
		
		MOV	A,RELAY_OUT
                ;CPL     A
		CALL	RELAY_OUTPUT
		
		
		
fast_send_data:
		jnb	tranready_1,man3_ADD
		setb	tranbuffull_1
		clr	tranready_1
		lcall	transbuf_1
man3_ADD:		
		JB	panelsetup,man2_A
		CALL	KEY_INPUT_PROC
man2_A:
		JB	flag20ms,flag20ms_PROC		;20ms CHECK
man2_B:
                JMP     repet
flag20ms_PROC:		
		
		CLR	flag20ms
		
		CALL	PROC_RELAY_OUTPUT
		
		
		;clr	EA
		;mov	wfeed1,#0A5h
		;mov	wfeed2,#05Ah

		;clr	sda
		;call	DELAY12NOOP
		;setb	sda
		;SETB	EA
		jnb	tranready,man3
		jb	DISABLE_tran,man3
		clr	tranready
		lcall	transbuf
man3:
		
		JNB	panelsetup,man3_b
		djnz	TASK_100MS_DELAY,man2_B
                jmp     0000h

man3_b:
		DJNZ	TASK_100MS_DELAY,tran_20ms_A
;#########################################################################
;		100 MS OVER
;#########################################################################

		CALL	TASK_100MS_PROC
		MOV	TASK_100MS_DELAY,#05H



tran_20ms_A:
;##################################################################
;		16个键的keyenable_buf检查,1)有数据发送?2)有计时需求
;		?3)是onetouch键?4)到达signal时间?
;##################################################################
                jb      Sreceiveend,check_key_in
		mov	r7,#keynumber_all			;20个键计数器
		mov	r0,#keyenable_buf+keynumber_all-1	;尾指针
key_buf_check_a:
		jb	tranbuffull,key_buf_check_B
		call	key_buf_tran
key_buf_check_B:
		mov	a,@r0
		jnb	acc.5,key_buf_check_c	;不需要计时跳转
		mov	a,r0
		add	a,#keynumber_all	;得到计时地址
		mov	r1,a
		inc	@r1			;计时+1 =>计时
		cjne	@r1,#onetouch_TIME,key_buf_check_D
					;不到onetouch_TIME跳转
		mov	a,@r0
		jnb	acc.4,key_buf_check_D	;不是onetouch键跳转
		call	onetouch_long_prese
		sjmp	key_buf_check_e
key_buf_check_D:
		cjne	@r1,#SIGNON_TIME,key_buf_check_c
		setb	signal			;置位注册信号标志

key_buf_check_e:
		mov	a,@r0
		clr	acc.5			;清计时标志
		mov	@r0,a

key_buf_check_c:
		dec	r0
		djnz	r7,key_buf_check_a
		jb	tranbuffull,check_key_in
		jnb	signal,check_key_in
;##############################################################
;		发注册信号?
;##############################################################
		clr	signal
		mov	bytecnt,#01			;读入注册信号字节
		mov	slvadr,#00h
		mov	subadr,#config_state
		mov	pSMB_DATA_IN,#temp
		call	rcvdata_EEPROM
		mov	a,temp
		jnb	acc.0,check_key_in
		mov	dptr,#enablecode
		lcall	trcode

check_key_in:


ISP_COUNTER_A:
		
		
		
		DJNZ	SECOND,proc20ledb_ADD_1
		JMP	proc20ledb_ADD_2
 proc20ledb_ADD_1:
		 JMP	proc20ledb
 proc20ledb_ADD_2:
;#############################################################################		
		mov	bytecnt,#01			;喂看门狗
		mov	slvadr,#00h
		mov	subadr,#config_state
		mov	pSMB_DATA_IN,#temp
		call	rcvdata_EEPROM
;#############################################################################
;		检查是否需要拿排后延时
;#############################################################################		
		MOV	DPTR,#MAN_OUT_DELAY
		MOVX	A,@DPTR
		JZ	NO_SUB_MAN_OUT_DELAY
		DEC	A
		MOVX	@DPTR,A
		JNZ	NO_SUB_MAN_OUT_DELAY
		CLR	ROMM_IN_MAN
		;SETB	HYEN			;MAIN RELAY OFF
;#############################################################################
;		SEND AIR STOP
;#############################################################################		
		CALL	SEND_MAN_OUT_AND_DELAY_IS_END
NO_SUB_MAN_OUT_DELAY:		
		
		
		JNB	ROMM_IN_MAN,ROOM_AIR_CALL_END		
		;插牌开关有动作,有人进入.

		MOV	dptr,#minute
		MOVX	A,@dptr
		DEC	A
		JNZ	CONTINU_30S
;############################30 秒到？，处理空调温度################################
		;CALL	AIR_PROC	
		MOV	A,#30
CONTINU_30S:		
		MOV	dptr,#minute
                MOVX	@dptr,A		
		
;##############################################################################
;		proc	AUX		
;		
;
ROOM_AIR_CALL_END:
		MOV	SECOND,#50
		SETB	service_led		;检测service sw (P1.7) 
		MOV	A,#40H
		DJNZ 	ACC,$
		mov	A,p2
		MOV	R0,A
		SETB	service_led
		JB	LED_SW,OUT_P4
		CLR	service_led
OUT_P4:		
		MOV	A,R0
		jb	acc.3,SERVICE_OFF
		jb	servicesw,proc20msrst1
		setb	servicesw
		mov	dptr,#enablecode
		lcall	trcode
		mov	buf_tran+2,boxnumber
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		ajmp	proc20msrst1
SERVICE_OFF:
		clr	servicesw
		
proc20msrst1:	
		
		jnb	serviceled,procsecond1a	;
		djnz	netactivetime,procsecond1b
		clr	serviceled		;
					
procsecond1a:		
						;normal led flash
		JBC	LED_SW,SERVICE_LED_ON
		SETB	service_led
		SETB	LED_SW
		JMP	procsecond1b
SERVICE_LED_ON:		
		CLR	service_led
procsecond1b:		
		
;###############################################################
;		秒计时先判=1?,是:清0,置位REN,清P12
;		否:置位该位.
;###############################################################
		
		JBC	TRFLAG,TRFLAG_CLEAR
		SETB	TRFLAG
		SJMP	proc20ledb
				
TRFLAG_CLEAR:	
		SETB	OE0
		SETB	OE1
		;CLR	OE0
		;CLR	OE1
		CLR	tranbuffull
		CLR	tranbuffull_1
		SETB	REN0
		SETB	EA	
		
		
		
		

proc20ledb:		
		jnb	serviceled,proc20ledc	; network not active
		mov	a,second
		cjne	a,#22h,proc20ledd
proc20ledd:	jc	proc20lede		
		 				;led off
		SETB	service_led
		SETB	LED_SW
		ajmp	proc20ledc
proc20lede:
		 				;led on
		CLR	service_led
		CLR	LED_SW
		
proc20ledc:		
		
		
		
		
ISP_COUNTER_B:		
		JMP	repet
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
;###############################################################################
;
;###############################################################################
proc_hotel_key:
		jnb	ROMM_IN_MAN,proc_hotel_key_013
		cjne	a,#010,proc_hotel_key_011
		call	proc_CLEAR_SW			;010 CLEAR key
		ret

proc_hotel_key_011:		
		cjne	a,#011,proc_hotel_key_012
		call	proc_NO_BREAK_IN_ON		;011 请勿打扰 key
		ret

proc_hotel_key_012:
		cjne	a,#012,proc_hotel_key_013
		setb	MASTER_OFF_key_presee
		call	proc_master_off			;012 master key
		ret
	
proc_hotel_key_013:
		cjne	a,#013,proc_hotel_key_014
		JB	keyprese,hotel_key_013_main_in
;#################################################################################
;		main_OUT
;#################################################################################
		mov	BUF_TRAN1,#0fah		;fa 69 ff 8d 01 ff ff 11  Guest out
		mov	BUF_TRAN1+1,#69h	;插牌读卡器
		mov	BUF_TRAN1+2,#0ffh	;插牌读卡器 box no
		mov	BUF_TRAN1+3,#08dh
		mov	BUF_TRAN1+4,#01h
		mov	BUF_TRAN1+5,#0ffh
		mov	BUF_TRAN1+6,#0ffh
		jmp	hotel_key_013_main_in_1		
hotel_key_013_main_in:		
		mov	BUF_TRAN1,#0fah		;fa 69 ff 8e 01 ff ff 11  Guest in
		mov	BUF_TRAN1+1,#69h	;插牌读卡器
		mov	BUF_TRAN1+2,#0ffh	;插牌读卡器 box no
		mov	BUF_TRAN1+3,#08eh
		mov	BUF_TRAN1+4,#01h
		mov	BUF_TRAN1+5,#0ffh
		mov	BUF_TRAN1+6,#0ffh
hotel_key_013_main_in_1:		
		setb	tranbuffull_1
		call	transbuf_1
		mov	r1,#BUF_RECE1
		mov	r0,#BUF_TRAN1
		mov	temp,#08
copyt_rbuf_1:	mov	a,@r0
		mov	@r1,a
		inc	r0
		inc	r1
		djnz	temp,copyt_rbuf_1
		setb	sreceiveend_1
		jb	tranbuffull_1,$
		
		
		
		;call	proc_open_first_sw		;插排开关
		ret

proc_hotel_key_014:
		jnb	ROMM_IN_MAN,proc_hotel_key_end
		cjne	a,#014,proc_hotel_key_015
		call	proc_door_key_sw		;门口按钮
		
		
proc_hotel_key_015:
		cjne	a,#015,proc_hotel_key_end
		call	proc_sos_sw			;sos按钮		
		
proc_hotel_key_end:		
		ret		
		
		
KEY_INPUT_PROC:
		;JNB	ROMM_IN_MAN,KEY_INPUT_PROC_END
		call	keyin
		jbc	keyaction,keyaction_PROC
KEY_INPUT_PROC_END:
		RET
keyaction_PROC:
		MOV	A,keynumber
		cjne	a,#10,check_key_no
check_key_no:		
		jc	check_key_no_less_10
		jmp	proc_hotel_key
check_key_no_less_10:		
		jnb	ROMM_IN_MAN,KEY_INPUT_PROC_END
		ljmp	check_key		;判该键是否disablie
check_key_ok:					;key is active
;#####################################################################
;		PROC	KEY 403H
;#####################################################################
		MOV	A,keynumber
		ADD	A,#40H
		mov	SUBADR,A
		CALL	OUT_403H
		
		
		
;######################################################
;		从50H读入4个字节到keyarea中
;
;######################################################
		;mov	bytecnt,#04
		;mov	slvadr,#000h
		;mov	a,keynumber
		;mov	b,#04h
		;mul	ab
		;add	a,#keyareaPRESEEE
		;mov	subadr,a
		;mov	r1,#keyarea
		;call	rcvdata_EEPROM
		CALL	CHECK_EEPROM_40H
		MOV	DPTR,#LED_40H_XRAM
		mov	a,keynumber
		mov	b,#04h
		mul	ab
		ADD	A,DPL
		MOV	DPL,A
		MOV	B,#04H
		MOV	R1,#keyarea
READ_LED_40H_4BYTE:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	B,READ_LED_40H_4BYTE
		mov	a,keyjoinnew
		CJNE	A,#0FFH,man4a_B
		JMP	man4a_C
man4a_B:
		jnz	man4a_a
man4a_C:
		mov	keyjoinnew,keyjoin	;ver 700 keyjoinnew =0,
						;VER 762 keyjoinnew =FF.
man4a_a:
		mov	a,keylednumber
		cjne	a,#80h,man4a		; >=80h=onetouch key
man4a:		jc	man5
		CJNE	A,#0FFH,man4a_ADD
		JMP     MAN5_TASK_PROC
                ;JMP	man5
man4a_ADD:
		cjne	a,#0C0H,CHECK_SETCHANNEL_A
CHECK_SETCHANNEL_A:
		JNC	man5
		JB	keyprese,onetouch_keyprese
		LJMP	onetouch_keyrelase
onetouch_keyprese:
;########################################################
;		;置标志 onetouch key
;########################################################
		;CLR	F0
		mov	dptr,#preseptr-7
		mov	a,#10h
		mov	b,keynumber
		mul	ab
		mov	r0,a
		mov	a,dpl
		add	a,r0
		mov	dpl,a
		jnc	onetouch_keyprese_add_2
		inc	dph
onetouch_keyprese_add_2:
		mov	subadr,a
		mov	a,dph
		ADD	A,B
		mov	slvadr,a
		mov	bytecnt,#04h
		mov	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM			;获得 KEY 的状态
		mov	a,RTC_BUF
		clr	F0
		JNB	ACC.4,onetouch_keyprese_add	;ACC.4=1 RAMPING
		SETB	F0
onetouch_keyprese_add:
		MOV	A,#onetouch_BUF
		ADD	A,KEYNUMBER
		MOV	R0,A
		MOV	A,#00110000B
		ORL	A,@R0
		MOV	@R0,A			;BIT 4 =1,SET onetouch FLAG,SETB RAMPING BIT
		JB	F0,onetouch_keyprese_add_1
		ANL	A,#11011111B		;TIME = 0 DISABLE COUNTER
		MOV	@R0,A
onetouch_keyprese_add_1:
		MOV	A,R0
		ADD	A,#keynumber_all
		MOV	R0,A
		MOV	@R0,#00H		;CLEAR TIME
JUMP_REPET:

		jmp	KEY_INPUT_PROC_END
man5:
		jb	state_0c,flower_state
MAN5_TASK_PROC:
		mov	dptr,#preseptr
		jb	keyprese,key4
		mov	dptr,#relaseptr
		jmp	key4
flower_state:
		mov	a,#onetouch_BUF
		add	a,keynumber
		mov	r0,a
		MOV	ACC,@R0
		jbc	acc.0,newkeyrelase
;####################################################################
;		该键的D0位为0,0->1,keyprese
;####################################################################
		clr	keyrelase
		setb	keyprese
		inc	ACC
		mov	@r0,a
		mov	dptr,#preseptr
		sjmp	key4
;#########################################################################
;		keyrelase PROC
;		NO onetouch KEY
;		该键的D0位为1,1->0,KEYRELASE
;#########################################################################

newkeyrelase:
		setb	keyrelase
		clr	keyprese
		mov	@r0,a
		;JNB	ACC.1,JUMP_REPET
		;KEYRELASE ANABLE
		mov	dptr,#relaseptr		;relase

key4:
;#################################################################
;		正常键处理，检查是键的状态
;#################################################################
		call COPY_EEROM_TO_key_tran_buf
;######################################################################
;		检查DPTR - 9 =05? 是的为TASK 取+4的数值为TASK号-1
;######################################################################
		clr	c
		mov	a,dpl
		subb	a,#09
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#TEMP
		call	rcvdata_EEPROM			;获得 KEY 的状态=05?
		MOV	A,TEMP
		CJNE	A,#05H,key4_ADD
		MOV	A,subadr
		ADD	A,#04H
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#TEMP
		call	rcvdata_EEPROM			;获得 TASK NO.
;######################################################################
;		START TASK
;######################################################################
		MOV	A,TEMP
		CALL	BASE_ACC_START_STAK
		JMP	KEY_INPUT_PROC_END
key4_ADD:
		INC	subadr
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#TEMP
		call	rcvdata_EEPROM			;获得 KEY 的状态
		mov	a,#onetouch_BUF
		add	a,keynumber
		mov	r0,a
		MOV	A,TEMP
;####################################################################
;		按键是UP,DOWN时,不发注册信号
;
;####################################################################
		cjne	a,#099h,no_UP_DOWN
		mov	a,@r0
		clr	acc.5			;停止计时
		mov	@r0,a
no_UP_DOWN:
		mov	a,temp
		JB	ACC.7,noddSend
		;keyprese OR keyrelase DDSEND
NEED_DDSEND:
		;mov	a,#onetouch_BUF
		;add	a,keynumber
		;mov	r0,a
		mov	a,#00001000b		;DDSED状态
		orl	a,@r0
		mov	@r0,a
noddSend:
		JB	keyrelase,NO_keyrelase
		mov	a,temp
		JB	ACC.5,NO_keyrelase
		;mov	a,#onetouch_BUF
		;add	a,keynumber
		;mov	r0,a
		mov	a,#00000010b		;KEYRELASE ANABLE
		orl	a,@r0
		mov	@r0,a
NO_keyrelase:
		call	set_tran_flag

key10:


;#############################################################################
;		ic area channel	76 fidetime 00 join	原来
;		ic area 0xff	7a 64 01 join		变换后
;
;
;
;#############################################################################
		;JNB	progout,key7
		mov	a,r3
		add	a,#03
		mov	r0,a
		MOV	A,@r0			;GET COMMAND
		CJNE	A,#76H,key7
		MOV	@r0,#7AH		;PROC 76 COMMAND
		dec	r0
		MOV	@r0,#0FFH
		inc	r0
		inc	r0
		MOV	@r0,#64H
		inc	r0
 		MOV	@r0,#0

key7:
		JMP	KEY_INPUT_PROC_END
DELAY12NOOP:
		MOV	DELE12NOP,#20H
		DJNZ	DELE12NOP,$
		RET

Delay10msa:

		MOV	R7,#080H	;00fh
D10ms:		MOV	R6,#00H
D40ms:		DJNZ	R6,D40ms
		DJNZ	R7,D10ms


		;clr	ea
		;mov	wfeed1,#0A5h
		;mov	wfeed2,#05Ah
		;setb	ea
		;clr	sda
		;call	DELAY12NOOP

		;setb	sda
		ret


KEYIN:
		MOV	ACC,P4
		MOV	R0,A		;SEVE P3
		XRL	A,P4buffer	;从内存读
		JNZ	P4KEYIN
		MOV	A,R0		;key已稳定
		MOV	R1,P4mirror
		XRL	A,R1
		JZ	P3KEYINCHK	;

;###############################################################
;		P4 NEW KEY ACTIVE
;		ACC <> 0 的位，表示该键有变化
;###############################################################
P4_NEW_KEY_ACTIVE:
		MOV	R3,#08H		;keynumber COUNTER
		CLR     F0
		call	FIND_NEW_KEY
		RET
;#################################################################
;		P4 <> P3buffer
;################################################################
P4KEYIN:
		MOV	P4buffer,R0	;NEW P0 SEVE TO P0buffer


P3KEYINCHK:
		MOV	ACC,P3
		MOV	R0,A		;SEVE P2
		XRL	A,P3buffer
		JNZ	P3KEYIN
		MOV	A,R0
		MOV	R1,P3mirror
		XRL	A,R1
		JZ	P3_P4_END
P3_NEW_KEY_ACTIVE:
		MOV	R3,#00H		;keynumber COUNTER
		SETB    F0
		call	FIND_NEW_KEY

P3_P4_END:
		RET			;NO NEW KEYPRESS
;#################################################################
;		P3 <> P4buffer
;################################################################
P3KEYIN:
		MOV	P3buffer,R0	;NEW P2 SEVE TO P2buffer

		RET

FIND_NEW_KEY:
		MOV	R0,A		;SAVE ACC
		MOV	R4,#01H		;KEYPOINT
		SETB	keyaction
		MOV	R2,#08H		;KEY COUNTER
		CLR	C
P0_ACTIVE_LOOP:
		MOV	ACC,R0
		RRC	A
		MOV	R0,A
		JC	P0_ACTIVE_FIND
		MOV	A,R4
		RL	A
		MOV	R4,A
		INC	R3		;keynumber COUNTER+1
		DJNZ	R2,P0_ACTIVE_LOOP

P0_ACTIVE_FIND:
		MOV	ACC,R1		;r1=P3mirror,p4mirror
		ANL	A,R4
		JNZ	P0_ACTIVE_KEY_PRESE	;1->0
		MOV	A,R4
		ORL	A,R1
		JB      F0,SAVE_P4MIRROR
		MOV	P4mirror,A
		SJMP    ACTIVE_keyprese
SAVE_P4MIRROR:
		MOV	P3mirror,A
ACTIVE_keyprese:
		CLR	keyprese
		SETB	keyrelase
		SJMP	KEY_BUFF_CHANGE
P0_ACTIVE_KEY_PRESE:
		CPL     A
		ANL	A,R1
		JB      F0,SAVE_P4MIRROR_B
		MOV	P4mirror,A
		SJMP    ACTIVE_keyRELAse

SAVE_P4MIRROR_B:
                MOV	P3mirror,A

ACTIVE_keyRELAse:
		SETB	keyprese
		CLR	keyrelase
;#######################################################################
;		入口：R3=KEYNUMBER
;		USE：R0，ACC
;#######################################################################
KEY_BUFF_CHANGE:
		MOV	A,#onetouch_BUF
		ADD	A,R3
		MOV	R0,A		;RO 为onetouch_BUF key point
		mov	a,@r0
		clr	acc.5
		jb	keyrelase,KEY_BUFF_CHANGE_save;relase clr

		setb	acc.5		;置位 PRESE
KEY_BUFF_CHANGE_save:
		MOV	@R0,A
		jb	keyrelase,keytimebuf_a	;不清计数器
		MOV	A,#keynumber_all
		ADD	A,R0		;得到keytimebuf位置
		MOV     R0,ACC
		MOV	@R0,#0		;清计数器

keytimebuf_a:
		MOV	keynumber,R3
		MOV	A,R3
                INC     A
		JB	keyprese,keytimebuf_B
		SETB	ACC.7
keytimebuf_B:
		MOV	ASM_KEY_MAP,A
		RET









;###########################################################
;		output  LED
;
;
;###########################################################
DISPLAY:

		RET


Delay1ms:	MOV	R7,#20H
D3:		MOV	R6,#0eFH
D4:		DJNZ	R6,D4
		DJNZ	R7,D3
		ret

		
receivedata6:	RET
receivedata6D:	CJNE	@R1,#0FCh,receivedata6
		LCALL	blockwritePROC
		RET
receivedata0DF:	cjne	a,#0DFh,receivedata0DE
;#######################################################################
;		CODE   	30H  block read  PROC
;
;######################################################################
		call	copyr_tbuf
		mov	buf_tran+3,#0DEh		;block read acknowledge
receivedata30a:
		MOV	R7,#015H	;15h
		call	D10ms
		setb	tranready
		ret
receivedata0DE:	cjne	a,#0DEh,receivedata0DD	;block WRITER
		LCALL	blockreadPROC
		ret
receivedata0DD:	CJNE	A,#0DDH,receivedata0DC
;############################################################
;		CODE   	32H  block write  PROC
;
;###########################################################
		jnb	panelsetup,receivedata0FA_END
		CALL	copyr_tbuf
		mov	buf_tran+3,#0DCh		;block WRITE acknowledge
		setb	tranready
		mov	a,buf_tran+5
		mov	subadr,a
		mov	a,buf_tran+4
		mov	wordadd1,a
		setb	blockwrite

		sjmp	receivedata30a
receivedata0DC:	CJNE	A,#0DCH,receivedata0E2
;###########################################################
;		CODE   	33H  block write  end   PROC
;
;###########################################################
		clr	blockwrite
receivedata0FA_END:
		ret
receivedata10:			;<>12,3,4,40,80,30,31,32,33
		cjne	a,#06,receivedata0E2
		ljmp	requestRAM

receivedata0E2:
		CJNE	A,#0F1H,receive_5c_11
;####################################################################################
;		START	TASK	BYTE4 01 = TASK1,02=TASK2,...
;####################################################################################
receive_5c_10_13:
		MOV	A,BUF_RECE+4		;GET TASK NUMBER
		DEC	A
		CALL	BASE_ACC_START_STAK
		RET
receive_5c_11:
		CJNE	A,#0F0H,receive_5c_12
;####################################################################################
;		STOP	TASK	BYTE4 00 = ALL,01 = TASK1,02=TASK2
;####################################################################################
		MOV	A,BUF_RECE+4		;GET TASK NUMBER
		JZ	STOP_ALL_TASK
		DEC	A
		ANL	A,#07H
		INC	A			;TASK 1~8
		MOV	R3,A			;R1 = TASK NUMBER 1=1TASK,8=8TASK
		MOV	R1,A
		MOV	A,#10000000B
FIND_task_2:
		RL	A
		DJNZ	R1,FIND_task_2
		CPL	A
		ANL	PR_LEVEL,A		;TASK BIT =1
		MOV	A,R3
		CALL	T0_7_COUNT_CLEAR
		RET
STOP_ALL_TASK:
		MOV	PR_LEVEL,#00H
		MOV	R3,#16
		CLR	A
		MOV	R1,#T0_7_COUNT
STOP_ALL_TASK_1:
		MOV	@R1,A
		INC	R1
		DJNZ	R3,STOP_ALL_TASK_1	;T0_7_COUNT CLEAR 16 BYTE
		RET
receive_5c_12:
		CJNE	A,#0EFH,receive_5c_13
;####################################################################################
;		Suspend Task(pause)	TASK	BYTE4 00 = ALL,01 = TASK1,02=TASK2
;####################################################################################
		MOV	A,BUF_RECE+4		;GET TASK NUMBER
		JZ	PAUSE_ALL_TASK
		DEC	A
		ANL	A,#07H
		INC	A			;TASK 1~8
		MOV	R3,A			;R1 = TASK NUMBER 1=1TASK,8=8TASK
		MOV	R1,A
		MOV	A,#10000000B
PAUSE_FIND_task_2:
		RL	A
		DJNZ	R1,PAUSE_FIND_task_2
		CPL	A
		ANL	PR_LEVEL,A		;TASK BIT =1
		MOV	A,R3
		CALL	T0_7_COUNT_CLEAR
		RET
PAUSE_ALL_TASK:
		MOV	PR_LEVEL,#00H
		MOV	R3,#16+2		;AUTO_RUN
		CLR	A
		MOV	R1,#T0_7_COUNT
PAUSE_ALL_TASK_1:
		MOV	@R1,A
		INC	R1
		DJNZ	R3,PAUSE_ALL_TASK_1	;T0_7_COUNT CLEAR 18 BYTE
		RET

receive_5c_13:
		CJNE	A,#0EEH,receive_FA_C8
;####################################################################################
;		Resume	TASK	BYTE4 00 = ALL,01 = TASK1,02=TASK2
;####################################################################################
		MOV	A,BUF_RECE+4		;GET TASK NUMBER
		JZ	Resume_ALL_TASK
		DEC	A
		ANL	A,#07H
		INC	A			;TASK 1~8
		MOV	R3,A			;R1 = TASK NUMBER 1=1TASK,8=8TASK
		MOV	R1,A
		MOV	A,#10000000B
Resume_FIND_task_2:
		RL	A
		DJNZ	R1,Resume_FIND_task_2
		ORL	PR_LEVEL,A		;TASK BIT =1
		RET
Resume_ALL_TASK:
		MOV	PR_LEVEL,#0FFH
		ret
receive_FA_C8:
;#######################################################################################
;		;Read Time Host requst DEVICE report
;
;
;#######################################################################################
		cjne	a,#0C8h,receive_FA_C9
		;CALL	copy_r_t_buffer
		call	copyr_tbuf		;Read Time Host requst Dtk936 report
		mov	buf_tran+3,#0C7h
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#000h
		mov	pSMB_DATA_IN,#RTC_BUF		;RTC_buf address->r1
		mov	bytecnt,#03
		call	rcvdata
		mov	a,RTC_BUF+2
		anl	a,#01111111b
		mov	buf_tran+4,a
		mov	buf_tran+5,RTC_BUF+1
		mov	buf_tran+6,RTC_BUF 	;Report Time byte5=Hours,Byte6=minutes,Byte7=seconds BCD Code
		jmp	re_wrproc3
;#######################################################################################
;		;WRITE Time TO DEVICE
;
;
;#######################################################################################

receive_FA_C9:
		cjne	a,#0C9h,receive_FA_C6
		call	copyr_tbuf
		mov	RTC_BUF,buf_tran+6
		mov	RTC_BUF+1,buf_tran+5
		mov	RTC_BUF+2,buf_tran+4
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#000h
		mov	pSMB_DATA_OUT,#RTC_BUF		;RTC_buf address->r1
		mov	bytecnt,#03
		call	SendData			;Write TIME byte5=Hour,Byte6=minutes,Byte7=seconds BCD Code
		
		ret					;20041115
		jmp	re_wrproc3
;#######################################################################################
;		;WRITE DATE TO DEVICE
;
;
;#######################################################################################

receive_FA_C6:
		cjne	a,#0C6H,receive_FA_C5
		mov	a,buf_rece+5
		anl	a,#11100000b
		swap	a
		rr	a
		jnz	receivedata12_53
		mov	a,#07h
receivedata12_53:
		mov	RTC_BUF,a		;day
		mov	RTC_BUF+1,buf_rece+4	;date
		mov	a,buf_rece+5
		anl	a,#00011111b
		mov	RTC_BUF+2,a		;month
		mov	RTC_BUF+3,buf_rece+6	;year
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#03h
		mov	pSMB_DATA_OUT,#RTC_BUF
		mov	bytecnt,#04
		call	SendData
						;Write Date byte5=Date,Byte6=month+week,Byte7=year BCD Code
		
 		ret		
;#######################################################################################
;		Read DATE Host requst DEVICE report
;
;
;#######################################################################################


receive_FA_C5:
		cjne	a,#0C5H,receive_FA_9C
		 				;Read Date Host requst Dtk934 report
		call	copyr_tbuf
		mov	buf_tran+3,#0C4H
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#03h
		mov	pSMB_DATA_IN,#RTC_BUF		;RTC_buf address->r1
		mov	bytecnt,#04		;read day ,date,month,year
		call	rcvdata
		mov	a,RTC_BUF+0
		swap	a
		rl	a
		anl	a,#11100000b
		cjne	a,#11100000b,receive_C5_1
		mov	a,#00h
receive_C5_1:
		orl	a,RTC_BUF+2
		mov	buf_tran+5,a
		mov	buf_tran+4,RTC_BUF+1
		mov	buf_tran+6,temp 	;Report Date byte5=Date,Byte6=month,Byte7=year BCD Code
		jmp	re_wrproc3
		
				
receive_FA_9C:
		cjne	a,#09CH,receive_FA_9B
		call	copyr_tbuf
		mov	buf_tran+3,#09BH
		MOV	DPTR,#set_temperature
		MOVX	A,@DPTR
		MOV	buf_tran+5,A			;SET  temperature
		INC	DPTR
		MOVX	A,@DPTR
		ANL	A,#0FH
		MOV	R1,A	  			;MODE
		INC	DPTR
		MOVX	A,@DPTR
		SWAP	A				;FAN SPEED
		anl	a,#0f0h
		ORL	A,R1
		MOV	buf_tran+4,A
		MOV	buf_tran+6,true_temperature	;Report temperature

		jmp	re_wrproc3		
receive_FA_9B:
		cjne	a,#08FH,receive_FA_NO_8F
;###############################################################################
;		OUTSIDE SEND SET MODE NEED REPLAY AND SEND TO SLAVE
;###############################################################################		
		call	copyr_tbuf
		MOV	BUF_TRAN1,BUF_TRAN
		MOV	BUF_TRAN1+1,#DEVICE_CODE_SUB
		MOV	BUF_TRAN1+2,DEVICE_SUB_BOX
		MOV	BUF_TRAN1+3,BUF_TRAN+3				
		MOV	BUF_TRAN1+4,BUF_TRAN+4
		MOV	BUF_TRAN1+5,BUF_TRAN+5
		MOV	BUF_TRAN1+6,BUF_TRAN+6
		CALL	transbuf_1
		mov	buf_tran+3,#09BH
		mov	buf_tran+6,true_temperature
		MOV	DPTR,#set_temperature
		MOV	A,BUF_TRAN+5
		MOVX	@DPTR,A
		MOV	A,BUF_TRAN+4
		ANL	A,#0FH
		INC	DPTR
		MOVX	@DPTR,A			;MODE
		INC	DPTR
		MOV	A,BUF_TRAN+4
		SWAP	A
		ANL	A,#0FH
		MOVX	@DPTR,A			;FAN SPEED
		jmp	re_wrproc3
receive_FA_NO_8F:
		cjne	a,#07BH,receive_FA_NO_7B
;##########################################################################
;		0faH,DEVICE_CODE,boxnumber,07BH,01h,03H,80H;请勿打扰开始
;##########################################################################		
		setb	do_not_disturb_lamp
		clr	need_clear_lamp
		MOV	R0,#BUF_RECE
		MOV	R1,#BUF_TRAN
		MOV	R7,#08H
receive_FA_7B_1:		
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	R7,receive_FA_7B_1
		CLR	RECEIVED_DATA_FROM_SLAVE
		CLR	MASTER_MUTE
		SETB	tranbuffull
		call	transbuf	;SEND	MARSTER
		jnb	sreceiveend,$	;从内网收到数据,发到外网后,重新要
					;处理,现清除sreceiveend标志位后,不再处理
		clr	sreceiveend
		RET
receive_FA_NO_7B:		
		cjne	a,#07AH,proc5c_end
;##########################################################################
;		0faH,DEVICE_CODE,boxnumber,07AH,01h,03H,80H;请勿打扰结束
;##########################################################################
		clr	do_not_disturb_lamp
		jnb	CLEAR_LAMP_ON,receive_FA_7A_2
		setb	need_clear_lamp
		
		
receive_FA_7A_2:		
		
		
		MOV	R0,#BUF_RECE
		MOV	R1,#BUF_TRAN
		MOV	R7,#08H
receive_FA_7A_1:		
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	R7,receive_FA_7A_1
		clr	RECEIVED_DATA_FROM_SLAVE
		CLR	MASTER_MUTE
		clr	tranbuffull
		call	transbuf			;SEND	MARSTER	
		jnb	sreceiveend,$
		clr	sreceiveend
			
		;mov	dptr,#NO_BREAK_IN_ON_END_CODE
		;lcall	trcode
		;MOV	buf_tran+2,boxnumber
;######################################################################
;		NO_BREAK_IN CODE (OFF) NEED SEND MASTER  
;######################################################################	
		;clr	tranready
		;lcall	transbuf
;####################################################################
;		发送门铃通道关,物理通道
;####################################################################
		
proc5c_end:
		ret				;<>12,3,4,40,80,30,31,32,33,10,11,15,14,16
		

PROC_DEVICE_SUB1_PROC:
		cjne	@r1,#DEVICE_CODE1_SUB,proc5c_end
;############################################################################
;		插排开关发码
;############################################################################		
		INC	R1
		INC	R1
		MOV	BUF_TRAN+3,#08DH		;MAN OUT
		CLR	keyprese
		SETB	keyrelase
		cjne	@r1,#8Eh,PROC_DEVICE_SUB1_PROC_END
		MOV	BUF_TRAN+3,#08EH		;MAN IN
		SETB	ROMM_IN_MAN
		;CLR	HYEN				;MAIN RELAY ON
;#############################################################################
;		SET	AIR CONDITION	set_temperature
;#############################################################################		
		mov	wordadd1,#SET_temperatureEE_H
		mov	subadr,#SET_temperatureEE_L
		mov	pSMB_DATA_IN,#RTC_BUF		;set_temperature
		mov	bytecnt,#1
		call	rcvdata_EEPROM
		
		MOV	DPTR,#set_temperature
		MOV	A,RTC_BUF
		MOVX	@DPTR,A
		
		MOV	A,true_temperature
		CLR	C
		SUBB	A,RTC_BUF			;
		JZ	NEED_FAN_PROC
		JC	NEED_ADD_temperature
;#############################################################################
;		温度大于设定温度,制冷
;#############################################################################
		MOV	A,#32H			;FAN=HIGH,MODE=COLD
		JMP	CHOICE_AIR_MODE_END
NEED_ADD_temperature:		
		MOV	A,#33H			;FAN=HIGH  MODE=HOT
		JMP	CHOICE_AIR_MODE_END

NEED_FAN_PROC:
		MOV	A,#21H			;FAN=MID  MODE=FAN
		
CHOICE_AIR_MODE_END:		
		MOV	RTC_BUF+1,A
		ANL	A,#0FH
		MOV	DPTR,#mode
		MOVX	@DPTR,A
		INC	DPTR
		MOV	A,RTC_BUF+1
		SWAP	A
		ANL	A,#0FH
		MOVX	@DPTR,A			;SAVE FAN SPEED
		;CALL	AIR_PROC
		SETB	keyprese
		CLR	keyrelase
PROC_DEVICE_SUB1_PROC_END:
		MOV	BUF_TRAN,buf_rece1
		MOV	BUF_TRAN+1,#DEVICE_CODE
		MOV	BUF_TRAN+2,boxnumber
		MOV	BUF_TRAN+4,buf_rece1+4
		MOV	BUF_TRAN+5,buf_rece1+5		;
		MOV	BUF_TRAN+6,buf_rece1+6
		CLR	RECEIVED_DATA_FROM_SLAVE
		CLR	MASTER_MUTE
		SETB	tranbuffull
		call	transbuf			;SEND	MARSTER  有人进入。
		
		MOV	BUF_TRAN1+2,buf_rece1+2		;BOX NO.
		MOV	BUF_TRAN1,#0FAH
		MOV	BUF_TRAN1+1,buf_rece1+1
		MOV	BUF_TRAN1+3,#08CH
		MOV	BUF_TRAN1+4,buf_rece1+4
		MOV	BUF_TRAN1+5,buf_rece1+5		;
		MOV	BUF_TRAN1+6,buf_rece1+6
		SETB	tranbuffull_1
		call	transbuf_1			;回复SLAVE 收到命令
		
		;JB	keyrelase,NO_CHECK_WHAT_MAN_IN
		CALL	CHECK_WHAT_MAN_IN
		
;NO_CHECK_WHAT_MAN_IN:		
		JB	keyrelase,first_sw_PROC
		
;##################################################################################
;		插排开关后,发空调当前状态码.	
;		tranready_1
;##################################################################################		
		;JB	tranbuffull,$
		;MOV	BUF_TRAN,#0FAH
		;MOV	BUF_TRAN+1,#DEVICE_CODE
		;MOV	BUF_TRAN+2,boxnumber
		;MOV	BUF_TRAN+3,#8FH			;SEND AIR_CONDITION
		;MOV	DPTR,#MODE
		;MOVX	A,@DPTR
		;MOV	RTC_BUF+1,A
		;INC	DPTR
		;MOVX	A,@DPTR
		;SWAP	A
		;ORL	A,RTC_BUF+1
		;MOV	RTC_BUF+1,A
		;MOV	BUF_TRAN+4,RTC_BUF+1
		;MOV	BUF_TRAN+5,RTC_BUF		;设定温度
		;MOV	BUF_TRAN+6,true_temperature
		;CLR	RECEIVED_DATA_FROM_SLAVE
		;CLR	MASTER_MUTE
		;SETB	tranbuffull
							;SEND	MARSTER  AIR PROC。
		;JB	DISABLE_tran,$
		;call	transbuf
		
		
first_sw_PROC:				
		CALL	proc_open_first_sw
				

proc5c_end_ADD:		
		RET
		
PROC_DEVICE_SUB_9BH:
		cjne	@r1,#09BH,proc5c_end_ADD
;##########################################################################
;		查寻温度返回数据
;##########################################################################
		MOV	DPTR,#set_temperature
		MOV	A,buf_rece1+5		;set_temperature
		MOVX	@DPTR,A
		MOV	true_temperature,buf_rece1+6		;true_temperature
		
		INC	DPTR			;
		MOV	A,buf_rece1+4
		anl	a,#0fh
		MOVX	@DPTR,A			;MODE
		inc	dptr
		MOV	A,buf_rece1+4
		swap	a
		anl	a,#0fh
		MOVX	@DPTR,A			;fan speed	
		
		RET
PROC_DEVICE_SUB1_PROC_ADD:
		 JMP	PROC_DEVICE_SUB1_PROC

PROC_DEVICE_SUB_PROC:
		cjne	@r1,#DEVICE_CODE_SUB,PROC_DEVICE_SUB1_PROC_ADD	;不是从设备0
		INC	R1
		MOV	A,@R1
		CJNE	A,DEVICE_SUB_BOX,proc5c_end_ADD	;BOX <>
		INC	R1
		CJNE	@R1,#08FH,PROC_DEVICE_SUB_9BH
;############################################################################
;		SLAVE SEND NEW temperature,NEED_RELPAY TO SLAVE MASTER
;############################################################################
		MOV	BUF_TRAN,buf_rece1
		MOV	BUF_TRAN+1,#DEVICE_CODE
		MOV	BUF_TRAN+2,boxnumber
		MOV	BUF_TRAN+3,#09BH
		MOV	BUF_TRAN+4,buf_rece1+4
		MOV	BUF_TRAN+5,buf_rece1+5		;
		MOV	BUF_TRAN+6,buf_rece1+6
		CLR	RECEIVED_DATA_FROM_SLAVE
		CLR	MASTER_MUTE
		call	transbuf
		
		
				
		MOV	BUF_TRAN1+2,buf_rece1+2		;BOX NO.
		MOV	BUF_TRAN1,buf_rece1
		MOV	BUF_TRAN1+1,buf_rece1+1
		MOV	BUF_TRAN1+3,#09BH
		MOV	BUF_TRAN1+4,buf_rece1+4
		MOV	BUF_TRAN1+5,buf_rece1+5		;
		MOV	BUF_TRAN1+6,buf_rece1+6
		call	transbuf_1
		MOV	DPTR,#set_temperature
		MOV	A,buf_rece1+5		;set_temperature
		MOVX	@DPTR,A
		MOV	true_temperature,buf_rece1+6		;true_temperature
		
		INC	DPTR			;
		MOV	A,buf_rece1+4
		anl	a,#0fh
		MOVX	@DPTR,A			;MODE
		inc	dptr
		MOV	A,buf_rece1+4
		swap	a
		anl	a,#0fh
		MOVX	@DPTR,A			;fan speed	
		;CALL	AIR_PROC
		RET



proc5c_end_1:
		JBC	RECEIVED_DATA_FROM_SLAVE,PROC_DEVICE_SUB_PROC
		JNB	MASTER_NEED_SLAVE_DATA,proc5c_end_ADD
		CLR	MASTER_NEED_SLAVE_DATA
		MOV	A,BUF_RECE1+1
		CJNE	A,#0E0H,RETURN_COMMAND_NO_F9H;proc5c_end_ADD
		MOV	BUF_TRAN,BUF_RECE1
		MOV	BUF_TRAN+1,#DEVICE_CODE
		MOV	BUF_TRAN+2,boxnumber
		MOV	A,BUF_RECE1+3
		MOV	BUF_TRAN+3,A
;##################################################################
;		根据命令字处理
;##################################################################			
		CJNE	A,#0FEH,RETURN_COMMAND_NO_FEH
SLAVE_SEND_REQUEST_REBOOT:		
		MOV	A,BUF_RECE1+2
		ANL	A,#03H
		SWAP	A
		ORL	A,BUF_RECE1+4
		MOV	BUF_TRAN+4,A
		MOV	BUF_TRAN+5,BUF_RECE1+5
		MOV	BUF_TRAN+6,BUF_RECE1+6
		CALL	transbuf
		RET
RETURN_COMMAND_NO_FEH:		
		CJNE	A,#0F9H,RETURN_COMMAND_NO_F9H
		MOV	A,BUF_RECE1+2
		ANL	A,#03H
		JZ	RETURN_COMMAND_NO_F9H		;BOX =0 ERROR
		DEC	A
		RL	A
		RL	A
		RL	A
		ADD	A,#20H
		ADD	A,BUF_RECE1+4
		MOV	BUF_TRAN+4,A	
		MOV	BUF_TRAN+5,BUF_RECE1+5
		MOV	BUF_TRAN+6,BUF_RECE1+6
		CALL	transbuf
		ret
RETURN_COMMAND_NO_F9H:
		
		
		RET
CHECK_temperature_send_command:
                
                
                
                ret

		
receivedata:
		clr	sreceiveend
		mov	r2,#08h
		mov	r1,#buf_rece
		clr	a
receivedata1:
		add	a,@r1
		inc	r1
		djnz	r2,receivedata1
		;jnz	receivedata6		;receivedata error
		jnz     RETURN_COMMAND_NO_F9H
		setb	serviceled
		mov	netactivetime,#06h
		JB	panelsetup,NO_CHECK_START_TASK
		CALL	CHECK_START_TASK
		
NO_CHECK_START_TASK:
		mov	r1,#buf_rece
		cjne	@r1,#0F5h,procFA	;
		ljmp	receive1cproc		;receive 1C
procFA:		cjne	@r1,#0FAh,receivedata6d_ADD	;jomp receive no 5C
		inc	r1			;receive 5C
		cjne	@r1,#DEVICE_CODE,proc5c_end_1	;
		 				;receive A7	Upan
proca7:		inc	r1
		mov	a,@r1
		cjne	a,boxnumber,proc5c_end_1

procbox:	mov	a,buf_rece+3		;=boxnumber
		mov	r2,a
		cjne	a,#0FDh,procboxww	;BOX,ADD_H,01,0 LISTEN TO ME
						;BOX,ADD_H,00,00 GO BACK TO WORK
;########################################################################
;		PENEL SETUP PROC
;########################################################################
		CLR	MASTER_NEED_SLAVE_DATA
		MOV	A,buf_rece+4
		CJNE	A,#01H,SETUP_DIM8
		setb	panelsetup
		mov	a,buf_rece+5		;panel setup
		mov	TASK_100MS_DELAY,#100
		JNZ	goworkend
		clr	panelsetup		;panel again work

goworkend:	
		CLR	RECEIVED_DATA_FROM_SLAVE
		ret
SETUP_DIM8:
		CLR	C
		SUBB	A,#20H
		RR	A
		RR	A
		RR	A
		INC	A
		MOV	BUF_TRAN1+2,A			;BOX NO.
		MOV	BUF_TRAN1,buf_rece
		MOV	BUF_TRAN1+1,#0E0H
		MOV	BUF_TRAN1+3,buf_rece+3
		MOV	BUF_TRAN1+4,buf_rece+4
		MOV	BUF_TRAN1+5,buf_rece+5		;CHECK panel setup
		MOV	BUF_TRAN1+6,buf_rece+6
		call	transbuf_1	
		RET
receivedata6d_ADD:
		JMP	receivedata6d

procboxww:	cjne	a,#0BAh,receivedata2
requestver:
		MOV	A,buf_rece+4
		SETB	MASTER_NEED_SLAVE_DATA
		JNZ	SETUP_DIM8
		CLR	MASTER_NEED_SLAVE_DATA
		mov	dptr,#enablecode
		Lcall	trcode
		ret
receivedata2:	cjne	a,#0FCH,receivedata4
		SETB	MASTER_NEED_SLAVE_DATA
		MOV	A,buf_rece+4
		JNZ	SETUP_DIM8
		CLR	MASTER_NEED_SLAVE_DATA
		ljmp	0000h		;reboot

;receivedata3:	cjne	a,#040h,receivedata4
		;mov	dptr,#statuscode
		;Lcall	trcode
		;ret
receivedata6A:
		AJMP	receivedata0DF
receivedata4:	cjne	a,#0FAH,receivedata_NO_FA
		setb	hostwrite
		ajmp	re_wrproc
receivedata_NO_FA:	
		cjne	a,#0FBH,receivedata6A
		clr	hostwrite
re_wrproc:					;panel read or write
		mov	a,buf_rece+4
		CJNE	A,#20H,CHECK_ADDRE_PANEL
CHECK_ADDRE_PANEL:		
		JNC	DIM8_RE_WR
		mov	slvadr,a
		mov	a,buf_rece+5
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#buf_rece+6
		jb	hostwrite,writeproc
		mov	a,buf_rece+4
		jnz	read_no0
		mov	a,buf_rece+5
		jnz	read_no0
		mov	buf_rece+6,#DEVICE_CODE
		mov	r7,#0Bh			;read addr 0000 delay
		CALL	D10ms
		sjmp	re_wrproc2
read_no0:
		CALL	rcvdata_EEPROM
		MOV	R7,#008H	;15h
		CALL	D10ms
		sjmp	re_wrproc2
writeproc:
		CALL	SendData_EEPROM

;###############################################################
;		AGAIN READ WRITED DATA
;
;###############################################################
		CALL	waitwriteend
		mov	slvadr,buf_rece+4
		mov	subadr,buf_rece+5
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#buf_rece+6
		CALL	rcvdata_EEPROM

re_wrproc2:
		Lcall	copyr_tbuf
		mov	buf_tran+3,#0F9H

re_wrproc3:
		setb	tranready
		MOV	time20ms,#06H
		JNB	panelsetup,re_wrproc3_end
		mov	TASK_100MS_DELAY,#100
re_wrproc3_end:
		ret

DIM8_RE_WR:
		SUBB	A,#20H
		ANL	A,#0F0H
		RR	A
		RR	A
		RR	A
		MOV	BUF_TRAN1+2,A		;BOX
		MOV	A,buf_rece+4
		ANL	A,#07H
		MOV	BUF_TRAN1+4,A		;ADDRE HIGH
		MOV	BUF_TRAN1+5,buf_rece+5	;ADDRE LOW
		MOV	BUF_TRAN1+6,buf_rece+6	;DATA
		MOV	BUF_TRAN1,buf_rece
		MOV	BUF_TRAN1+1,#0E0H	;DIM8 DEVICE CODE
		MOV	BUF_TRAN1+3,buf_rece+3
		SETB	MASTER_NEED_SLAVE_DATA
		CALL	transbuf_1
		RET
		
receivedata_1:
		mov	r2,#08h
		mov	r1,#buf_rece1
		clr	a
receivedata1_1:
		add	a,@r1
		inc	r1
		djnz	r2,receivedata1_1
		jnz     re_wrproc3_end
		setb	serviceled
		mov	netactivetime,#06h
;###################################################################
;
;###################################################################		
		SETB	RECEIVED_DATA_FROM_SLAVE
		JNB	MASTER_NEED_SLAVE_DATA,receivedata1_2
		CLR	RECEIVED_DATA_FROM_SLAVE
receivedata1_2:
		MOV	R2,#08
		MOV	R0,#buf_rece1
		MOV	R1,#buf_rece
receivedata1_3:		
		MOV	A,@R0
		MOV	@R1,A
		INC	R0
		INC	R1
		DJNZ	R2,receivedata1_3
		SETB	sreceiveend
		RET
copyr_tbuf:

		setb	tranbuffull
		setb	tranready
		mov	r0,#buf_rece
		mov	r1,#buf_tran
		mov	r3,#08
copyr_tbuf1:	mov	a,@r0
		mov	@r1,a
		inc	r0
		inc	r1
		djnz	r3,copyr_tbuf1
		ret

trcode:
		jb	tranbuffull,trcode	;wait tranbuf is empty
		MOV	r0,#buf_tran
		mov	r2,#07
		clr	a
		mov	r1,a
trcode1:
		MOVC	A,@A+dptr
		MOV	@r0,A
		inc	r0
		inc	r1
		mov	a,r1
		djnz	r2,trcode1
		mov	buf_tran+2,boxnumber
trcode2:
		setb	tranbuffull
		setb	tranready
		RET
resetcode:	DB	0faH,DEVICE_CODE,04H,0feH,02h,03H,80H
enablecode:	db	0faH,DEVICE_CODE,04H,0feH,02h,03H,00H
		;20070712  resetcode:1.01=>1.02  normal ver
		;20070819  resetcode:1.02=>1.03  ADD MORE FUNCTION
		;20071118  resetcode:1.03=>1.04  ADD MORE FUNCTION	
		;20080814  resetcode:1.04=>2.00  没有插入门口卡功能
		;200801013  resetcode:2.00=>2.01  外部发请勿打扰码功能
		;20090521  resetcode:2.01=>2.02  Guest OUT 台灯关闭
		;20090623  resetcode:2.02=>2.03  Guest OUT 可以重新进入
						;清洁工进入场景为20~23
						;工程师进入场景为24~27
						;按键可以选择不同的人有功能.
						;RELASEPTR	0249=>	0259H
						;key_RELASEPTR	0239=>	0249H
						;请勿打扰灯亮时，请清洁灯灭
					
Guest_in_code:	db	0fah, DEVICE_CODE, 0ffh, 8eh ,01h, 0ffh, 0ffh ,0ceh;  Guest in		
;#######################################################################
;		1C CODE	处理程序
;		R7  =  opcode
;		r2  =  area
;########################################################################
a1ccodeprocend:
		ret
receive1cproc:
		CALL	CHECK_MAIN_SW_ON_OFF
		mov	a,BUF_RECE+3
		mov	r7,a				;R7  =  opcode
		mov	r2,BUF_RECE+1			;r2  =  area
		cjne	a,#0f8h,panelnolock
		CLR	F0
panellockproc:
		clr	keyenable
		lcall	panel_lock
		setb	keyenable
		lcall	panel_lock
		ret

panelnolock:	cjne	a,#0F6h,panelnolock1
		SETB	F0
		sjmp	panellockproc			;panic	JUMP
panelnolock1:	cjne	a,#0F7H,panelnolock2
panelunlockproc:
		mov	dptr,#key_PRESEPTR
		clr	keyenable
		lcall	panel_un_lock
		mov	dptr,#key_RELASEPTR
		setb	keyenable
		lcall	panel_un_lock
		ret

panelnolock2:	cjne	a,#0F5H,panelnolock3
		sjmp	panelunlockproc			;UNPANIC
panelnolock3:	cjne	a,#0F9H,CHECK_COMMAND_0FEH	;JUMP no JOIN
		;	JOIN CODE proc
		;mov	r3,BUF_RECE+2		;R3=join level
		;mov	r5,BUF_RECE+6		;R5=JOIN CODE
		mov	r4,#keyareaPRESEEE-4	;50h-4
		mov	r6,#16			;r6=4*4

join1:
		mov	slvadr,#00h
		mov	a,r4
		add	a,#04h
		mov	r4,a
		mov	subadr,a
		mov	bytecnt,#04h
		mov	pSMB_DATA_IN,#keyareatemp
		lcall	rcvdata_EEPROM			;get panel data start from 50h
		mov	a,keyjoinnewtemp
		jnz	join1_a
		mov	keyjoinnewtemp,keyjointemp
join1_a:
		mov	a,BUF_RECE+1		;r2=area
		JZ	join1_a_ADD
		cjne	a,keyareatemp,join3	;jmp area not equal
join1_a_ADD:
		mov	a,BUF_RECE+2
;###################################################################################
;		bit7 = 1,no join   add area
;###################################################################################
		jb	acc.7,join3
		;anl	a,#0fh			;mask high 4 bit
		anl	a,keyjoinnewtemp
		jz	join3			;jmp join code and=0  no change
;#################################################################################
;		get key command = 0x14 jump next data
;		use r2,r1,r0,acc,r7
;#################################################################################
		jmp	get_command_byte
get_command_byte_ok:
		mov	a,BUF_RECE+5
		ANL	A,#07FH			;D7 MUST BE 0
;###############################################################################
;		join   PROC
;
;
;#############################################################################
		JNZ	WRITER_NEWJOIN
		MOV	A,keyjointemp
WRITER_NEWJOIN:
		mov	keyjoinnewtemp,a
		mov	a,r4
		add	a,#03h
		mov	subadr,a
		mov     wordadd1,#00h
		mov	bytecnt,#01h
		mov	pSMB_DATA_OUT,#keyjoinnewtemp
		lcall	SendData_EEPROM
		CALL	waitwriteend
		ajmp 	join3

join3:		;next key data

		djnz	r6,join1		;proc next join data(byte)
		ret



CHECK_COMMAND_0FEH:
;################################################################
;		CHECK COMMAND IS 0FEH? YES SENSE NO ANATHER COMMAND
;################################################################
		CJNE	A,#0FEH,CHECK_COMMAND_0D3H
		JMP	sense_1			;find sense

CHECK_COMMAND_0D3H:
;################################################################
;		CHECK COMMAND IS 0D3H? YES SET CHANNEL NO ANATHER COMMAND
;################################################################
		CJNE	A,#0D3H,CHECK_COMMAND_OFF
		JMP	setchannel_1			;find setchannel

;#########################################################################
;		非场景,非通道命令检查
;
;#########################################################################
CHECK_COMMAND_OFF:
		cjne	r7,#0FDH,CHECK_F5_0D6H
		mov	keylednumber,#77h	;off	command
		clr	ledon
		ljmp	sense_2_a
CHECK_F5_0D6H:
		cjne	r7,#0D6h,CHECK_F5_0D5H	;6F
a1ccode_6e_6f:
		setb	ledon
a1ccode_74_off:
		mov	a,BUF_RECE+5		;Fade to toggle /Current preset proc
		cjne    a,#0ffh,a1ccode_74_off_add
                mov	keylednumber,#0bfh
		ljmp	sense_2_a
a1ccode_74_off_add:
		orl	a,#10000000b
		mov	keylednumber,a
		ljmp	sense_2_a
CHECK_F5_0D5H:
		cjne	r7,#0D5h,CHECK_F5_0DBH	;75 ON
		sjmp	a1ccode_6e_6f		;Fade to Current preset proc
CHECK_F5_0DBH:
		cjne	r7,#0DBh,CHECK_F5_0DCH	;74 OFF
		sjmp	a1ccode_6e_6f
CHECK_F5_0DCH:
		cjne	r7,#0DCh,CHECK_F5_0D4H	;70  TOGGLE
		clr	ledon			;Fade to Off proc
		sjmp	a1ccode_74_off
CHECK_F5_0D4H:
		cjne	r7,#0D4h,CHECK_F5_0D3H	;71 channel to level
;##############################################################################
;		70h toggle proc
;		setb toggle70
;##############################################################################
		setb	toggle70
		sjmp	a1ccode_74_off

CHECK_F5_0D3H:
		cjne	r7,#0D3h,CHECK_F5_0D2H	;FADE 0.1S
F5_code_D3_D2_D1:
		mov	a,BUF_RECE+6
		cjne	a,#0ffh,a1_71_level_01
		clr	ledon			;Fade to Off proc
		sjmp	a1ccode_74_off
a1_71_level_01:
		cjne	a,#01,CHECK_F5_END
					;Fade to ON proc
		sjmp	a1ccode_6e_6f

CHECK_F5_0D2H:
		cjne	r7,#0D2h,CHECK_F5_0D1H	;FADE 1S
		JMP	F5_code_D3_D2_D1
CHECK_F5_0D1H:
		cjne	r7,#0D1h,CHECK_F5_END	;FADE 1M
		JMP	F5_code_D3_D2_D1
CHECK_F5_END:
		;lcall	comp_all_byte
		ret				;no sense command
sense_1_add_1:		
		jmp     sense_1_add
;####################################################################
;
;
;####################################################################
sense_1	:
		mov	slvadr,#high(AIR_contrl_area_EE)
		mov	subadr,#low(AIR_contrl_area_EE)
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#temp
		lcall	rcvdata_EEPROM
		
		mov     a,BUF_RECE+1
		cjne    a,temp,sense_1_add_1
		mov	BUF_TRAN,#0FAH
		MOV	BUF_TRAN+1,#DEVICE_CODE
		MOV	BUF_TRAN+2,boxnumber
		MOV	BUF_TRAN+3,#09BH
		MOV	A,BUF_RECE+4
		jz      flag_scene_1
		cjne    a,#01h,flag_scene_3
		mov     a,#22h
                jmp     SEND_temperature_COMMAND_end
flag_scene_3:
		cjne    a,#02h,flag_scene_4
		mov     a,#12h
                jmp     SEND_temperature_COMMAND_end
flag_scene_4:
                cjne    a,#03h,flag_scene_5
		mov     a,#00h
                jmp     SEND_temperature_COMMAND_end
flag_scene_5:
                cjne    a,#04h,flag_scene_6
		mov     a,#33h
                jmp     SEND_temperature_COMMAND_end
flag_scene_6:
                cjne    a,#05h,flag_scene_7
		mov     a,#23h
                jmp     SEND_temperature_COMMAND_end
flag_scene_7:
                cjne    a,#06h,flag_scene_9
		mov     a,#13h
                jmp     SEND_temperature_COMMAND_end
flag_scene_9:
                cjne    a,#08h,flag_scene_10
		mov     a,#31h
                jmp     SEND_temperature_COMMAND_end
flag_scene_10:
                cjne    a,#09h,flag_scene_11
		mov     a,#21h
                jmp     SEND_temperature_COMMAND_end
flag_scene_11:
                cjne    a,#0ah,SEND_temperature_COMMAND_end
		mov     a,#11h
                jmp     SEND_temperature_COMMAND_end
flag_scene_1:
                mov     a,#32h
                jmp     SEND_temperature_COMMAND_end

SEND_temperature_COMMAND_end:
		MOV	BUF_TRAN+4,A
		mov     temp,a
		mov     dptr,#mode
		movx    a,@dptr
		anl     a,#0fh
		mov     r1,a
		inc     dptr
		movx    a,@dptr
		swap    a
		anl     a,#0f0h
		orl     a,r1
		cjne    a,temp,SEND_temperature_COMMAND_mod_fan
		jmp     CHECK_F5_END
SEND_temperature_COMMAND_mod_fan:
                mov     a,temp
                anl     a,#0f0h
                swap    a
                movx    @dptr,a         
                mov     dptr,#mode
                mov     a,temp
                anl     a,#0fh
                movx    @dptr,a		
		MOV	BUF_TRAN+6,TRUE_temperature
                mov     dptr,#set_temperature
                movx    a,@dptr
		MOV	BUF_TRAN+5,a
		setb	tranready
		clr	RECEIVED_DATA_FROM_SLAVE
		
		
		
		
		
		
		ret
sense_1_add:		
		mov     keylednumber,BUF_RECE+4
                mov     keyarea,BUF_RECE+1
                mov     keyjoin,BUF_RECE+2
                mov     keyjoinnew,BUF_RECE+2
                CALL	CHECK_EEPROM_40H
                CALL	CHECK_EEPROM_PRESEPTR
                CALL	CHECK_EEPROM_RELASEPTR
		call    ledproc
                CLR	A
                MOV	R7,A
                ;mov	r7,#0h
		;mov	r4,#42h-4			;get new join
		MOV	R4,A
sense_4:
		MOV	DPTR,#LED_40H_XRAM+2-4
		;mov	slvadr,#00h
		mov	a,r4
		add	a,#04h
		mov	r4,a
		ADD	A,DPL
		MOV	DPL,A
		PUSH	TEMP
		MOV	TEMP,#02H
		MOV	r1,#srelase
READ_42H_TO_srelase:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	TEMP,READ_42H_TO_srelase
		POP	TEMP
		;mov	slvadr,#00h
		;mov	a,r4
		;add	a,#04h
		;mov	r4,a
		;mov	subadr,a
		;mov	bytecnt,#02h
		;mov	r1,#srelase
		;lcall	rcvdata_EEPROM			;get panel save data 52h,53h
		mov	a,srelase+1
		jnz	sense_3
		mov	srelase+1,srelase	;CHANNELPOINT save new join
sense_3:
		;mov	dptr,#PRESEPTR
		MOV	DPTR,#PRESEPTR_XRAM

		lcall	comp_data
		jnb	f0,relase_comp
;###########################################################
;		PRESE is same,keyenable_buf的D0位置1
;
;###########################################################
		mov	acc,#keyenable_buf
		add	a,r7
		mov	r1,a
		mov	a,@r1
		orl	a,#00000001b
		mov	@r1,a
;###############################################################
;		set	led
;###############################################################
	;	mov	a,r7
	;	mov	b,#04h
	;	mul	ab
	;	add	a,#40h
	;	mov	slvadr,#00h
	;	mov	subadr,a
	;	mov	bytecnt,#04h
	;	mov	r1,#keyarea
	;	lcall	rcvdata_EEPROM			;get panel save data 52h,53h
	;	call	ledproc
		jmp	no_relase_comp



relase_comp:
		MOV	DPTR,#RELASEPTR_XRAM
		lcall	comp_data

;###########################################################
;		RELASE is same,keyenable_buf的D0位清0
;
;###########################################################
		mov	acc,#keyenable_buf
		add	a,r7
		mov	r1,a
		mov	a,@r1
		anl	a,#11111110b
		mov	@r1,a
no_relase_comp:
		inc	r7
		cjne	r7,#10,sense_4
		ret

setchannel_1:
		CALL	CHECK_EEPROM_40H
                CALL	CHECK_EEPROM_PRESEPTR
                CALL	CHECK_EEPROM_RELASEPTR
		CLR	A
		mov	r7,A
		MOV	R4,A
		;mov	r4,#42h-4			;get new join
setchannel_4:
		;mov	slvadr,#00h
		;mov	a,r4
		;add	a,#04h
		;mov	r4,a
		;mov	subadr,a
		;mov	bytecnt,#02h
		;mov	r1,#srelase
		;lcall	rcvdata_EEPROM			;get panel save data 52h,53h
		MOV	DPTR,#LED_40H_XRAM+2-4
		;mov	slvadr,#00h
		mov	a,r4
		add	a,#04h
		mov	r4,a
		ADD	A,DPL
		MOV	DPL,A
		PUSH	TEMP
		MOV	TEMP,#02H
		MOV	r1,#srelase
READ_42H_TO_srelase_1:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	TEMP,READ_42H_TO_srelase_1
		POP	TEMP

		mov	a,srelase+1
		jnz	setchannel_3
		mov	srelase+1,srelase		;temp save new join
setchannel_3:
		mov	dptr,#PRESEPTR_XRAM
		lcall	comp_data_channel
		jnb	f0,setchannel_relase_comp
;###########################################################
;		PRESE is same,keyenable_buf的D0位置1
;
;###########################################################
		mov	acc,#keyenable_buf
		add	a,r7
		mov	r1,a
		mov	a,@r1
		orl	a,#00000001b
		mov	@r1,a
		JMP	setchannel_no_relase_comp
setchannel_relase_comp:
		;mov	a,dpl
		;clr	c
		;subb	a,#8
		;mov	subadr,a
		;mov	bytecnt,#01h
		;mov	r1,#srelase
		;lcall	rcvdata_EEPROM
		;mov	a,srelase
		;jb	acc.5,setchannel_no_relase_comp
		;xrl	a,#099h
		;jz	setchannel_no_relase_comp
		;mov	dptr,#RELASEPTR
		MOV	DPTR,#RELASEPTR_XRAM
		lcall	comp_data_channel
		jnb	f0,setchannel_no_relase_comp
;###########################################################
;		RELASE is same,keyenable_buf的D0位清0
;
;###########################################################
		mov	acc,#keyenable_buf
		add	a,r7
		mov	r1,a
		mov	a,@r1
		anl	a,#11111110b
		mov	@r1,a
setchannel_no_relase_comp:
		inc	r7
		cjne	r7,#10h,setchannel_4
		ret

;##########################################################################
;		get command byte
;		r6 = key number
;  		use r2,r0,r1,acc,temp
;##########################################################################


get_command_byte:
		clr	c
		mov	a,#17
		subb	a,r6
		mov	b,#10h
		mul	ab
		mov	dptr,#key_PRESEPTR+3
		add	a,dpl
		mov	SUBADR,a
		mov	a,b
		addc	a,dph
		mov	SLVADR,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#temp
		call	rcvdata_EEPROM
		mov	a,temp
		cjne	a,#0F9h,get_command_byte_end
		jmp	join3
get_command_byte_end:
		jmp	get_command_byte_ok
;##############################################################
;		发光二极管处理程序  led proc
;		r2      bit counter  4bit
;		r3	led byte point
;		r4	key address	4*
;		r5	led buffer
;##############################################################


ledprocoff_allch:
		cjne	a,#0ffh,ledprocoff_all
		mov	a,keynumbertemp
		anl	a,#10000000b
		jnz	ledproc_allch_a
ledprocoff_all:
		mov	a,keylednumber
		Ljmp	ledprocoff		;next channel
ledproc_allch_a:
		mov	a,keylednumber
		Ljmp	ledproc_allch	;all ch & channel command

sense_2_a:	mov	keyarea,BUF_RECE+1		;area-> keyarea
		mov	keyjoin,BUF_RECE+5		;ture channel
		mov	keyjoinnew,BUF_RECE+2		;join -> keyjoinnew

ledproc:
		mov 	a,keylednumber
		cjne 	a,#07eh,ledproc2	;???
		ljmp	ledproc1		;7E no proc(up,down,proc)
ledproc2:
		;mov	r4,#keyareaPRESEEE-4	;50h-4
		CLR	A
		mov	r7,A			;r6=0
		MOV	R4,A
		mov	temp,#onetouch_BUF		;onetouch_BUF address
ledproc6:
		MOV	DPTR,#LED_40H_XRAM-4
		;mov	slvadr,#00h
		mov	a,r4
		add	a,#04h
		mov	r4,a
		ADD	A,DPL
		MOV	DPL,A
		PUSH	TEMP
		MOV	TEMP,#04H
		MOV	r1,#keyareatemp
READ_40H_TO_keyareatemp:
		MOVX	A,@DPTR
		MOV	@R1,A
		INC	R1
		INC	DPTR
		DJNZ	TEMP,READ_40H_TO_keyareatemp
		POP	TEMP

		;mov	slvadr,#00h
		;mov	a,r4
		;add	a,#04h
		;mov	r4,a
		;mov	subadr,a
		;mov	bytecnt,#04h
		;mov	r1,#keyareatemp
		;lcall	rcvdata_EEPROM			;get panel save data 50h
		mov	a,keyjoinnewtemp
		jnz	ledproc5_a
		mov	keyjoinnewtemp,keyjointemp
ledproc5_a:
		mov	dptr,#PRESEPTR+2
		mov	acc,r7
		mov	b,#10h
		mul	ab
		add	a,dpl
		mov	dpl,a
		jnc	get_chdata_1
		inc	dph

get_chdata_1:
		mov	a,b
		jz	get_chdata_2
		inc	dph

get_chdata_2:					;new point
		mov	slvadr,dph
		mov	subadr,dpl
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#keyjointemp		;true channel
		lcall	rcvdata_EEPROM			;get panel save data

		mov	a,keyarea
		jz	ledproc5a
		cjne	a,keyareatemp,ledproc3	;jmp area not equal
ledproc5a:
		mov	a,keyjoinnew
		anl	a,keyjoinnewtemp
		jz	ledproc3		;jmp join code and=0
		mov	a,keyjoin
		cjne	a,keyjointemp,ledprocoff_allch_ADD	;ture channel comp
ledproc_allch:

;####################################################################
;		Toggle proc need onetouch_BUF
;
;
;####################################################################
		jnb	toggle70,notoggle70
		ljmp	toggle70proc
ledprocoff_allch_ADD:
		JMP	ledprocoff_allch
notoggle70:

ledprocon:
		mov	a,keynumbertemp
		cjne	a,#80h,ledprocon_onetouch
ledprocon_onetouch:
		jc	ledproc4
		CJNE	A,#0C0H,CHECK_SETCHANNEL_B
CHECK_SETCHANNEL_B:
		JNC	ledproc4		;SET CHANNEL COMMAND
;######################################################
;		onetouch press proc
;		onetouch_BUF setb =1
;		ledon
;######################################################
		jnb	ledon,ledprocoff2
		mov	a,#onetouch_BUF
		add	a,r7
		mov	r1,a
		mov	a,@r1
		orl	a,#00000001b
		mov	@r1,a			;onetouch on presee
		sjmp	ledproc4
ledproc6_a:
		sjmp	ledproc6
ledprocoff:	cjne	a,#77h,ledprocoff1
ledprocoff2:
		mov	a,keynumbertemp
		cjne	a,#80h,ledonetouch
ledonetouch:	jc	ledproc4
		CJNE	A,#0C0H,CHECK_SETCHANNEL_C
CHECK_SETCHANNEL_C:
		JNC	LEDPROC4
		;mov	r1,b
		mov	a,#onetouch_BUF
		add	a,r7
		mov	r1,a
		mov	a,@r1
		anl	a,#11111110b
		mov	@r1,a			;onetouch off presee
		sjmp	ledproc4
ledprocoff1:	cjne	a,#080h,ledproc7

ledproc7:	jc	ledprocoff3
		CJNE	A,#0C0H,CHECK_SETCHANNEL_F
CHECK_SETCHANNEL_F:
		JNC	ledprocoff3
		JMP	ledproc3		;nochang
ledprocoff3:	mov	a,keynumbertemp
		cjne	a,#080h,ledprocoff4
ledprocoff4:	jc	ledprocoff2
		;jb	ledon,ledprocon		onetouchkey led proc
		CJNE	A,#0C0H,CHECK_SETCHANNEL_E
CHECK_SETCHANNEL_E:
		JNC     LEDPROCOFF2



ledproc3:

ledproc4:					;next key data
		;inc	temp
		inc	r7
		cjne	r7,#10h,ledproc6_a		;proc next led data(byte)
;#######################################################################
;		masterled proc
;		led3.4=0 or led3.0	led ON
;		led3.4=1 or led3.0	led OFF
;#######################################################################

ledproc1:
		clr	toggle70
		ret
;#######################################################################
;		Toggle 70h  proc
;
;#######################################################################

toggle70proc:
		mov	a,#onetouch_BUF
		add	a,r7
		mov	r1,a
		MOV	A,@R1
		ANL	A,#01H
		setb	ledon
;#####################################################################
;		原来为关,现为开,jump
;
;#####################################################################

		JZ	notoggle70
;#####################################################################
;		原来为开,现为关
;
;#####################################################################
		clr	ledon
		ljmp	notoggle70


;toggle70proc_OFFtoON:
;#####################################################################
;		原来为关,现为开
;
;#####################################################################


;		ajmp	@@@@@@@@
;########################################################
;
;     host block write proc 6C code
;
;
;########################################################
blockwritePROC:
		JNB	blockwrite,blockwriteEND
		;mov	slvadr,#EEPROM
		mov	a,SUBADR
		mov	r6,#06h
		mov	r2,a		;r2 = old low addre
		mov	r3,wordadd1	;r3 = old high addre
		ADD	A,r6
		;jmp	0000h
		jnb	ac,blockwrite3	;jump page write not over
		anl	a,#0fh
		jz	blockwrite3
		mov	r5,a		;r5 = next page BYTECNT
		mov	a,r6
		clr	c
		subb	a,r5		;get new BYTECNT (6 - next page BYTECNT)
		mov	r4,a		;r4 = new BYTECNT
		mov	BYTECNT,a
		MOV	pSMB_DATA_OUT,#BUF_RECE+1

		LCALL	SendData_EEPROM	;WRITE old page TO EEPROM
		call	waitwriteend
;###################################################################
;
;		WRITE new page TO EEPROM
;
;###################################################################
		;mov	slvadr,#EEPROM
		mov	a,SUBADR
		add	a,r4
		mov	SUBADR,a		;next page adder
		clr	a
		addc	a,r3
		mov	wordadd1,a
		MOV	BYTECNT,r5
		mov	a,#BUF_RECE+1
		add	a,r4
		mov	pSMB_DATA_OUT,a
		LCALL	SendData_EEPROM	;WRITE new page TO EEPROM
		jmp	blockwrite4
blockwrite3:
		MOV	BYTECNT,r6
		MOV	pSMB_DATA_OUT,#BUF_RECE+1
		LCALL	SendData_EEPROM		;WRITE 6 BYTE TO EEPROM
blockwrite4:
		clr	ren0
		mov	BUF_TRAN,#0FAh
		mov	BUF_TRAN+1,#DEVICE_CODE	;device code 6 relay
		MOV	BUF_TRAN+2,boxnumber
		MOV	BUF_TRAN+3,#0DCh
		mov	BUF_TRAN+4,r3		;high address
		mov	BUF_TRAN+5,r2		;low address
		mov	a,r2
		ADD	A,#6
		mov	SUBADR,a		;address +6
		clr	a
		MOV	BUF_TRAN+6,a
		addc	a,r3
		mov	wordadd1,a
		jmp	re_wrproc3
blockwriteEND:	RET
		NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP

;########################################################
;
;     host block read proc 31h code
;
;
;########################################################
blockreadPROC:
		mov	BUF_TRAN,#0fch
		MOV	SUBADR,BUF_RECE+5	;LOW ADDER
		mov	wordadd1,BUF_RECE+4	;HIGH ADDER
		MOV	BYTECNT,#06H
		MOV	pSMB_DATA_IN,#BUF_TRAN+1
		;mov	slvadr,#EEPROM
		CALL	rcvdata_EEPROM
		JMP	RE_WRPROC3

;################################################################
;               check 50H(LED) area = send command area ?
;		let 50H area = send command area i's right
;
;###############################################################
checkarea:

		mov	r5,#0ffh
checkarea1:
		inc	r5
		mov	bytecnt,#02
		mov	slvadr,#00h
		mov	a,r5
		mov	b,#04h
		mul	ab
		add	a,#keyareaPRESEEE
		mov	r4,a			;r4 save addre
		mov	subadr,a
		mov	pSMB_DATA_IN,#keyarea
		Lcall	rcvdata_EEPROM
		mov	a,keylednumber
		cjne 	a,#0ffh,checkarea2	;
		jmp	nextkeycheck
checkarea2:
		cjne 	a,#07eh,checkarea3
		jmp	nextkeycheck
checkarea3:
		mov	dptr,#preseptr
		mov	a,#10h
		mov	b,r5
		mul	ab
		mov	r0,a
		;mov	r1,#07h
		mov	a,dpl
		add	a,r0
		mov	dpl,a
		jnc	checkarea4
		inc	dph
checkarea4:
		mov	subadr,a
		mov	a,dph
		mov	slvadr,a
		mov	bytecnt,#02h
		mov	pSMB_DATA_IN,#keyjoin
		Lcall	rcvdata_EEPROM
		mov	a,keyjoin
		cjne	a,#0ffh,checkarea5
		inc	r4
		mov	keyjoinnew,#0ffh
		sjmp	checkarea6
checkarea5:	mov	a,keyjoinnew			;check area
		cjne	a,keyarea,checkarea6

nextkeycheck:	mov	a,r5
		cjne	a,#keynumber_all-1,checkarea1		;keynumber=16
		ret

checkarea6:						;write new area or FF
		mov	bytecnt,#1
		mov	slvadr,#00h
		mov	subadr,r4
		mov	pSMB_DATA_OUT,#keyjoinnew
		LCALL	SendData_EEPROM
		call	waitwriteend
		sjmp	nextkeycheck
;########################################################
;		5c,a7,box,06,04,addre,data   write
;
;		5c,a7,box,06,03,addre,xx     read
;return:	5c,a7,box,06,05,addre,data
;#######################################################
requestRAM:
		MOV	a,BUF_RECE+4
		cjne	a,#04,RAMread
;########################################################
;		ram write BUF_RECE+5=addre BUF_RECE+6=data
;########################################################
		mov	r0,BUF_RECE+5
		mov	@r0,BUF_RECE+6
		lcall   copyr_tbuf

		mov	BUF_TRAN+4,#06h
		setb	tranready
		ret
RAMread:	cjne	a,#03,requestRAMend
;########################################################
;		ram read BUF_RECE+5=addre
;########################################################
requestRAMend:
		lcall   copyr_tbuf
		mov	r0,BUF_RECE+5
		mov	a,@r0
		mov	BUF_TRAN+6,a
		mov	BUF_TRAN+4,#05h
		setb	tranready
		ret
;################################################################
;		2002.12.5
;
;
;################################################################
check_key:
		jb	keyprese,key_keyprese_622
;################2009 06 22  ###############check key with guest in  #############
		mov	dptr,#RELASEPTR-7		
		jmp	key_keyprese_622_1
key_keyprese_622:		
;################2009 06 22  ###############check key with guest in  #############		
		mov	dptr,#PRESEPTR-7		
key_keyprese_622_1:		
		mov	a,#10h
		mov	b,keynumber
		mul	ab
		mov	r0,a
		mov	a,dpl
		add	a,r0
		mov	dpl,a
		jnc	key_keyprese_key_with_guest_in 
		inc	dph
key_keyprese_key_with_guest_in:
		mov	subadr,a
		mov	a,dph
		ADD	A,B
		mov	wordadd1,a
		mov	bytecnt,#1
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM		;key with guest in buf	
		;mov	a,RTC_BUF
		;cjne	a,#0ffh,key_keyprese_key_with_guest_in_1
		;jmp	key_disable
;key_keyprese_key_with_guest_in_1:			
		mov	dptr,#man_in_status
		movx	a,@dptr
		cjne	a,#01h,key_keyprese_key_with_guest_in_02
		mov	a,RTC_BUF
		jb	acc.2,check_key_622
		jmp	key_disable
key_keyprese_key_with_guest_in_02:		
		cjne	a,#02h,key_keyprese_key_with_guest_in_03
		mov	a,RTC_BUF
		jb	acc.3,check_key_622
		jmp	key_disable
key_keyprese_key_with_guest_in_03:		
		mov	a,RTC_BUF
		jnb	acc.4,key_disable
		
check_key_622:		
		
		MOV	A,#keyenable_buf
		ADD	A,KEYNUMBER
		MOV	R1,A
		MOV	A,@R1			;get key status
		jb	keyprese,key_keyprese
		anl	a,#40h
		jnz	key_disable
		ajmp	key_enable
key_keyprese:
		anl	a,#80h
		jnz	key_disable
key_enable:
		ljmp	check_key_ok
key_disable:
		JB	keyprese,SETB_keyenable_buf_keyprese
		MOV	A,#keyenable_buf
		ADD	A,KEYNUMBER
		MOV	R1,A
		MOV	A,@R1			;get key status
		ANL	A,#0FEH
		MOV	@R1,A
		jmp	KEY_INPUT_PROC_END
SETB_keyenable_buf_keyprese:
		MOV	A,@R1			;get key status
		ORL	A,#01H
		MOV	@R1,A
		ljmp	KEY_INPUT_PROC_END
;##################################################################
;		r2=area r3=counter
;		PANEL LOCK
;
;##################################################################
panel_lock:


		mov	r3,#0h
		mov	r4,#0h
panel_lock_1:
		mov	dptr,#key_PRESEPTR
		jnb	keyenable,panel_lock_prese
		mov	dptr,#key_RELASEPTR
panel_lock_prese:
		mov	a,r4
		add	a,#10h
		mov	r4,a
		jnc	panel_lock_prese_low
		inc	dph
		;inc	dph
panel_lock_prese_low:
		add	a,dpl
		mov	dpl,a
		jnc	panel_lock_2
		inc	dph
panel_lock_2:
		mov	subadr,a
		mov	slvadr,dph
		mov	bytecnt,#02h
		mov	pSMB_DATA_IN,#srelase
		lcall	rcvdata_EEPROM
		mov	a,srelase
		cjne	a,#1ch,panel_lock_3
;###################################################################
;		AREA = 0 CHECK
;###################################################################
		CJNE	R2,#00H,AREA_CHECK
		JMP	AREA_CHECK_OK
AREA_CHECK:
		mov	a,srelase+1		;temp area
		xrl	a,r2
		jnz	panel_lock_3
AREA_CHECK_OK:
;##################################################################
;		CHECK KEY COMMAND = UNPANIC?
;		OK THIS KEY NON DISABLE
;##################################################################
		JNB	F0,NORMOL_KEY_PROC
		MOV	A,DPL
		ADD	A,#3			;COMMAND ADDRE
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#srelase
		lcall	rcvdata_EEPROM
		MOV	A,srelase
		CJNE	A,#17H,CHECK_UNPANIC_COMMAND
		JMP	panel_lock_3		;PANIC KEY NON DISABLE
CHECK_UNPANIC_COMMAND:
		CJNE	A,#18H,NORMOL_KEY_PROC
		JMP	panel_lock_3		;UN PANIC KEY NON DISABLE

NORMOL_KEY_PROC:
		mov	a,dpl
		clr	c
		subb	a,#07h
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#srelase
		lcall	rcvdata_EEPROM
		mov	a,srelase
		anl	a,#00000001b
		jz	panel_lock_6
		mov	a,#keyenable_buf
		add	a,r3
		mov	r0,a
		mov	a,@r0
		jb	keyenable,panel_lock_4
		orl	a,#80h
		sjmp	panel_lock_5
panel_lock_6:
		mov	a,srelase
		anl	a,#00000010b
		jnz	panel_lock_3		;DYNET Mute
		;cjne	a,#0fbh,panel_lock_3
		mov	a,#keyenable_buf
		add	a,r3
		mov	r0,a
		mov	a,@r0
		orl	a,#0c0h
		sjmp	panel_lock_5
panel_lock_4:
		orl	a,#40h			;relase disable
panel_lock_5:
		mov	@r0,a
panel_lock_3:

		inc	r3
		cjne	r3,#10h,panel_lock_1_ADD
		ret
panel_lock_1_ADD:
		JMP	panel_lock_1
;#################################################################
;		PANEL UNLOCK
;
;
;#################################################################
panel_un_lock:
		mov	r3,#0h
		mov	r4,#00
panel_un_1:
		mov	dptr,#key_PRESEPTR
		jnb	keyenable,panel_un_prese
		mov	dptr,#key_RELASEPTR
panel_un_prese:
		mov	a,r4
		add	a,#10h
		mov	r4,a
		jnc	panel_un_prese_low
		inc	dph
		;inc	dph
panel_un_prese_low:
		add	a,dpl
		mov	dpl,a
		jnc	panel_un_2
		inc	dph
panel_un_2:
		mov	subadr,a
		mov	slvadr,dph
		mov	bytecnt,#02h
		mov	pSMB_DATA_IN,#srelase
		lcall	rcvdata_EEPROM
		mov	a,srelase
		cjne	a,#1ch,panel_un_3
		mov	a,r2
		jz	panel_un_2_area0
		mov	a,channelpoint			;temp area
		xrl	a,r2
		jnz	panel_un_3
panel_un_2_area0:
		mov	a,dpl
		clr	c
		subb	a,#07h
		mov	subadr,a
		mov	bytecnt,#01h
		mov	pSMB_DATA_IN,#srelase
		lcall	rcvdata_EEPROM
		mov	a,srelase
		anl	a,#00000001b
		jz	panel_un_6
		mov	a,#keyenable_buf
		add	a,r3
		mov	r0,a
		mov	a,@r0
		jb	keyenable,panel_un_4
		ANL	a,#7Fh			;prese
		sjmp	panel_un_5
panel_un_6:
		mov	a,srelase
		anl	a,#00000010b
		jz	panel_un_3
		mov	a,#keyenable_buf
		add	a,r3
		mov	r0,a
		mov	a,@r0
		ANL	a,#3Fh			;prese and relase
		sjmp	panel_un_5

panel_un_4:
		ANL	a,#0BFh			;relase
panel_un_5:
		mov	@r0,a
panel_un_3:
		inc	r3
		mov	a,r3
		cjne	a,#10h,panel_un_1
		ret

;#################################################################################
;		AUTO	START    TASK PROC
;
;#################################################################################
CHECK_START_TASK:
		MOV	wordadd1,#00
		mov	SUBADR,#START_TASK_POINT
		mov	BYTECNT,#2
		MOV	pSMB_DATA_IN,#RTC_BUF
		call	rcvdata_EEPROM
		MOV	DPH,(RTC_BUF)
		MOV	DPL,(RTC_BUF+1)		;SAVE START_TASK_POINT
		MOV	A,#0FFH
		CJNE	A,(RTC_BUF),GET_NEW_ADDRE
		CJNE	A,(RTC_BUF+1),GET_NEW_ADDRE
		RET				;12H = 0FFH,13H=0FFH,NO_START_TASK
GET_NEW_ADDRE:
		MOV	wordadd1,DPH
		MOV	A,DPL
		MOV	SUBADR,A
		ADD	A,#08H			;NEXT START_TASK_POINT
		MOV	DPL,A
		JNC	GET_NEW_ADDRE_3
		INC	DPH
GET_NEW_ADDRE_3:
		mov	BYTECNT,#8
		MOV	pSMB_DATA_IN,#CLOCK_4D
		call	rcvdata_EEPROM
		MOV	A,CLOCK_4D
		JZ	CHECK_START_TASK_END
		CJNE	A,#0FFH,CAN_BE_CHECK_STARTUP_TASK
CHECK_START_TASK_END:
		RET
CAN_BE_CHECK_STARTUP_TASK:
		CALL	COMP_RECEV_DATA_EEROM_DATA
;###########################################################################
;		F0=0,MUST BE START TASK,TASK_NO=CLOCK_4D+7
;###########################################################################
		JB	F0,GET_NEW_ADDRE
		MOV	R1,(CLOCK_4D+7)
		MOV	A,#80H
GET_PR_LEVEL_BIT:
		RL	A
		DJNZ	R1,GET_PR_LEVEL_BIT
		ORL	PR_LEVEL,A		;TASK BIT =1
;###################################################################################
;		BASE TASK NUMBER R3 GET TASK ADDRE
;###################################################################################
		MOV	A,(CLOCK_4D+7)
		CALL	READ_TASK_POINT_T0_7_VECTOR
		jmp	GET_NEW_ADDRE		;20060711  can be start anather task
		RET




COMP_RECEV_DATA_EEROM_DATA:
		mov	r3,#07h
		mov	r0,#CLOCK_4D		;EEROM DATA
		mov	r1,#BUF_RECE		;RESEVE DATA
		CLR	F0
COMP_RECEV_DATA_EEROM_DATA_1:
		mov	a,@r0
		CJNE	A,#0FFH,COMP_RECEV_DATA_EEROM_DATA_2
		SJMP	COMP_RECEV_DATA_EEROM_DATA_3		;JUMP NO COMP
COMP_RECEV_DATA_EEROM_DATA_2:
		XRL	a,@r1
		jz	COMP_RECEV_DATA_EEROM_DATA_3
		SETB	F0
		RET						;NOT EQU RETURN
COMP_RECEV_DATA_EEROM_DATA_3:
		inc	r0
		inc	r1
		djnz	r3,COMP_RECEV_DATA_EEROM_DATA_1
		RET		
		NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		;JNB	flag20ms,REPET
		;CLR	flag20ms
		;DJNZ	SECOND,REPET
		;MOV	SECOND,#50
		;mov	a,p4
		;cpl	acc.5
		;mov	p4,a
		;MOV     A,#30H
		;mov	r1,#8
		;mov	r0,#BUF_RECE
iic_test:		
		;mov	@r0,a
		;inc	a
		;inc	r0
		;djnz	r1,iic_test
		;MOV	wordadd1,#02
		;mov	SUBADR,#0
		;mov	BYTECNT,#8
		;MOV	pSMB_DATA_OUT,#BUF_RECE
		;call	SendData_EEPROM
		;MOV	wordadd1,#00
		;mov	SUBADR,#00
		;mov	BYTECNT,#7
                ;MOV     BUF_RECE+0,#32H
                ;MOV     BUF_RECE+1,#52H
                ;MOV     BUF_RECE+2,#15H
                ;MOV     BUF_RECE+3,#07H
                ;MOV     BUF_RECE+4,#30H
                ;MOV     BUF_RECE+5,#06H
                ;MOV     BUF_RECE+6,#07H
		;MOV	pSMB_DATA_OUT,#BUF_RECE
                ;MOV     CHIP_ADDRE,#0D0H
                ;CLR     NEED_SEND_WORDADD
                ;CALL    SENDDATA
                ;NOP
		
		;MOV	wordadd1,#00
		;mov	SUBADR,#1
		;mov	BYTECNT,#6
		;MOV	pSMB_DATA_IN,#BUF_TRAN1+1
                ;MOV     CHIP_ADDRE,#0D0H
                ;CLR     NEED_SEND_WORDADD
		;call	rcvdata
		;NOP
		;MOV	BUF_TRAN1,#0FAH
		;lcall	transbuf_1
		
		JMP	REPET  
;####################################################################
;
;####################################################################		
OUT_403H:		  
		MOV	pSMB_DATA_IN,#RTC_BUF
		MOV	wordadd1,#03H
		mov	BYTECNT,#1
		call	rcvdata_EEPROM		;得到一字节
		MOV	A,RTC_BUF
		JBC	ACC.4,OUT_403H_END
		MOV	C,ACC.0
		MOV	RELAY_OUT.0,C
		MOV	C,ACC.1
		MOV	RELAY_OUT.1,C
		
		
OUT_403H_END:		
		RET
		
proc_master_off:		;08h master key
		jbc	keyprese,master_off_keyprese_proc
		clr	keyrelase
		ret
MASTER_ON_PROC:
		MOV	pSMB_DATA_IN,#RTC_BUF
		MOV	wordadd1,#02H
		mov	SUBADR,#24H
		mov	BYTECNT,#4
		call	rcvdata_EEPROM		;GET AREA,FADE_L,FADE_H,JOIN
		MOV	A,keynumber
		MOV	B,#07H
		MUL	AB
		ADD	A,#key_tran_buf
		MOV	R0,A
		MOV	A,#0F5H			;1
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF		;AREA
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+3		;JOIN
		MOVX	@R0,A
		INC	R0
		MOV	A,#0FEH			;COMMAND
		MOVX	@R0,A
		INC	R0
		MOV	A,#0			;SCENE  0=ON
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+1		;FADE_L
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+2		;FADE_H
		MOVX	@R0,A
		INC	R0
		MOV	A,keynumber
		ADD	A,#onetouch_BUF
		MOV	R0,A
		CLR	A
		SETB	ACC.2
		MOV	@R0,A			;置位发送位
		mov	r0,#onetouch_BUF+14
		mov	a,@r0
		setb	acc.0
		mov	@r0,a
;##############################################################################
;		处理403H 4位开关
;##############################################################################		
		mov	SUBADR,#54h			;#20+#40H		;总开
		CALL	OUT_403H
		
		
		RET		
master_off_keyprese_proc:
		
		
		
		
		
		JBC	MASTER_OFF,MASTER_ON_PROC			
		MOV	pSMB_DATA_IN,#RTC_BUF
		MOV	wordadd1,#02H
		mov	SUBADR,#20H
		mov	BYTECNT,#4
		call	rcvdata_EEPROM		;GET AREA,FADE_L,FADE_H,JOIN
		MOV	A,keynumber
		MOV	B,#07H
		MUL	AB
		ADD	A,#key_tran_buf
		MOV	R0,A
		MOV	A,#0F5H			;1
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF		;AREA
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+3		;JOIN
		MOVX	@R0,A
		INC	R0
		MOV	A,#0FEH			;COMMAND
		MOVX	@R0,A
		INC	R0
		MOV	A,#11			;SCENE  12=OFF
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+1		;FADE_L
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+2		;FADE_H
		MOVX	@R0,A
		INC	R0
		SETB	MASTER_OFF
		MOV	A,keynumber
		ADD	A,#onetouch_BUF
		MOV	R0,A
		CLR	A
		SETB	ACC.2
		MOV	@R0,A			;置位发送位
		mov	r0,#onetouch_BUF+14
		mov	a,@r0
		clr	acc.0
		mov	@r0,a
;##############################################################################
;		处理403H 4位开关
;##############################################################################		
		mov	SUBADR,#53h		;#19+#40H
		CALL	OUT_403H
master_off_keyprese_proc_END:		
		ret
		
;##############################################################################
;		处理人进入时,是谁?清洁工进入时,要灭打扫灯(此灯电源要常有)
;		FA AC BOX 8E 01 X X   客人进入
;		FA AC BOX 8E 02 X X   清洁工进入
;		FA AC BOX 8E 03 X X   维修工进入
;##############################################################################
CHECK_WHAT_MAN_IN:
		MOV	A,buf_rece1+4
		MOV	DPTR,#man_in_status
		MOVX	@DPTR,A		;人进入时,状态保存到man_in_status
		CJNE	A,#02H,master_off_keyprese_proc_END
;###################   清洁工进入   ########################################
		CLR	CLEAR_LAMP_ON
		CALL	CLEAR_CLEAR_LAMP
		RET
CLEAR_CLEAR_LAMP:				
		clr	need_clear_lamp
		RET
		
open_first_FIND_TIME_PROC:		
		;DEC     R2
                mov	wordadd1,#02h
		mov	a,R2
		MOV	R3,A
		add	a,#0FH
		mov	subadr,A
		MOV	R2,A
		mov	pSMB_DATA_IN,#RTC_BUF	;插排区号
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		mov	a,R3 
		RL      A                       ;R3*2
		ADD     A,#12H
		mov	subadr,a
		mov	wordadd1,#02h
		mov	pSMB_DATA_IN,#RTC_BUF+1	;插排FADE TIME
		mov	bytecnt,#02
		call	rcvdata_EEPROM
		MOV	A,R3
		ADD	A,#1BH
		mov	subadr,a
		mov	wordadd1,#02h
		mov	pSMB_DATA_IN,#RTC_BUF+3	;插排JOIN
		mov	bytecnt,#01
		call	rcvdata_EEPROM	
			
		MOV	A,keynumber
		MOV	B,#07H
		MUL	AB
		ADD	A,#key_tran_buf
		MOV	R0,A
		MOV	A,#0F5H			;1
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF		;AREA
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+3		;JOIN
		MOVX	@R0,A
		INC	R0
		MOV	A,#0FEH			;COMMAND
		MOVX	@R0,A
		INC	R0
;***************   20090622  *******;		;清洁工进入场景为20~23
						;工程师进入场景为24~27		
		;MOV	A,R3			;R3 = 1,2,3,4
		;ADD	A,#11			;SCENE  13,14,15,16
		mov	dptr,#man_in_status
		movx	a,@dptr
		cjne	a,#01h,guest_02_IN
		MOV	A,R3			;R3 = 1,2,3,4
		ADD	A,#11			;SCENE  13,14,15,16
		jmp	guest_ALL_IN	
guest_02_IN:
		cjne	a,#02h,guest_03_IN
		MOV	A,R3			;R3 = 1,2,3,4		清洁工进入
		ADD	A,#18			;SCENE  20,21,22,23
		jmp	guest_ALL_IN
guest_03_IN:		
		MOV	A,R3			;R3 = 1,2,3,4		工程师进入	
		ADD	A,#22			;SCENE  24,25,26,27
guest_ALL_IN:		
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+1		;FADE_L
		MOVX	@R0,A
		INC	R0
		MOV	A,RTC_BUF+2		;FADE_H
		MOVX	@R0,A
		INC	R0
		CLR	MASTER_OFF
		MOV	A,keynumber
		ADD	A,#onetouch_BUF
		MOV	R0,A
		CLR	A
		SETB	ACC.2
		MOV	@R0,A			;置位发送位
;##############################################################################
;		处理403H 4位开关
;##############################################################################
		MOV	A,R3
		ADD	A,#4CH		
		mov	SUBADR,A
		CALL	OUT_403H
		ret		
	
proc_open_first_sw:		;插排开关
		jbc	keyprese,open_first_keyprese_proc
		clr	keyrelase
		mov	wordadd1,#HIGH(MAN_OUT_DELAY_EE)
		mov	subadr,#LOW(MAN_OUT_DELAY_EE)
		mov	pSMB_DATA_IN,#TEMP		;人离开延时时间
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		MOV	DPTR,#MAN_OUT_DELAY
		MOV	A,TEMP
		MOVX	@DPTR,A
		clr	do_not_disturb_lamp		;led off
		jnb	CLEAR_LAMP_ON,proc_open_first_sw_1
		setb	need_clear_lamp
		
		
proc_open_first_sw_1:		
		
		ret
		
open_first_keyprese_proc:
		MOV	DPTR,#MAN_OUT_DELAY
		CLR	A
		MOVX	@DPTR,A
		mov	wordadd1,#02h
		mov	subadr,#000h
		mov	pSMB_DATA_IN,#KEY_READ_BUFF	;插排开关四个时间段->r1
		mov	bytecnt,#08
		call	rcvdata_EEPROM
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#001h
		mov	pSMB_DATA_IN,#RTC_BUF		;RTC_buf address->r1
		mov	bytecnt,#02
		call	rcvdata
		mov	a,RTC_BUF+1
		anl	a,#01111111b
		MOV	RTC_BUF+1,A				;SAVE 小时
		MOV	R0,#KEY_READ_BUFF
		MOV	R2,#00H
		MOV	R4,#04H
FIND_FOUR_TIME:		
		MOV	A,@R0
		CJNE	A,RTC_BUF+1,CHECK_HOUR_1
		JMP	CHECK_MIN
CHECK_HOUR_1:		
		JC	GET_NNEXT_FOUR_POINT
;###########################################################
;		找到时间段,取数据
;###########################################################		
		CJNE	R0,#KEY_READ_BUFF,NO_FIRST_TIME
;###########################################################
;		时间小于第一时间段,要取第四个数据
;###########################################################		
		MOV	R2,#04
		
NO_FIRST_TIME:		
		CALL	open_first_FIND_TIME_PROC	;R2=时间段指针
		ret

		
CHECK_MIN:
		INC	R0
		MOV	A,@R0				;分钟
		CJNE	A,RTC_BUF,CHECK_MIN_1
;############################################################
;		分钟相等
;############################################################
		INC	R2		
		JMP	NO_FIRST_TIME
CHECK_MIN_1:		
		JNC	CHECK_MIN_2
		INC	R2
		JMP	NO_FIRST_TIME
GET_NNEXT_FOUR_POINT:		
		INC	R0
		INC	R0
		INC	R2
		DJNZ	R4,FIND_FOUR_TIME
                JMP	NO_FIRST_TIME
		
CHECK_MIN_2:
		CJNE	R0,#KEY_READ_BUFF+1,NO_FIRST_TIME_3
;###########################################################
;		时间小于第一时间段,要取第四个数据
;###########################################################		
		MOV	R2,#04
		JMP	NO_FIRST_TIME

NO_FIRST_TIME_3:		
		INC	R2
		JMP	NO_FIRST_TIME

proc_door_key_sw:		;门口按钮
		jbc	keyprese,door_key_sw_keyprese_proc
		clr	keyrelase
		ret
		
door_key_sw_keyprese_proc:
		MOV	A,keynumber
		ADD	A,#onetouch_BUF
		MOV	R0,A
		MOV	A,@R0
		JNB	ACC.0,door_key_sw_keyprese_OLD_PROC
;#####################################################################
;		门口按钮  开-->关
;#####################################################################		
		SETB	ACC.2
		CLR	ACC.0			;单键ON-OFF状态，OFF=xxxxxxx0，
		MOV	@R0,A			;置位发送位
		mov	wordadd1,#02h
		mov	subadr,#040h
		CALL	door_key_sw_SEND_PROC
;##############################################################################
;		处理403H 4位开关
;##############################################################################
		mov	SUBADR,#55H		;355H,门口按钮关
		CALL	OUT_403H
		ret
		RET
door_key_sw_keyprese_OLD_PROC:		
		JB	MASTER_OFF,door_key_sw_SP_PROC
;#############################################################
;		NORMAL PROC
;#############################################################		
door_key_sw_NOMAL_PROC:		
		CLR	A
		SETB	ACC.2			;置位发送位
		SETB	ACC.0			;单键ON-OFF状态，ON=xxxxxxx1，
		MOV	@R0,A			
		mov	wordadd1,#02h
		mov	subadr,#034h
		CALL	door_key_sw_SEND_PROC

;##############################################################################
;		处理403H 4位开关
;##############################################################################
		mov	SUBADR,#51H
		CALL	OUT_403H
		ret		
		
door_key_sw_SP_PROC:
		mov	wordadd1,#02h
		mov	subadr,#030h
		mov	pSMB_DATA_IN,#KEY_READ_BUFF	;门口按钮时间数据
		mov	bytecnt,#04
		call	rcvdata_EEPROM
		
		mov	CHIP_ADDRE,#CCR
		mov	wordadd1,#00h
		mov	subadr,#001h
		mov	pSMB_DATA_IN,#RTC_BUF		;当前时间数据
		mov	bytecnt,#02
		call	rcvdata
		
		MOV	A,KEY_READ_BUFF
		CJNE	A,RTC_BUF+1,door_key_sw_COMP_1
		JMP	door_key_sw_COMP_MIN
door_key_sw_COMP_1:		
		JC	door_key_sw_NEW_PROC
		JMP	door_key_sw_CHECK_END_PROC
		RET
;####################	小时时间数据相等  ######################	
door_key_sw_COMP_MIN:		
		MOV	A,RTC_BUF
		CJNE	A,KEY_READ_BUFF+1,door_key_sw_COMP_2
door_key_sw_COMP_2:		
		JNC	door_key_sw_NEW_PROC
                JMP     door_key_sw_NOMAL_PROC		;分钟时间数据相等
;#########################################################################
;
;#########################################################################				
door_key_sw_NEW_PROC:			
		mov	wordadd1,#02h
		mov	subadr,#03ah
		CALL	door_key_sw_SEND_PROC
		MOV	A,keynumber
		ADD	A,#onetouch_BUF
		MOV	R0,A
		CLR	A
		SETB	ACC.2
		SETB	ACC.0
		MOV	@R0,A			;置位发送位
;##############################################################################
;		处理403H 4位开关
;##############################################################################
		mov	SUBADR,#52H
		CALL	OUT_403H
		ret


door_key_sw_SEND_PROC:		
		mov	pSMB_DATA_IN,#KEY_READ_BUFF	;门口按钮数据
		mov	bytecnt,#06
		call	rcvdata_EEPROM
		
		MOV	A,keynumber
		MOV	B,#07H
		MUL	AB
		ADD	A,#key_tran_buf
		MOV	R0,A
		MOV	A,#0F5H			;1
		MOVX	@R0,A
		INC	R0
		MOV	A,KEY_READ_BUFF+1	;AREA
		MOVX	@R0,A
		INC	R0
		MOV	A,KEY_READ_BUFF+3	;JOIN
		JNZ	door_key_sw_SEND_PROC_1
		MOV	A,KEY_READ_BUFF+2
door_key_sw_SEND_PROC_1:		
		MOVX	@R0,A
		INC	R0
		MOV	A,#0FEH			;COMMAND
		MOVX	@R0,A
		INC	R0
		MOV	A,KEY_READ_BUFF
		MOVX	@R0,A			;SCENE
		INC	R0
		MOV	A,KEY_READ_BUFF+4	;FADE_L
		MOVX	@R0,A
		INC	R0
		MOV	A,KEY_READ_BUFF+5	;FADE_H
		MOVX	@R0,A
		RET
		
door_key_sw_CHECK_END_PROC:		
		mov	wordadd1,#02h
		mov	subadr,#032h
		mov	pSMB_DATA_IN,#KEY_READ_BUFF	;门口按钮终止时间数据
		mov	bytecnt,#04
		call	rcvdata_EEPROM
		
		MOV	A,RTC_BUF+1
		CJNE	A,KEY_READ_BUFF,door_key_sw_COMP_END_1
		JMP	door_key_sw_COMP_END_MIN
door_key_sw_COMP_END_1:		
		JC	door_key_sw_NEW_PROC
		JMP	door_key_sw_NOMAL_PROC
		RET
;####################	小时时间数据相等  ######################	
door_key_sw_COMP_END_MIN:		
		MOV	A,KEY_READ_BUFF+1
		CJNE	A,RTC_BUF,door_key_sw_COMP_END_2
door_key_sw_COMP_END_2:		
		JNC	door_key_sw_NEW_PROC
                JMP     door_key_sw_NOMAL_PROC		;分钟时间数据相等
		RET

SOS_BEJENE_CODE:
		DB	0faH,DEVICE_CODE,04H,07FH,01h,03H,80H
SOS_END_CODE:
		DB	0faH,DEVICE_CODE,04H,07EH,01h,03H,80H
proc_sos_sw:		
		jnb	keyprese,sos_sw_release
		mov	dptr,#SOS_BEJENE_CODE
proc_sos_sw_1:		
		lcall	trcode
;######################################################################
;		RESET CODE NEED SEND MASTER SLAVE
;######################################################################	
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		ret
sos_sw_release:		
		mov	dptr,#SOS_END_CODE
		jmp	proc_sos_sw_1

NO_BREAK_IN_ON_CODE:
		DB	0faH,DEVICE_CODE,04H,07BH,01h,03H,80H;请勿打扰开始

NO_BREAK_IN_ON_END_CODE:		
		DB	0faH,DEVICE_CODE,04H,07AH,01h,03H,80H;请勿打扰结束

proc_NO_BREAK_IN_ON:			;请勿打扰		
		jnb	keyprese,NO_BREAK_IN_ON_release
		mov	dptr,#NO_BREAK_IN_ON_CODE
		
		lcall	trcode
		MOV	buf_tran+2,boxnumber
;######################################################################
;		NO_BREAK_IN CODE (ON) NEED SEND MASTER SLAVE
;######################################################################	
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		clr	tranready
		lcall	transbuf
		setb	tranbuffull_1
		clr	tranready_1
		lcall	transbuf_1	
		jb	tranbuffull_1,$
		JB	DISABLE_tran_1,$
;####################################################################
;		发送门铃通道开,物理通道
;####################################################################
		;MOV	BUF_TRAN1,#0FAH
		;MOV	BUF_TRAN1+1,#0E0H
		;MOV	BUF_TRAN1+2,#01
		;MOV	BUF_TRAN1+3,#0B7H				
		;MOV	BUF_TRAN1+4,#01
		;MOV	BUF_TRAN1+5,#0FEH
		;MOV	BUF_TRAN1+6,#5
		;CALL	transbuf_1
		setb	do_not_disturb_lamp
		clr	need_clear_lamp				
		ret
NO_BREAK_IN_ON_release:		
		mov	dptr,#NO_BREAK_IN_ON_END_CODE
		lcall	trcode
		MOV	buf_tran+2,boxnumber
;######################################################################
;		NO_BREAK_IN CODE (OFF) NEED SEND MASTER SLAVE 
;######################################################################	
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		clr	tranready
		lcall	transbuf
		setb	tranbuffull_1
		clr	tranready_1
		lcall	transbuf_1	
;####################################################################
;		发送门铃通道关,物理通道
;####################################################################
		jb	tranbuffull_1,$
		JB	DISABLE_tran_1,$
		;MOV	BUF_TRAN1,#0FAH
		;MOV	BUF_TRAN1+1,#0E0H
		;MOV	BUF_TRAN1+2,#01
		;MOV	BUF_TRAN1+3,#0B7H				
		;MOV	BUF_TRAN1+4,#01
		;MOV	BUF_TRAN1+5,#00H
		;MOV	BUF_TRAN1+6,#5
		;CALL	transbuf_1
		clr	do_not_disturb_lamp
		jnb	CLEAR_LAMP_ON,NO_BREAK_IN_ON_release_1
		setb	need_clear_lamp
		
		
NO_BREAK_IN_ON_release_1:	
		ret
		
		
		
NEED_CLEAR_ROOM_CODE:
		DB	0faH,DEVICE_CODE,04H,079H,01h,03H,80H

CLEAR_ROOM_END_CODE:		
		DB	0faH,DEVICE_CODE,04H,078H,01h,03H,80H
		
proc_CLEAR_SW:					;010 CLEAR key		
		jnb	keyprese,proc_CLEAR_SW_END
		JBC	CLEAR_LAMP_ON,CANCEL_NEED_CLEAR_ROOM
		SETB	CLEAR_LAMP_ON
		clr	do_not_disturb_lamp	
		mov	dptr,#NEED_CLEAR_ROOM_CODE
		lcall	trcode
;######################################################################
;		NEED_CLEAR_ROOM CODE NEED SEND MASTER SLAVE
;######################################################################	
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		clr	tranready
		lcall	transbuf
		setb	tranbuffull_1
		clr	tranready_1
		lcall	transbuf_1	
		jb	tranbuffull_1,$
		JB	DISABLE_tran_1,$
;####################################################################
;		发送CLEAR 灯通道开,物理通道
;####################################################################
		;MOV	BUF_TRAN1,#0FAH
		;MOV	BUF_TRAN1+1,#0E0H
		;MOV	BUF_TRAN1+2,#01
		;MOV	BUF_TRAN1+3,#0B7H				
		;MOV	BUF_TRAN1+4,#02
		;MOV	BUF_TRAN1+5,#0FEH
		;MOV	BUF_TRAN1+6,#5
		;CALL	transbuf_1
		setb	need_clear_lamp
proc_CLEAR_SW_END:						
		ret

CANCEL_NEED_CLEAR_ROOM:
		CALL	CLEAR_CLEAR_LAMP	;发送CLEAR 灯通道关,物理通道
		mov	dptr,#CLEAR_ROOM_END_CODE
		lcall	trcode
;######################################################################
;		NEED_CLEAR_ROOM CODE NEED SEND MASTER 
;######################################################################	
		clr	tranready
		call	transbuf
		RET		
		
BCD_HEX:
		MOV	R7,A
		ANL	A,#0FH
		MOV	R6,A
		MOV	A,R7
		ANL	A,#0F0H
		SWAP    A
		MOV	B,#10
		MUL	AB
		ADD	A,R6
		RET
;#######################################################################
;
;#######################################################################
AIR_PROC:
		ret
			;插牌开关有动作,有人进入.
;########################################################################################
;HYEN(P2.7)=0	自动控制,P2.6  P2.5  11	FAN STOP         P2.4 P2.3   11   STOP
;				     00 FAN LOW			     00   FAN 
;				     01 FAN MID			     01   COLD PROC
;				     10 FAN HIGH		     10   HOT  PROC
;########################################################################################		
		mov	slvadr,#HIGH(AIR_WATER_IS_HOT_EE)
		mov	subadr,#LOW(AIR_WATER_IS_HOT_EE)
		mov	pSMB_DATA_IN,#TEMP
		mov	bytecnt,#01
		call	rcvdata_EEPROM
		
		MOV	dptr,#MODE
		MOVX	A,@dptr	
		MOV	RTC_BUF+1,A		;SAVE MODE&FAN
		ANL	A,#0FH
		JNZ	AIR_PROC_END_ADD
		JMP	AIR_PROC_END
AIR_PROC_END_ADD:
		SETB	P2.6
		SETB	P2.5
		INC	DPTR
		MOVX	A,@dptr			;
		MOV	RTC_BUF+2,A		;SAVE FAN
;##############################################################################
;
;##############################################################################		
		MOV	A,RTC_BUF+2
		ANL	A,#0FH			;GET FAN SPEED
		CJNE	A,#01,PROC_HIGH_FAN
;##############################################################################
;		FAN SPEED LOW
;##############################################################################		
		CLR	P2.6
		CLR	P2.5
		JMP	PROC_MODE
PROC_HIGH_FAN:
		CJNE	A,#03,PROC_HIGH_MID
		SETB	P2.5
		CLR	P2.6
		JMP	PROC_MODE
PROC_HIGH_MID:
		CJNE	A,#02,PROC_MODE
		CLR	P2.5
		SETB	P2.6
PROC_MODE:		
;############################################################################
;		检查温度模式.
;
;############################################################################		
		MOV	A,RTC_BUF+1		;MODE&FAN
		ANL	A,#0FH
		CJNE	A,#02H,PROC_MODE_HOT
;############################################################################
;		冷模式检查当前温度和设定温度的差别
;############################################################################		
		MOV	dptr,#set_temperature
		MOVX	A,@dptr			;GET SETING temperature
		MOV	RTC_BUF,A
		MOV	A,true_temperature
		CLR	C
		SUBB	A,RTC_BUF
		JC	true_temperature_IS_COLD;房间温度<设定温度 不需要加冷
;#############################################################################
;		房间温度>设定温度 需要加冷
;#############################################################################		
		MOV	A,TEMP
		JB	ACC.0,TWO_TUBE_CONTROL_COLD	
		;SIGEL TUBE
;##########################################################################
;		SIGEL TUBE  ACC.1=0 HOT   ACC.1=1 COLD
;##########################################################################		
		CLR	P2.4
		CLR	P2.3
		JB	ACC.1,AIR_PROC_END_1	;当前是冷水.
		SETB	P2.4
		SETB	P2.3			;当前是热水.
		RET
TWO_TUBE_CONTROL_COLD:
;############################################################################
;		OPEN COLD TUBE 00
;############################################################################		
		CLR	P2.4
		CLR	P2.3
		RET
true_temperature_IS_COLD:
;#############################################################################
;		关水管,结束
;#############################################################################
		SETB	P2.4
		SETB	P2.3			;关水管
		MOV	dptr,#MODE
		MOV	A,#01
		MOVX	@DPTR,A	
		CALL	SET_NEW_MODE
		RET

PROC_MODE_HOT:		
		CJNE	A,#03H,AIR_PROC_END_1	;风模式,结束
;############################################################################
;		热模式检查当前温度和设定温度的差别
;############################################################################		
		MOV	dptr,#set_temperature
		MOVX	A,@dptr			;GET SETING temperature
		MOV	RTC_BUF,A
		MOV	A,true_temperature
		CLR	C
		SUBB	A,RTC_BUF
		JNC	true_temperature_IS_COLD;房间温度>=设定温度 不需要加热		
;##########################################################################
;		房间温度<设定温度 需要加热
;##########################################################################				
		MOV	A,TEMP
		JB	ACC.0,TWO_TUBE_CONTROL_HOT
;##########################################################################
;		SIGEL TUBE  ACC.1=0 HOT   ACC.1=1 COLD
;##########################################################################		
		CLR	P2.4
		CLR	P2.3
		JNB	ACC.1,AIR_PROC_END_1	;当前是热水.
		SETB	P2.4
		SETB	P2.3			;当前是冷水.
		RET
		
TWO_TUBE_CONTROL_HOT:
;############################################################################
;		OPEN HOT TUBE 01
;############################################################################		
		CLR	P2.4
		SETB	P2.3
		RET		
		
		
		
		
		


CHECK_AIR_FAN:
;###############################################################################
;		风模式,P2.4 P2.3= H
;###############################################################################		
		SETB	P2.4
		SETB	P2.3
		RET
		
		
		
		
AIR_PROC_END:		
		SETB	P2.3
		SETB	P2.4
		SETB	P2.5
		SETB	P2.6
AIR_PROC_END_1:		
		RET
all_area_off:
		db	0f5h,200,0ffh,0feh,0bh,64h,0
SEND_MAN_OUT_AND_DELAY_IS_END:		
		MOV	A,#00h
		mov     RELAY_OUT,a
                CALL	RELAY_OUTPUT
		mov	dptr,#all_area_off
		call	trcode
		mov	BUF_TRAN+2,#03fh		;除需要清洁灯
		CALL	COPY_BUF_TRAN_TO_BUF_TRAN1
		clr	tranready
		clr	tranbuffull
		setb	tranbuffull_1
		clr	tranready_1
		lcall	transbuf_1	
		jb	tranbuffull_1,$
		JB	DISABLE_tran_1,$
		
                setb    MASTER_MUTE
                call	transbuf
                jb	tranbuffull,$
		JB	DISABLE_tran,$		
		
;###################################################################################
;		GPan AIR_CONDITION
;###################################################################################		
		
		;MOV	BUF_TRAN,#0FAH
		;MOV	BUF_TRAN+1,#DEVICE_CODE
		;MOV	BUF_TRAN+2,boxnumber
		;MOV	BUF_TRAN+3,#8FH			;SEND AIR_CONDITION
		;CLR	A
		;MOV	DPTR,#MODE
		;MOVX	@DPTR,A
		;MOV	BUF_TRAN+4,#0
		;MOV	DPTR,#set_temperature
		;MOVX	A,@DPTR
		;MOV	BUF_TRAN+5,A			;设定温度
		;MOV	BUF_TRAN+6,true_temperature
		;CLR	RECEIVED_DATA_FROM_SLAVE
		;CLR	MASTER_MUTE
		;SETB	tranbuffull
		;CALL	AIR_PROC			;SEND	MARSTER  AIR PROC。
		;JB	DISABLE_tran,$
		;call	transbuf
		
		
		
		
		
		
		
		
			
SEND_MAN_OUT_AND_DELAY_IS_END_1:		
		
		RET



		
;########################################################################
;	本程序判别F5码的区域是否和主开关的区域相同,否:返回
;	是:判别是场景码?是:当场景是12时,MASTER_OFF=1(表示是关灯)ALL_ON_CHANNEL_CONTER=0
;			   其他场景 :MASTER_OFF=0(表示是开灯)
;		否:是关灯指令(FD)?是:MASTER_OFF=1(表示是关灯)ALL_ON_CHANNEL_CONTER=0
;					  否:是通道命令?(D6,DB,D5,D3)
;	是:ALL_ON_CHANNEL_CONTER 记数加一.
;	否:是通道命令?(DC)是:ALL_ON_CHANNEL_CONTER 记数减一.
;			  否:返回
;########################################################################		
		
		
CHECK_MAIN_SW_ON_OFF:		
		mov	a,BUF_RECE+1		;AREA
		CJNE	A,ALL_ON_AREA,SEND_MAN_OUT_AND_DELAY_IS_END_1	;RET
		MOV	A,BUF_RECE+3		;COMMAND
		CJNE	A,#0FEH,CHECK_MAIN_F5_FD
		MOV	A,BUF_RECE+4
		CLR	MASTER_OFF
		SETB	ALL_SCENE_ON_FLAG
		CJNE	A,#011,SEND_MAN_OUT_AND_DELAY_IS_END_1
		SETB	MASTER_OFF
		MOV	ALL_ON_CHANNEL_CONTER,#00
		CLR	ALL_SCENE_ON_FLAG
		RET
CHECK_MAIN_F5_FD:		
		CJNE	A,#0FDH,CHECK_MAIN_CHANNEL_F5_D6
;#################################################################
;		OFF CAMMAND
;#################################################################		
		SETB	MASTER_OFF
		MOV	ALL_ON_CHANNEL_CONTER,#00
		CLR	ALL_SCENE_ON_FLAG
		RET
CHECK_MAIN_CHANNEL_F5_D6:		
		CJNE	A,#0D6H,CHECK_MAIN_CHANNEL_F5_DB
;#################################################################
;		CHANNEL ON COMMAND
;#################################################################		
CHANNEL_ON_CAMMAND:		
		CLR	MASTER_OFF
		INC	ALL_ON_CHANNEL_CONTER
		RET
CHECK_MAIN_CHANNEL_F5_DB:		
		CJNE	A,#0DBH,CHECK_MAIN_CHANNEL_F5_D5
		JMP	CHANNEL_ON_CAMMAND
CHECK_MAIN_CHANNEL_F5_D5:		
		CJNE	A,#0D5H,CHECK_MAIN_CHANNEL_F5_D3
		JMP	CHANNEL_ON_CAMMAND
CHECK_MAIN_CHANNEL_F5_D3:
		CJNE	A,#0D3H,CHECK_MAIN_CHANNEL_F5_DC
		JMP	CHANNEL_ON_CAMMAND
CHECK_MAIN_CHANNEL_F5_DC:		
		CJNE	A,#0DCH,SEND_MAN_OUT_AND_DELAY_IS_END_1	;RET
;################################################################
;		OFF CAMMAND
;################################################################
		MOV	A,BUF_RECE+5
		INC	A
		JZ	SEND_MAN_OUT_AND_DELAY_IS_END_1		;RET(ALL CHANNEL
		MOV	A,ALL_ON_CHANNEL_CONTER	
		JZ	SEND_MAN_OUT_AND_DELAY_IS_END_1
		DEC	A
		MOV	ALL_ON_CHANNEL_CONTER,A
		JNZ	SEND_MAN_OUT_AND_DELAY_IS_END_1
;################################################################
;		ONE TOUCH ALL CHANNEL OFF
;################################################################
		JB	ALL_SCENE_ON_FLAG,SEND_MAN_OUT_AND_DELAY_IS_END_1
		SETB	MASTER_OFF
		RET
		
		
SET_NEW_MODE:
		MOV	BUF_TRAN,#0FAH
		MOV	BUF_TRAN+1,#DEVICE_CODE
		MOV	BUF_TRAN+2,boxnumber
		MOV	BUF_TRAN+3,#8FH			;SEND AIR_CONDITION
		MOV	DPTR,#MODE
		MOVX	A,@DPTR
		MOV	BUF_TRAN+4,A
		INC	DPTR
		MOVX	A,@DPTR
		SWAP	A
		ANL	A,#0F0H
		ORL	A,BUF_TRAN+4
		MOV	BUF_TRAN+4,A
		MOV	DPTR,#set_temperature
		MOVX	A,@DPTR
		MOV	BUF_TRAN+5,A			;设定温度
		MOV	BUF_TRAN+6,true_temperature
		CLR	RECEIVED_DATA_FROM_SLAVE
		CLR	MASTER_MUTE
		SETB	tranbuffull
		;CALL	AIR_PROC			;SEND	MARSTER  AIR PROC。
		JB	DISABLE_tran,$
		call	transbuf
		RET
;######################## 403H PROC #######################################

		
;###########################################################################
;		INPUT: ACC      OUTPUT  RELAY_N  BIT
;###########################################################################
RELAY_OUTPUT:
		;JB	RALAY_WAIT_ON_0,RELAY_OUTPUT_1
		JB	ACC.0,RELAY_OUTPUT_0_1
		CLR	RELAY_0
		JMP	RELAY_OUTPUT_1
RELAY_OUTPUT_0_1:
		SETB	RELAY_0
RELAY_OUTPUT_1:
		;JB	RALAY_WAIT_ON_1,RELAY_OUTPUT_2
		JB	ACC.1,RELAY_OUTPUT_1_1
		CLR	RELAY_1
		RET
RELAY_OUTPUT_1_1:
		SETB	RELAY_1
		RET
		
;################################################################
;		GET P0 DATA      得到外部V1，V2，I1，I2的信号
;				转换成原来的P0信号.
;				P0.0=V1 P0.1=V2
;				P0.4=I1 P0.5=I2
;				现在:P2.7=V1  P2.5=V2
;				     P2.6=I1  P2.4=I2
;		DATA SAVE TO R6
;		USE ACC,R6
;################################################################
GET_P0_DATA:
		MOV	A,P2
		ANL	A,#0F0H
		MOV	R6,A
AGAIN_READ_P0:
		CLR	A
		DJNZ	Acc,$
		MOV	A,P2
		ANL	A,#0F0H
		XRL	A,R6
		JNZ	GET_P0_DATA
		MOV	A,R6
		MOV	C,ACC.7
		MOV	ACC.0,C		;V1
		MOV	C,ACC.5
		MOV	ACC.1,C		;V2
		MOV	C,ACC.4
		MOV	ACC.5,C		;I2
		MOV	C,ACC.6
		MOV	ACC.4,C		;I1
		ANL	A,#00110011B
		MOV	R6,A
		RET

PROC_RELAY_OUTPUT:
		MOV	keyprocpoint,#02H
		MOV	R7,#2
KEY_PROC:
		MOV	R5,keyprocpoint
		MOV	A,RELAY_OUT
		ANL	A,R5
		JNZ	KEY_PROC_1
		CALL	PROC_CH_ZERO
		JMP	KEY_PROC_NEXT
KEY_PROC_1:
		CALL	PROC_CH_ONE
KEY_PROC_NEXT:
		MOV	A,keyprocpoint
		RR	A
		MOV	keyprocpoint,A
		DJNZ	R7,KEY_PROC
		RET
		
;####################################################################################
;		RELAY IS ACTION
;		继电器吸合，应该有电流，没有电压
;####################################################################################
PROC_CH_ONE:
		CALL	GET_P0_DATA
		MOV	A,R6
		MOV	P0BUFFER,A		;SAVE DATA
		XRL	A,P0mirror
		JZ	P0_DATA_IS_SAME
		SWAP	A
		ANL	A,R5
		JZ	P0_DATA_IS_SAME
;##################################################################
;		CURRENT IS CHANGE
;##################################################################
		MOV	P0mirror,P0BUFFER
		RET
P0_DATA_IS_SAME:
		MOV	A,P0mirror
		SWAP	A			;电流是高半字节
		ANL	A,R5
		JZ	CHANNEL_IS_ON
		MOV	A,RALAY_OFF_TO_ON__CHANGE
		ANL	A,R5
		JZ	RELAY_IS_ON_BUT_I_ZERO
;##################################################################
;		继电器已吸合，人手没有释放开关，电流没有,需要判有电压？
;		有电压，人手已释放开关，RALAY_WAIT_ON可以清零
;		没有电压，人手没有释放开关
;##################################################################

		;MOV	A,P0mirror
		;ANL	A,R5
		;JNZ	P0_DATA_IS_SAME_END
		;MOV	A,R5
	;	CPL	A
	;	ANL	RALAY_WAIT_ON,A

;P0_DATA_IS_SAME_END:		
		RET

RELAY_IS_ON_BUT_I_ZERO:
;##################################################################
;		RELAY IS ON BUT CURRENT=0
;		OUT SWITCH IS BREAKEN
;##################################################################
		MOV	A,#key1_OFF_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	START_key1_OFF_delay
;##############################################################
;		正在断电延时.
;##############################################################
		DEC	A
		MOV	@R0,A
		JNZ	GET_CURRENT_END			;RET
;##############################################################
;		断电延时时间到.
;##############################################################
		MOV	A,R5
		CPL	A
		MOV	R2,A
		ANL	RELAY_OUT,A
		MOV	A,R5
		ORL	RALAY_ON_TO_OFF_IS_CHANGE,A
                ;CPL     A
                ;ANL	RALAY_OFF_TO_ON__CHANGE ,A
	
		RET
START_key1_OFF_delay:
		MOV	A,#key1_ON_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	RELAY_ON_LONG_TIME_OUTSWITCH_IS_BREAKE
		MOV	@R0,#00H		;继电器刚吸合，去抖动。
		RET
RELAY_ON_LONG_TIME_OUTSWITCH_IS_BREAKE:
                MOV	A,#key1_OFF_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	@R0,#DELAY_TIME
		RET
CHANNEL_IS_ON:
;########################################################################
;		继电器已吸合有电流
;########################################################################
		MOV	A,RALAY_OFF_TO_ON__CHANGE
		ANL	A,R5
		JZ	CHANNEL_IS_ON_LONG
		MOV	A,#key1_ON_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	CHANNEL_IS_ON_START
		DEC	A
		MOV	@R0,A
		JNZ	GET_CURRENT_END
		MOV	A,R5
		CPL	A
		ANL	RALAY_OFF_TO_ON__CHANGE,A
		RET
CHANNEL_IS_ON_START:
		MOV	@R0,#DELAY_TIME
		RET
CHANNEL_IS_ON_LONG:
		MOV	A,#key1_OFF_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	GET_CURRENT_END		;NO  TIME DELAY
		MOV	@R0,#00H
GET_CURRENT_END:
                ;MOV	A,R5
		;CPL	A
		;ANL	RALAY_ON_TO_OFF_IS_CHANGE,A
                RET


;#####################################################################
;		继电器释放，应该有电压，没有电流
;#####################################################################

PROC_CH_ZERO:
		CALL	GET_P0_DATA
		MOV	A,R6
		MOV	P0BUFFER,A		;SAVE DATA
		XRL	A,P0mirror
		JZ	P0_DATA_IS_SAME_ZERO
		ANL	A,R5
		JZ	P0_DATA_IS_SAME_ZERO
;##################################################################
;		voltage IS CHANGE
;##################################################################
		MOV	P0mirror,P0BUFFER
		RET
P0_DATA_IS_SAME_ZERO:
		MOV	A,P0mirror
		ANL	A,R5
		JZ	CHANNEL_IS_OFF
;##################################################################
;		继电器已释放,电压没有，没有电流
;##################################################################
		MOV	A,RALAY_ON_TO_OFF_IS_CHANGE
		ANL	A,R5
		JZ	RELAY_IS_OFF_BUT_voltage_ZERO
;##################################################################
;		继电器已释放，人手没有释放开关，电压没有，没有电流
;##################################################################
		RET

RELAY_IS_OFF_BUT_voltage_ZERO:
;##################################################################
;		RELAY IS OFF BUT voltage=0
;		OUT SWITCH IS BREAKEN NEXT NEED RELAY_ON
;##################################################################
		MOV	A,#key1_ON_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	START_key1_ON_delay
;##############################################################
;		正在 OUT SWITCH 状态延时.
;##############################################################
		DEC	A
		MOV	@R0,A
		JNZ	GET_CURRENT_END			;RET
;##############################################################
;		断电延时时间到.
;##############################################################
		MOV	A,R5
		MOV	R2,A
		ORL	RELAY_OUT,A
		MOV	A,R5
		ORL	RALAY_OFF_TO_ON__CHANGE,A
		;ORL	RALAY_WAIT_ON,A
                ;CPL     A
                ;ANL     RALAY_ON_TO_OFF_IS_CHANGE,A
		RET
START_key1_ON_delay:
		MOV	@R0,#DELAY_TIME
		RET

CHANNEL_IS_OFF:
;########################################################################
;		继电器已释放,有电压 ，没有电流
;########################################################################
		MOV	A,RALAY_ON_TO_OFF_IS_CHANGE
		ANL	a,R5
		JZ	CHANNEL_IS_OFF_LONG
		MOV	A,#key1_OFF_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	CHANNEL_IS_OFF_START
		DEC	A
		MOV	@R0,A
		JNZ	GET_voltage_END
		MOV	A,R5
		CPL	A
		ANL	RALAY_ON_TO_OFF_IS_CHANGE,A
		RET
CHANNEL_IS_OFF_START:
                MOV	A,#key1_OFF_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	@R0,#DELAY_TIME
		RET
CHANNEL_IS_OFF_LONG:
		MOV	A,#key1_ON_delay-1
		ADD	A,R7
		MOV	R0,A
		MOV	A,@R0
		JZ	GET_voltage_END		;NO  TIME DELAY
		MOV	@R0,#00H
GET_voltage_END:


		RET


											
end				

