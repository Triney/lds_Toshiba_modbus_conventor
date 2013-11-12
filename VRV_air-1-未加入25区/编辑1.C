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