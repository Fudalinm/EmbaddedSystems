
.equ CPU_CLOCK = 11059200

.equ POWER_PWM = 130 ;200 ; 255 means minimum PWM, 0 means maximum PWM
.equ BUTTON_TIME = 216              ; 216 *  46.3 us = 10ms
/*
Phase   123456
      R --0++0
      S +0--0+
      T 0++0--
;---------+---+---+---+---+---+---+
;  Phase  | 1 | 2 | 3 | 4 | 5 | 6 |
;---------+---+---+---+---+---+---+
;     R   | - | - | 0 | + | + | 0 |
;---------+---+---+---+---+---+---+
;     S   | + | 0 | - | - | 0 | + |
;---------+---+---+---+---+---+---+
;     T   | 0 | + | + | 0 | - | - |
;---------+---+---+---+---+---+---+
*/
/*
 PD3 - OC2B - TL - t
 PD5 - OC0B - RH - R
 PD6 - OC0A - RL - r
 PB1 - OC1A - SH - S
 PB2 - OC1B - SL - s
 PB3 - OC2A - TH - T
*/ 

; -----------------------------------------------------------------------------
; Phase Init
; R = -
; S = +
; T = -
;                  ----TsS-
.equ PH_INIT_B = 0b00001000
;                  -rR-t---
.equ PH_INIT_D = 0b01101000

; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Phase 1
; R = -
; S = +
; T = 0
;              ----TsS-
.equ PH1_B = 0b00001000
;              -rR-t---
.equ PH1_D = 0b01100000
; -----------------------------------------------------------------------------
; Phase 2
; R = -
; S = 0
; T = +
;              ----TsS-
.equ PH2_B = 0b00000010
;              -rR-t---
.equ PH2_D = 0b01100000
; -----------------------------------------------------------------------------
; Phase 3
; R = 0
; S = -
; T = +
;              ----TsS-
.equ PH3_B = 0b00000110
;              -rR-t---
.equ PH3_D = 0b00100000
; -----------------------------------------------------------------------------
; Phase 4
; R = +
; S = -
; T = 0
;              ----TsS-
.equ PH4_B = 0b00001110
;              -rR-t---
.equ PH4_D = 0b00000000
; -----------------------------------------------------------------------------
; Phase 5
; R = +
; S = 0
; T = -
;              ----TsS-
.equ PH5_B = 0b00001010
;              -rR-t---
.equ PH5_D = 0b00001000
; -----------------------------------------------------------------------------
; Phase 6
; R = 0
; S = +
; T = -
;              ----TsS-
.equ PH6_B = 0b00001000
;              -rR-t---
.equ PH6_D = 0b00101000

; -----------------------------------------------------------------------------
; -----------------------------------------------------------------------------

; --- INPUT BUTTON ---
.equ PORT_BUTTON = PORTD
.equ DDR_BUTTON  = DDRD
.equ PIN_BUTTON  = PIND
.equ IN_BUTTON   = PIND4


; --- OUTPUT RL ---
.equ PORT_OUT_RL = PORTD
.equ DDR_OUT_RL  = DDRD
.equ PIN_OUT_RL  = PIND
.equ IN_OUT_RL   = PIND6

; --- OUTPUT RH ---
.equ PORT_OUT_RH = PORTD
.equ DDR_OUT_RH  = DDRD
.equ PIN_OUT_RH  = PIND
.equ IN_OUT_RH   = PIND5

; --- OUTPUT SL ---
.equ PORT_OUT_SL = PORTB
.equ DDR_OUT_SL  = DDRB
.equ PIN_OUT_SL  = PINB
.equ IN_OUT_SL   = PINB2

; --- OUTPUT SH ---
.equ PORT_OUT_SH = PORTB
.equ DDR_OUT_SH  = DDRB
.equ PIN_OUT_SH  = PINB
.equ IN_OUT_SH   = PINB1

; --- OUTPUT TL ---
.equ PORT_OUT_TL = PORTD
.equ DDR_OUT_TL  = DDRD
.equ PIN_OUT_TL  = PIND
.equ IN_OUT_TL   = PIND3

; --- OUTPUT TH ---
.equ PORT_OUT_TH = PORTB
.equ DDR_OUT_TH  = DDRB
.equ PIN_OUT_TH  = PINB
.equ IN_OUT_TH   = PINB3


; --- OUTPUT TEST1 ---
.equ PORT_TEST1= PORTC
.equ DDR_TEST1 = DDRC
.equ PIN_TEST1 = PINC
.equ IN_TEST1  = PINC5

; --- OUTPUT TEST2 ---
.equ PORT_TEST2= PORTC
.equ DDR_TEST2 = DDRC
.equ PIN_TEST2 = PINC
.equ IN_TEST2  = PINC4

; --- HALL INPUT ---
.equ PORT_HALL = PORTD
.equ DDR_HALL  = DDRD
.equ PIN_HALL  = PIND
.equ IN_HALL   = PIND0


;******************************************************************************
; Register Definitions
;
.def productL  =     R0         ; reserved, uses by command mul /Product LOW
.def productH  =     R1         ; reserved, uses by command mul /Product HIGH
.def zero      =     R2
.def one       =     R3
.def ff        =     R4
;.def = R5
;.def = R6
;.def = R7
.def sreg_save =     R8         ; flags register save for irq
;.def = R9
;.def = R10
;.def = R11
;.def = R12
;.def = R13
.def pwm       =     R14
;.def  =     R15

.def param     =     R16
.def param1    =     R17
.def param2    =     R18
.def param3    =     R19

.def time_l    =     R20
.def time_h    =     R21

.def cycles_cnt=     R22

;def    =     R23

.def flags    =      R24
 .equ F_STABLE         = 0
 .equ F_TIMEOUT        = 1
 ;.equ F_STILL_AT_START = 2


;.def = R25
 
;.def	XL	= r26		; used by Timer IRQ
;.def	XH	= r27		; used by Timer IRQ
;.def	YL	= r28		; Y pointer low
;.def	YH	= r29		; Y pointer high
;.def	ZL	= r30		; Z pointer low
;.def	ZH	= r31		; Z pointer high


;******************************************************************************
;******************************************************************************
.cseg
.org  $0000                 ; RESET External Pin, Power-on Reset, Brown-out Reset, Watchdog Reset, and JTAG AVR Reset
        rjmp   _reset_		; Reset handler 0000

.org  $0001                 ; INT0 External Interrupt Request 0
        rjmp    _unused_

.org  $0002                 ; INT1 External Interrupt Request 1
        rjmp    _unused_

.org  $0003
        rjmp    _unused_	; PCINT0 Pin Change Interrupt Request 0

.org  $0004
        rjmp    _unused_	; PCINT1 Pin Change Interrupt Request 1

.org  $0005
        rjmp    PcInt   	; PCINT2 Pin Change Interrupt Request 2

.org  $0006
        rjmp    _unused_	; WDT Watchdog Time-out Interrupt

.org  $0007                 ; TIMER2 COMPA Timer/Counter2 Compare Match
        rjmp    _unused_    ; unused interrupt

.org  $0008                 ; TIMER2 COMPA Timer/Counter2 Compare Match
        rjmp    _unused_    ; unused interrupt

.org  $0009                 ; TIMER2 OVF Timer/Counter2 Overflow
        rjmp    _unused_

.org  $000A                 ; TIMER1 CAPT Timer/Counter1 Capture Event
        rjmp    _unused_    ;

.org  $000B                 ; TIMER1 COMPA Timer/Counter1 Compare Match A
        rjmp    _unused_    ; unused interrupt

.org  $000C                 ; TIMER1 COMPB Timer/Coutner1 Compare Match B
        rjmp    _unused_    ; unused interrupt

.org  $000D                 ; TIMER1 OVF Timer/Counter1 Overflow
        rjmp    Timer1Ov

.org  $000E                 ; TIMER0 COMPA Timer/Counter0 Compare Match A
        rjmp    _unused_

.org  $000F                 ; TIMER0 COMPB Timer/Counter0 Compare Match B
        rjmp    _unused_    ; unused interrupt

.org  $0010                 ; TIMER0 OVF Timer/Counter0 Overflow
        rjmp    _unused_    ; unused interrupt

.org  $0011                 ; SPI, STC SPI Serial Transfer Complete
        rjmp    _unused_    ; unused interrupt

.org  $0012                 ; USART, RX USART Rx Complete
        rjmp    _unused_

.org  $0013                 ; USART, UDRE USART, Data Register Empty
        rjmp    _unused_    ; unused interrupt

.org  $0014                 ; USART, TX USART, Tx Complete
        rjmp    _unused_

.org  $0015                 ; ADC ADC Conversion Complete
        rjmp    _unused_     ; unused interrupt

.org  $0016                 ; EE READY EEPROM Ready
        rjmp    _unused_    ; unused interrupt

.org  $0017                 ; ANALOG COMP Analog Comparator
        rjmp    _unused_    ; unused interrupt

.org  $0018                 ; TWI 2-wire Serial Interface
        rjmp    _unused_    ; unused interrupt

.org  $0019                 ; SPM READY Store Program Memory Ready
        rjmp    _unused_    ; unused interrupt


;------------------------------------------------------------------------------
_unused_:

    reti

;------------------------------------------------------------------------------
; IRQ procedure
Timer1Ov:
    in      sreg_save,SREG

    sbiw    XH:XL,1                 ; decrement time 
    brne    Timer1OvSkip        
    ; tick
    sbr     flags,(1<<F_TIMEOUT)
    movw    XH:XL,time_h:time_l     ; reload timer (2 Bytes)
Timer1OvSkip:

    out     SREG,sreg_save

    reti
;------------------------------------------------------------------------------ 
; Hall sensor irq - for test only
PcInt:
    sbic    PIN_HALL,IN_HALL
    cbi     PORT_TEST2,IN_TEST2   ; Hall inactive
    sbis    PIN_HALL,IN_HALL
    sbi     PORT_TEST2,IN_TEST2   ; Hall active

    reti
;------------------------------------------------------------------------------ 
;******************************************************************************
;------------------------------------------------------------------------------ 
_reset_:



    ; Init Stack
    ldi     param, LOW(RAMEND)  ; LOW-Byte of upper RAM-Adress
    out     SPL, param
    ldi     param, HIGH(RAMEND) ; HIGH-Byte of upper RAM-Adress
    out     SPH, param

    ; clear RAM - delay for power stabilisation
    ldi     YL,LOW(0x100)
    ldi     YH,HIGH(0x100)
    ldi     param,LOW(0x2ff)
    ldi     param1,HIGH(0x2ff)
ClrRamLoop:
    st      Y+,zero
    cp      param,YL
    cpc     param1,YH
    brne    ClrRamLoop   

    ; init variables
    clr     flags
    clr     zero
    ldi     param,0xFF
    mov     ff,param

    ; timer will be used for checking button 
    ldi     time_l, LOW(BUTTON_TIME)
    ldi     time_h, HIGH(BUTTON_TIME)
    mov     XH,time_h
    mov     XL,time_l

    ; init hardware
    rcall   InitHdw   

    sei

    ldi     param, (1<<SE)    ; enable sleep (idle) mode
    sts     SMCR,param

    ; For test only !
    ; rcall  CurrentTest ;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

;------------------------
    ; wait for start button


main_wait_push_button:
    rcall   WaitForTimer
    sbic    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_wait_push_button

main_wait_release_button:
    rcall   WaitForTimer
    sbis    PIN_BUTTON,IN_BUTTON    ; 1 deactivated
    rjmp    main_wait_release_button

    cbr     flags,(1<<F_STABLE)     ; set flags to inital values


    ; set starting position  
    sbi     DDR_OUT_RL,IN_OUT_RL    ; R = -     OUTPUT means PWM
    cbi     PORT_OUT_SH,IN_OUT_SH   ; S = +     LOW means on 
    sbi     DDR_OUT_TL,IN_OUT_TL    ; T = -     OUTPUT means PWM


    sbi     PORT_TEST1,IN_TEST1   ; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    ldi     ZH,HIGH(TimeTable<<1)   ; pointer to TimeTable
    ldi     ZL,LOW(TimeTable<<1)
main_start_loop:
    lpm     time_l, Z+
    lpm     time_h, Z+
    lpm     cycles_cnt,Z+   ; cycles
    lpm     pwm,Z+          ; PWM
    rcall   SetPwms
    mov     XH,time_h
    mov     XL,time_l

    cpi     cycles_cnt,255
    brne    main_start_loop_end 
    ; it is not the start cycle
    rcall   WaitForTimer
    rjmp    main_start_loop

main_start_loop_end:
    cbi     PORT_TEST1,IN_TEST1   ; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    cbi     DDR_OUT_RL,IN_OUT_RL    ; R = 0 - should be te same as Phase 6

main_start1_loop:
    cpi     cycles_cnt,254
    brne    main_start1_loop_end 
    rcall   WaitForTimer

    lpm     time_l, Z+
    lpm     time_h, Z+
    lpm     cycles_cnt,Z+   ; cycles
    lpm     pwm,Z+          ; PWM
    rcall   SetPwms
    mov     XH,time_h
    mov     XL,time_l

    rjmp    main_start1_loop

main_start1_loop_end:

    sbi     PORT_TEST1,IN_TEST1   ; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;---------+---+---+---+---+---+---+
;  Phase  | 1 | 2 | 3 | 4 | 5 | 6 |
;---------+---+---+---+---+---+---+
;     R   | - | - | 0 | + | + | 0 |
;---------+---+---+---+---+---+---+
;     S   | + | 0 | - | - | 0 | + |
;---------+---+---+---+---+---+---+
;     T   | 0 | + | + | 0 | - | - |
;---------+---+---+---+---+---+---+

main_loop:
    rcall   WaitAndSetPhase1

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor
    rcall   WaitAndSetPhase2

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor
    rcall   WaitAndSetPhase3

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor
    rcall  WaitAndSetPhase4

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor
    rcall   WaitAndSetPhase5

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor
    rcall   WaitAndSetPhase6

    sbis    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    main_stop               ; go to stop

    rcall   SetMotor


    rjmp    main_loop       

main_stop:
    ; stop motor
    rcall   WaitForTimer

    rcall   OutputsBrake

    ; timer will be used for checking button 
    ldi     time_l, LOW(BUTTON_TIME)
    ldi     time_h, HIGH(BUTTON_TIME)
    mov     XH,time_h
    mov     XL,time_l

main_stop_wait_release:
    rcall   WaitForTimer

    sbis    PIN_BUTTON,IN_BUTTON    ; 1 deactivated
    rjmp    main_stop_wait_release

    rcall   OutputsOff

    rcall   WaitForTimer

    rjmp    main_wait_push_button

main_end_loop:
    nop
    rjmp    main_end_loop


;------------------------------------------------------------------------------
WaitAndSetPhase1:

    rcall   WaitForTimer
    ; PHASE 1
    ; R = -
    ; S = +
    ; T = 0
    ; don't chage RH - was R = 0 
    sbi     DDR_OUT_RL,IN_OUT_RL    ; R = -     OUTPUT means PWM
    ; don't chage, was  S = +       ; S = +
    ; don't chage TL - was T = - 
    cbi     DDR_OUT_TL,IN_OUT_TL    ; T = 0     INPUT means off

    ret
;------------------------------------------------------------------------------
WaitAndSetPhase2:
    rcall   WaitForTimer
    ; PAHSE 2
    ; R = -
    ; S = 0
    ; T = +
    ; don't chage, was R = -        ; R = -
    sbi     PORT_OUT_SH,IN_OUT_SH   ; S = 0     HI means off
    ; don't chage SL - was S = + 
    cbi     PORT_OUT_TH,IN_OUT_TH   ; T = +     LOW means on    
    ; don't chage TL - was T = 0 

    ret

;------------------------------------------------------------------------------
WaitAndSetPhase3:

    rcall   WaitForTimer
    ; PAHSE 3
    ; R = 0
    ; S = -
    ; T = +
    ; don't chage RH - was R = - 
    cbi     DDR_OUT_RL,IN_OUT_RL    ; R = 0     INPUT means off
    ; don't chage SH - was S = 0
    sbi     DDR_OUT_SL,IN_OUT_SL    ; S = -     OUTPUT means PWM
    ; don't chage, was  T = +       ; T = +

    ret

;------------------------------------------------------------------------------
WaitAndSetPhase4:

    rcall   WaitForTimer

    ; PAHSE 4
    ; R = +
    ; S = -
    ; T = 0
    cbi     PORT_OUT_RH,IN_OUT_RH   ; R = +     LOW means on
    ; don't chage RL - was R = 0
    ; don't chage, was S = -        ; S = -
    sbi     PORT_OUT_TH,IN_OUT_TH   ; T = 0     HI means off

    ret

;------------------------------------------------------------------------------
WaitAndSetPhase5:

    rcall   WaitForTimer

    ; PAHSE 5
    ; R = +
    ; S = 0
    ; T = -
    ; don't chage, was R = +        ; R = +
    ; don't chage SH - was S = -
    cbi     DDR_OUT_SL,IN_OUT_SL    ; S = 0     INPUT means off
    ; don't chage TH - was T = 0
    sbi     DDR_OUT_TL,IN_OUT_TL    ; T = -     OUTPUT means PWM

    ret

;------------------------------------------------------------------------------
WaitAndSetPhase6:

    sbi     PORT_TEST1,IN_TEST1   ; 

    rcall   WaitForTimer

    cbi     PORT_TEST1,IN_TEST1   ; 

    ; PAHSE 6
    ; R = 0
    ; S = +
    ; T = -
    sbi     PORT_OUT_RH,IN_OUT_RH   ; R = 0     HI means off
    ; don't chage RL - was R = +
    cbi     PORT_OUT_SH,IN_OUT_SH   ; S = +     LOW means on
    ; don't chage SL - was S = 0
    ; don't chage, was T = +        ; T = -

    ret

;------------------------------------------------------------------------------
SetMotor:

    sbrc    flags,F_STABLE
    rjmp    SetMotorEnd     ; speed is stable

    tst     cycles_cnt
    brne    SetMotorSkipReading
    ; cycles_cnt == 0, read next motor data
    lpm     time_l,Z+       ; LSB Time
    lpm     time_h,Z+       ; MSB Time
    lpm     cycles_cnt,Z+   ; cycles
    lpm     pwm,Z+          ; PWM
    rcall   SetPwms
    mov     XH,time_h
    mov     XL,time_l
SetMotorSkipReading:

    tst     cycles_cnt
    brne    SetMotorNotStable
    ; cycles_cnt == 0 - stable
    sbr     flags,(1<<F_STABLE)
    rjmp    SetMotorEnd     ; speed is stable
SetMotorNotStable:

    ; send time via UART for test only
    sts     UDR0,time_h      ; send Time MSB thru UART
SetMotorWaitUart:
    lds     param,UCSR0A
    sbrs    param,UDRE0
    rjmp    SetMotorWaitUart
    sts     UDR0,time_l      ; send Time LSB thru UART


    dec     cycles_cnt

SetMotorEnd:

    ret                    

;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
InitHdw:
    ; --- init hardware ---

    ; set xL outputs - all as input, OFF (inactive)
    ; xL control by changing port direction
    cbi     PORT_OUT_RL,IN_OUT_RL
    cbi     DDR_OUT_RL,IN_OUT_RL

    cbi     PORT_OUT_SL,IN_OUT_SL
    cbi     DDR_OUT_SL,IN_OUT_SL

    cbi     PORT_OUT_TL,IN_OUT_TL
    cbi     DDR_OUT_TL,IN_OUT_TL

    ; set xH outputs - all OFF (inactive)
    sbi     PORT_OUT_RH,IN_OUT_RH
    sbi     DDR_OUT_RH,IN_OUT_RH    ; as output

    sbi     PORT_OUT_SH,IN_OUT_SH
    sbi     DDR_OUT_SH,IN_OUT_SH

    sbi     PORT_OUT_TH,IN_OUT_TH
    sbi     DDR_OUT_TH,IN_OUT_TH

    ; Test pin outputs
    cbi     PORT_TEST1,IN_TEST1
    sbi     DDR_TEST1,IN_TEST1

    cbi     PORT_TEST2,IN_TEST2
    sbi     DDR_TEST2,IN_TEST2

    ; Hall sensor input
    cbi     DDR_HALL,IN_HALL
    sbi     PORT_HALL,IN_HALL       ; enable  pull-up



    ; ------------------------------------- TIMERS
    ; Set all timers in "Phase correct mode". Do not enable outputs yet.
    ; for all: prescaler 1, PWM, phase correct 8-bit
    ; Timer0:  OC0A (pin PD6) controls RL (r) transistor
    out     OCR0A,ff
    ldi     param,(1 << WGM00) | (1<<COM0A1) | (1<<COM0A0)
    out     TCCR0A,param
    ; Timer1: OC1B (pin PB2) controls SL (s) transistor
    ldi     param,0x00
    sts     ICR1H,param
    ldi     param,0xFF
    sts     ICR1L,param     ; Set top value for Timer1
    ldi     param,(1 << WGM11) | (1 << COM1B1) | (1 << COM1B0)
    sts     TCCR1A,param
    ; Timer1: OC2B (pin PD3) controls TL (t) transistor
    sts     OCR2A,ff
    ldi     param,(1 << WGM20) | (1 << COM2B1) | (1 << COM2B0) 
    sts     TCCR2A,param


    ; Synchronize timers
    ldi     param,0
    out     TCNT0,param
    sts     TCNT1H,param
    ldi     param,2    
    sts     TCNT1L,param
    ldi     param,4
    sts     TCNT2,param

    ; Start all 3 timers
    ldi     param,(0 << CS01) | (1 << CS00)
    ldi     param1,(1 << WGM13) | (0 << CS11) | (1 << CS10)
    ldi     param2,(0 << CS21) | (1 << CS20)
    out     TCCR0B,param
    sts     TCCR1B,param1
    sts     TCCR2B,param2

    ldi     param, POWER_PWM
    mov     pwm,param
    rcall   SetPwms

    ; ---------------------------------------- UART for debug
    sts     UBRR0H, zero
    ldi     param,8     ; 115200 bps @ 16MHz
    sts     UBRR0L, param
    ; Enable transmitter only
    ldi     param, (1<<TXEN0)
    sts     UCSR0B,param
    ; Set frame format: 8 data, 1 stop bit
    ldi     param, (3<<UCSZ00)
    sts     UCSR0C, param

    ; ---------------------------------------- Interrupts
    ldi     param,(1<<TOV1)     ; enable interrupt
    sts     TIMSK1,param        ; interrupt every  46.3 us, 11059200 Hz/(2*256) = 21600 Hz

    ldi     param,(1<<PCINT16)
    sts     PCMSK2,param        ; enable interrupt on Test Pin
    ldi     param,(1<<PCIE2)
    sts     PCICR,param
    


    ret


;------------------------------------------------------------------------------
SetPwms:
    out     OCR0A,pwm       ; power for RL
    sts     OCR1BL,pwm      ; power for SL
    sts     OCR2B,pwm       ; poewr for TL

    ret

;------------------------------------------------------------------------------
OutputsBrake:

    cbi     PORT_OUT_RH,IN_OUT_RH   ; R = 0     LOW means on
    cbi     PORT_OUT_SH,IN_OUT_SH   ; S = 0     LOW means on
    cbi     PORT_OUT_TH,IN_OUT_TH   ; T = 0     LOW means on
    cbi     DDR_OUT_RL,IN_OUT_RL    ; R = 0     INPUT means off
    cbi     DDR_OUT_SL,IN_OUT_SL    ; S = 0     INPUT means off
    cbi     DDR_OUT_TL,IN_OUT_TL    ; T = 0     INPUT means off

    ret
       
;------------------------------------------------------------------------------
OutputsOff:

    sbi     PORT_OUT_RH,IN_OUT_RH   ; R = 0     HI means off
    sbi     PORT_OUT_SH,IN_OUT_SH   ; S = 0     HI means off
    sbi     PORT_OUT_TH,IN_OUT_TH   ; T = 0     HI means off
    cbi     DDR_OUT_RL,IN_OUT_RL    ; R = 0     INPUT means off
    cbi     DDR_OUT_SL,IN_OUT_SL    ; S = 0     INPUT means off
    cbi     DDR_OUT_TL,IN_OUT_TL    ; T = 0     INPUT means off

    ret
       
;------------------------------------------------------------------------------
   WaitForTimer:

    sbrs    flags,F_TIMEOUT
    rjmp    WaitForTimer

    cbr     flags,(1<<F_TIMEOUT)

    ret

   
;------------------------------------------------------------------------------
; For test only
;------------------------------------------------------------------------------
.equ CURRENT_TEST_TIMER = 10 ; 9 PWM cycles

CurrentTest:
    ldi     param,200
    out     OCR0A,param     ; power for RL

    ; disable all outputs
    rcall   OutputsOff

    ldi     time_l, LOW(BUTTON_TIME)
    ldi     time_h, HIGH(BUTTON_TIME)


CurrentTestWaitPress:
    rcall   WaitForTimer
    sbic    PIN_BUTTON,IN_BUTTON    ; 0 activated
    rjmp    CurrentTestWaitPress

CurrentTestWaitRelease:
    rcall   WaitForTimer
    sbis    PIN_BUTTON,IN_BUTTON    ; 1 deactivated
    rjmp    CurrentTestWaitRelease

    ldi     time_l, LOW(CURRENT_TEST_TIMER)
    ldi     time_h, HIGH(CURRENT_TEST_TIMER)
    rcall   WaitForTimer

    sbi     DDR_OUT_RL,IN_OUT_RL    ; R = -
    cbi     PORT_OUT_SH,IN_OUT_SH   ; S = +
    ; T = 0

    rcall   WaitForTimer            ; CURRENT_TEST_TIMER pulses

    rjmp    CurrentTest

    ret  
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
.include "StartTable.asm"
