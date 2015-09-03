


;******************************************************************************
;   This file is a basic code template for object module code                 *
;   generation on the PIC12F683. This file contains the                       *
;   basic code building blocks to build upon.                                 *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on             *
;   features of the assembler and linker (Document DS33014).                  *
;                                                                             *
;   Refer to the respective PIC data sheet for additional                     *
;   information on the instruction set.                                       *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:      xxx.asm                                                   *
;    Date:                                                                    *
;    File Version:                                                            *
;                                                                             *
;    Author:                                                                  *
;    Company:                                                                 *
;                                                                             *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Files required: P12F683.INC                                              *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Notes:                                                                   *
;                                                                             *
;******************************************************************************

;------------------------------------------------------------------------------
; PROCESSOR DECLARATION
;------------------------------------------------------------------------------

     LIST      P=12F683              ; list directive to define processor
     #INCLUDE <P12F683.INC>          ; processor specific variable definitions

;------------------------------------------------------------------------------
;
; CONFIGURATION WORD SETUP
;
; The 'CONFIG' directive is used to embed the configuration word within the
; .asm file. The lables following the directive are located in the respective
; .inc file.  See the data sheet for additional information on configuration
; word settings.
;
;------------------------------------------------------------------------------

    __CONFIG   _FCMEN_ON & _IESO_OFF & _CP_OFF & _CPD_OFF & _BOD_OFF & _MCLRE_ON & _WDT_OFF & _PWRTE_ON & _INTRC_OSC_NOCLKOUT

;------------------------------------------------------------------------------
; VARIABLE DEFINITIONS
;------------------------------------------------------------------------------

; example of using Shared Uninitialized Data Section
INT_VAR     UDATA_SHR
W_TEMP      RES     1             ; variable used for context saving
STATUS_TEMP RES     1             ; variable used for context saving

.udata UDATA
src         res     1             ; some sort of source, may be modified when you call a function
dst         res     1             ; some sort of destination, may be modified
temp_2      res     1
temp_3      res     1
temp_ctr    res     1             ; Temp counter for whatever
offset      res     1             ; some sort of offset into a pointer
offset_RG   res     1             ; brightness_R + this = brightness_G
offset_GB   res     1             ; brightness_G + this = brightness_B
offset_RB   res     1             ; brightness_R + this = brightness_B

.idata idata
light_dir   db 0, 0               ; 1: light is going up 0: light is going down

.fixed idata 0x20
max_brightness  res 1             ; Maximum brightness, gets set in the init code
temp            res 1             ; Temps for whatever
brightness_step res 1
brightness_R db 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; it looks like this initialization doesn't actually work
brightness_G db 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
brightness_B db 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0


;------------------------------------------------------------------------------
; EEPROM INITIALIZATION
;
; The 12F683 has 256 bytes of non-volatile EEPROM, starting at address 0x2100
;
;------------------------------------------------------------------------------

DATAEE    CODE  0x2100
    DE    "MCHP"          ; Place 'M' 'C' 'H' 'P' at address 0,1,2,3

;------------------------------------------------------------------------------
; RESET VECTOR
;------------------------------------------------------------------------------

RESET_VECTOR  CODE    0x0000  ; processor reset vector
        GOTO    START         ; go to beginning of program
;------------------------------------------------------------------------------
; INTERRUPT SERVICE ROUTINE
;------------------------------------------------------------------------------

INT_VECTOR    CODE    0x0004  ; interrupt vector location
        MOVWF   W_TEMP        ; save off current W register contents
        MOVF    STATUS,w      ; move status register into W register
        MOVWF   STATUS_TEMP   ; save off contents of STATUS register

        banksel INTCON
        bcf		INTCON, GIE   ;Turn off all interrupts for the duration

        call    brightness_lights_interrupt

        banksel INTCON
        movlw   0x00
        movwf   TMR0
        bcf     INTCON, T0IF
        bsf		INTCON, GIE   ;Interrupts back on
                  
        MOVF    STATUS_TEMP,w ; retrieve copy of STATUS register
        MOVWF   STATUS        ; restore pre-isr STATUS register contents
        SWAPF   W_TEMP,f
        SWAPF   W_TEMP,w      ; restore pre-isr W register contents

        RETFIE                ; return from interrupt


;------------------------------------------------------------------------------
; MAIN PROGRAM
;------------------------------------------------------------------------------

MAIN_PROG     CODE

setup_timer
		;Set the timer prescaler
		;Clear the timer
		;Clear the timer overflow bit
		clrwdt
		movlw	b'01010000' 	;Make a mask for the option register
        banksel OPTION_REG
		andwf	OPTION_REG,W
		iorlw	b'10000000'		;we want ~GPPU=1 (disable pull-ups), TOCS=0, PSA=0 and prescaler=last three
        banksel OPTION_REG
		movwf	OPTION_REG

        banksel INTCON  ;Turn on the timer0 overflow interrupt
        bsf     INTCON, T0IE
        bsf     INTCON, GIE ;And the global interrupts

clean_timer
		banksel TMR0
		clrf	TMR0			;Reset the timer to zero
		banksel	INTCON
		bcf		INTCON, T0IF	;T0IF is the timer 0 overflow bit, make sure that's cleared.

		return

setup_serial_clock

        ;Cut off output on GPIO 2
        movlw   b'111011';
        banksel TRISIO
        andwf   TRISIO
        banksel PIR1
        bcf     PIR1, TMR2IF
        ;set up the pwm pin (GPIO2) to act as our serial clock
        movlw   .12 ;There's 13 instructions in a cycle
        banksel PR2
        movwf   PR2 ;PR2 = frequency

        movlw   .1;
        banksel CCPR1L
        movwf   CCPR1L ; Pulse width = CCPR1L : CCP1CON<5:4>, we want a width of 4 so this is 1
        movlw   b'001100' ; <4:5> 0 because we want pulse width = 4, 110X to set pwm to active-high.
        banksel CCP1CON
        movwf   CCP1CON

        return

memcpy  ;takes an address in src, an address in dst, and copies a number of bytes equal to what's in w
        ;W contains src on return because you might need it, but both src and dst are changed
        banksel temp_ctr
        movwf   temp_ctr;

        banksel src
        movfw   src
        banksel temp_2
        movwf   temp_2

memcpy_loop
        banksel src
        movfw   src
        incf    src, F
        movwf   FSR
        movfw   INDF
        banksel temp
        movwf   temp
        banksel dst
        movfw   dst
        incf    dst, F
        movwf   FSR
        banksel temp
        movfw   temp
        movwf   INDF

        banksel temp_ctr
       decfsz  temp_ctr, F
       goto    memcpy_loop

        banksel temp_2
        movfw   temp_2

        return


START
		;Init
		call	setup_timer
        call    setup_serial_clock

        movlw   h'00' ;All pins off
        banksel GPIO
        movwf   GPIO

		movlw	h'00'	;Everything is set to I/O
        banksel TRISIO
        movwf   TRISIO

		;Set everything for digital IO
		banksel	ANSEL
		clrf	ANSEL
		movlw	b'00000111' ;turn off the comparator entirely
		banksel	CMCON0
		movwf	CMCON0

        movlw   0x60
        banksel max_brightness
        movwf   max_brightness ; Initialize the maximum brightness
        movwf   brightness_step ; and we start out at max brightness

        movlw   brightness_R
        movwf   FSR

        movlw   0x10
        banksel temp_ctr
        movwf   temp_ctr

        banksel temp
        clrf    temp

        incf    temp, F ; First light will be at 1 brightness, we'll increment by 2 after that

initialize_brightness
        banksel temp
        movfw   temp
        movwf   INDF

        incf    temp, F
        incf    temp, F

        incf    FSR, F
        
        banksel temp_ctr
        decfsz  temp_ctr, F
        goto    initialize_brightness

        movlw   0xFF
        banksel light_dir
        movwf   light_dir ; all lights start out going up
        movwf   light_dir+1

all_loop
        movlw   brightness_R
        movwf   FSR

        movlw   0x2
        banksel temp_2
        movwf   temp_2

        banksel light_dir
        movfw   light_dir

        banksel temp_3
        movwf   temp_3

        banksel light_dir
        movfw   light_dir+1
        movwf   light_dir
brightness_part_2

        movlw   0x8
        banksel temp_ctr
        movwf   temp_ctr

all_brightness_loop
        banksel light_dir
        rrf     light_dir, F ;Put the current bit into the carry register
        btfsc   STATUS, C
        goto    light_up
light_down
        decf    INDF, F
        movfw   INDF
        sublw   0x0         ;C = 1 if INDF == 0
        goto    all_brightness_end

light_up
        incf    INDF, F
        movfw   INDF
        banksel max_brightness
        subwf   max_brightness, W     ;want C = 0 if INDF >= max_brightness

all_brightness_end
        incf    FSR, F  ;This is safe because incf doesn't affect carry

        banksel temp_ctr
        decfsz  temp_ctr    ;same with this
        goto    all_brightness_loop

        banksel light_dir
        rrf     light_dir, F

        banksel temp_2
        decfsz  temp_2
        goto    another_round
        goto    all_loop

another_round
        movfw   light_dir
        movwf   light_dir+1

        banksel temp_3
        movfw   temp_3
        banksel light_dir
        movwf   light_dir

        goto    brightness_part_2
        

finish



brightness_lights_interrupt   ;Takes an address in W that points to an array of bytes,
                              ;then does a cycle of outputs based on the brightness values

        banksel brightness_step ; Everything is guaranteed to be in this bank
        clrf    temp ;we'll build our list of which lights are on in here,
                     ;all lights start out on but once their brightness
                     ;threshold is past they're off

        bsf     T2CON, TMR2ON

brightness_next_step
        clrf    TMR2

        clrf    temp
        movfw   brightness_G+.0
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.0
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.0
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.1
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.1
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.1
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.2
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.2
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.2
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.3
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.3
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.3
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.4
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.4
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.4
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.5
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.5
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.5
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.6
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.6
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.6
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.7
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.7
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.7
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.8
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.8
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.8
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.9
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.9
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.9
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.10
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.10
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.10
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.11
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.11
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.11
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.12
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.12
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.12
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.13
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.13
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.13
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.14
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.14
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.14
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO


        clrf    temp
        movfw   brightness_G+.15
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_R+.15
        subwf   brightness_step, W
        rlf     temp, F

        movfw   brightness_B+.15
        subwf   brightness_step, W
        btfsc   STATUS, C
        bsf     temp, 5

        movfw   temp
        movwf   GPIO

brightness_end_step

        movlw   b'010000';
        movwf   GPIO; Bang out the register clock
        clrf    GPIO

        decfsz  brightness_step, F
        goto    brightness_next_step

        bcf     T2CON, TMR2ON ;Disable our pwm serial clock

        ;If brightness_step was zero, reset it to max brightness
        movfw   max_brightness
        movwf   brightness_step

        return


        END                       ; directive 'end of program'