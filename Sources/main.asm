;*****************************************************************
;* EEBOT Navigation System                                        *
;*****************************************************************

              XDEF Entry, _Startup ;
              ABSENTRY Entry 
              INCLUDE "derivative.inc"

; LCD as well as addresses 
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   $40                   ; Starting address of 2nd line of LCD
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD Enable pin
LCD_RS        EQU   $40                   ; LCD Register Select pin
NULL          EQU   00                    ; Null character 
CR            EQU   $0D                   ; Char-return
SPACE         EQU   ' '                   ; Space


;FSM states, to be used with Dispatcher logic 
START         EQU   0  ; Each variable references a state in the FSM
FWD           EQU   1                     
ALL_STOP      EQU   2                     
LEFT_TRN      EQU   3
RIGHT_TRN     EQU   4
REV_TRN       EQU   5                     
LEFT_ALIGN    EQU   6                     
RIGHT_ALIGN   EQU   7 


; Timer for an entire left and right turn 
T_LEFT        EQU   8                           
T_RIGHT       EQU   8                             

; Variable/Data section
; ---------------------
              ORG   $3800

;*****************************************************************************
;Initial values determined from the initial measurements and Variance.       *
;*****************************************************************************
BASE_LINE     FCB   $9D                  ;Sensor Calibration (line sensor) based on EEBOT chosen 
BASE_BOW      FCB   $CA                  ;Bow sensor calibration 
BASE_MID      FCB   $CA                  ;Mid sensor calibration
BASE_PORT     FCB   $CC                  ;Port sensor calibration
BASE_STBD     FCB   $CC                  ;Starboard sensor calibration

;*****************************************************************************
LINE_VARIANCE           FCB   $21           ; Adding variance based on testing for threshold tweaks 
BOW_VARIANCE            FCB   $21           
PORT_VARIANCE           FCB   $21                      
MID_VARIANCE            FCB   $20
STARBOARD_VARIANCE      FCB   $21

; Buffers for LCD 
TOP_LINE      RMB   20                      ; LCD display buffer at top line
              FCB   NULL                    ; Null termination used
              
BOT_LINE      RMB   20                      ; LCD display buffer at bottom line
              FCB   NULL                    ; Null termination used

CLEAR_LINE    FCC   '                  '    ; Clear the line of display
              FCB   NULL                    ; Null termination used

TEMP          RMB   1                       ;A temporary variable used for its hardware state
                                            
                                            
;Sensor registers, so photoresistors effectively
SENSOR_LINE   FCB   $01                      
SENSOR_BOW    FCB   $23                     ;Hold initial sensor readings at each of these storage reserves
SENSOR_PORT   FCB   $45  
SENSOR_MID    FCB   $67                     
SENSOR_STBD   FCB   $89                     ;Current sensor number that is read.
SENSOR_NUM    RMB   1 



; Variable Section (Sourced from Lab 4 & 5)
;***************************************************************************************************
              ORG   $3850                   ; Where TOF counter register is located
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  2                       ; Current state Reg
T_TURN        ds.b  1                       ; time (Halt) turning
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit
NO_BLANK      ds.b  1                       ; Blnk
HEX_TABLE     FCC   '0123456789ABCDEF'    ; Table for converting values
BCD_SPARE     RMB   2

; Code Section
;***************************************************************************************************
              ORG   $4000
Entry:                                                                       
_Startup: 

              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              JSR   INIT                   ; Initialize ports
              JSR   openADC                ; Initialize the ATD
              JSR   initLCD                ; Initialize the LCD
              BSET  DDRA,%00000011         ; PORTA configured for motor direction                    
              BSET  DDRT,%00110000         ; PORTT configured for motor speed                    
              JSR   initAD                 ; Initialize ATD converter                  
              JSR   initLCD                ; Initialize the LCD                        
              JSR   clrLCD                 ; Clear LCD & home  
              LDX   #msg1                     ; Display msg1                              
              JSR   putsLCD                   ; ""                                        
                                                ;                                         
              LDAA  #$C0                      ; Move LCD cursor to the 2nd row            
              JSR   cmd2LCD                   ; ""                                        
              LDX   #msg3                     ; Display msg3                              
              JSR   putsLCD                   ; ""                                      
              JSR   ENABLE_TOF             ; Jump to TOF initialization

MAIN        
              JSR   G_LEDS_ON              ; Turn on guider LEDs   
              JSR   READ_SENSORS           ; Read guider sensor values
              JSR   G_LEDS_OFF             ; Disable the guider LEDs                   
              JSR   UPDT_DISPL             ; Update display
              LDAA  CRNT_STATE             ; Fetch current state and load it
              JSR   DISPATCHER             ; State transitions
              BRA   MAIN               

; Data Section
;***************************************************************************************************
;********************************************************************************************
          msg1: dc.b  "CURR_ST:",0                   ; Current state label                   
          msg3: dc.b  "Batt_V:",0                    
          
           tab: dc.b  "START  ",0               ; State names
                dc.b  "FWD    ",0               
                dc.b  "REV    ",0               
                dc.b  "RIGHT_TN ",0               
                dc.b  "LEFT_TN ",0               
                dc.b  "REV_TN ",0               
                dc.b  "STANDBY",0               
                dc.b  "Timed_REV ",0  
                                                                                                             
;*********************************************************************************************************|
; Subroutine section, mainly from lab 5 as dispatcher logic is followed
    
DISPATCHER        JSR   VERIFY_START                        ; Start of the Dispatcher                     
                  RTS                                                                                    
                                                                                                         
VERIFY_START      CMPA  #START                              ; Compare current state with START state       
                  BNE   VERIFY_FORWARD                      ; If not, jump to VERIFY_FORWARD   
                  JSR   START_ST                            ; Call START_ST if in START state                      
                  RTS                                                                                    
                                                                                                         
VERIFY_FORWARD    CMPA  #FWD                                ; Compare current state with FORWARD state      
                  BNE   VERIFY_STOP                         ; If not, move to VERIFY_STOP  
                  JSR   FWD_ST                              ; Call FWD_ST if in FORWARD state                      
                  RTS                                                                                    
                                                                                                         
VERIFY_REV_TRN    CMPA  #REV_TRN                            ; Compare current state with REV_TRN state    
                  BNE   VERIFY_LEFT_ALIGN                   ; If not, move to VERIFY_LEFT_TRN
                  JSR   REV_TRN_ST                          ; Call REV_TRN_ST if in REV_TRN state                    
                  RTS                                                                                   
                                                                                                        
VERIFY_STOP       CMPA  #ALL_STOP                           ; Compare current state with ALL_STOP state    
                  BNE   VERIFY_LEFT_TRN                     ; If not, move to VERIFY_LEFT_TRN  
                  JSR   ALL_STOP_ST                         ; Call ALL_STOP_ST if in ALL_STOP                   
                  RTS                                                                                   
                                                                                                        
VERIFY_LEFT_TRN   CMPA  #LEFT_TRN                           ; Compare current state with LEFT_TRN state   
                  BNE   VERIFY_RIGHT_TRN                    ; If not, move to VERIFT_RIGHT_TRN 
                  JSR   LEFT                                ; Call LEFT if in LEFT_TRN                    
                  RTS                                                                                                                     
                                                                                                        
VERIFY_LEFT_ALIGN CMPA  #LEFT_ALIGN                         ; Compare current state with LEFT_ALIGN state  
                  BNE   VERIFY_RIGHT_ALIGN                  ; If not, move to VERIFY_RIGHT_ALIGN
                  JSR   LEFT_ALIGN_DONE                     ; Call LEFT_ALIGN_DONE if in LEFT_ALIGN                  
                  RTS                                                                                   
                                                                                                        
VERIFY_RIGHT_TRN  CMPA  #RIGHT_TRN                          ; Compare current state with RIGHT_TRN state   
                  BNE   VERIFY_REV_TRN                      ; If not, move to VERIFY_REV_TRN  
                  JSR   RIGHT                               ; Call RIGHT if in RIGHT_TRN                                       
                                                                                                        
VERIFY_RIGHT_ALIGN CMPA  #RIGHT_ALIGN                       ; Compare current state with RIGHT_ALIGN state  
                  JSR   RIGHT_ALIGN_DONE                    ; Call RIGHT_ALIGN_DONE if in RIGHT_ALIGN                  
                  RTS                                                                      
                                                                                                        
;*********************************************************************************************************
;Start State Function to determine if roboto is in START state, and moves forward if so                  *                                              |
;*********************************************************************************************************
START_ST          BRCLR   PORTAD0, %00000100,RELEASE                                    
                  JSR     INIT_FWD                                                               
                  MOVB    #FWD, CRNT_STATE

RELEASE           RTS                                                                                                                                  

;*********************************************************************************************************
;Forward State deals with behaviour when moving forward, checking bumper sensors and managing alignment  *                                              |
;*********************************************************************************************************

FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP           ; Checks if the Front bumper is hit                           
                  MOVB    #REV_TRN, CRNT_STATE                ; if true, The state will change to REV_TURN                                
                                                                                           
                  JSR     UPDT_DISPL                          ; Update the display                                
                  JSR     INIT_REV                                                                
                  LDY     #12000                                                                   
                  JSR     del_50us                                                                
                  JSR     INIT_RIGHT                                                              
                  LDY     #6000                                                                   
                  JSR     del_50us                                                             
                  LBRA    EXIT                                                                    

NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP      ; Checks if the Rear bumper is hit
                  MOVB    #ALL_STOP, CRNT_STATE               ; if true, The state will Change to the                    
                  JSR     INIT_STOP                           ; ALL_STOP state (Means Halt)
                  LBRA    EXIT 
                  
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW                                                              
                  ADDA    BOW_VARIANCE                                                               
                  CMPA    BASE_BOW                                                                
                  BPL     NOT_ALIGNED                                                                
                  LDAA    SENSOR_MID                                                              
                  ADDA    MID_VARIANCE                                                                
                  CMPA    BASE_MID                                                                
                  BPL     NOT_ALIGNED                                                               
                  LDAA    SENSOR_LINE                                                             
                  ADDA    LINE_VARIANCE                                                                
                  CMPA    BASE_LINE                                                               
                  BPL     CHECK_RIGHT_ALIGN                                                          
                  LDAA    SENSOR_LINE          ;thresholds are dynamically determined based on the initial readings                                                    
                  SUBA    LINE_VARIANCE        ;and variances defined in the data section of the program. If the actual                                                     
                  CMPA    BASE_LINE            ;sensor readings deviate from these thresholds (baseline +/- variance),                                                     
                  BMI     CHECK_LEFT_ALIGN     ;the program takes specific actions, such as initiating turns or stopping the robot.

;*********************************************************************************************************
;NOT_ALIGNED deals with alignment based on the sensors, adjusting movement if misalignment is found      *                                              |
;*********************************************************************************************************                                                                 

NOT_ALIGNED       LDAA    SENSOR_PORT        ;Determines the movement of the Robot depending on the Sensors                                                    
                  ADDA    PORT_VARIANCE                                                               
                  CMPA    BASE_PORT                                                              
                  BPL     PARTIAL_LEFT_TRN    ;If the result is greater than or equal to BASE_PORT, branch to PARTIAL_LEFT_TRN.                                                     
                  BMI     NO_PORT             ;This  means that the sensor reading indicates an alignment condition that requires a partial left turn.                                               
                                              ;If the result is less than BASE_PORT, branch to NO_PORT. This likely means that the sensor reading does not indicate an alignment condition.
NO_PORT           LDAA    SENSOR_BOW                                                            
                  ADDA    BOW_VARIANCE                                                                 
                  CMPA    BASE_BOW                                                                
                  BPL     EXIT                                                                    
                  BMI     NO_BOW                                                              

NO_BOW            LDAA    SENSOR_STBD                                                             
                  ADDA    STARBOARD_VARIANCE                                                               
                  CMPA    BASE_STBD                                                               
                  BPL     PARTIAL_RIGHT_TRN                                                         
                  BMI     EXIT                 ;If certain conditions are met, The bot determines whether a partial left turn, a partial right turn, or no turn is needed.

;*********************************************************************************************************
;PARTIAL_LEFT_TRN does a slight left turn is alignment is only a little off to the right                 *                                              |
;*********************************************************************************************************

PARTIAL_LEFT_TRN  LDY     #6000                  ; The Robot's behavior related to left turns and left alignment.                                             
                  jsr     del_50us                                                                
                  JSR     INIT_LEFT                                                               
                  MOVB    #LEFT_TRN, CRNT_STATE                                                  
                  LDY     #6000                  ;The delays (6000 iterations of del_50us) introduced to control the duration of actions / to finish an action                                                 
                  JSR     del_50us                                                                
                  BRA     EXIT                                                                    

CHECK_LEFT_ALIGN  JSR     INIT_LEFT                                                               
                  MOVB    #LEFT_ALIGN, CRNT_STATE                                                 
                  BRA     EXIT

;*********************************************************************************************************
;PARTIAL_RIGHT_TRN does a slight right turn is alignment is only a little off to the left                *                                              |
;*********************************************************************************************************
PARTIAL_RIGHT_TRN LDY     #6000                                                                  
                  jsr     del_50us                      ; The Robot's behavior related to Right turns and right alignment                                          
                  JSR     INIT_RIGHT                                                              
                  MOVB    #RIGHT_TRN, CRNT_STATE        ;The delays (6000 iterations of del_50us) introduced to control the duration of actions / to finish an action                                         
                  LDY     #6000                                                                   
                  JSR     del_50us                                                                
                  BRA     EXIT                                                                   

CHECK_RIGHT_ALIGN JSR     INIT_RIGHT                                                              
                  MOVB    #RIGHT_ALIGN, CRNT_STATE                                                
                  BRA     EXIT                                                                                                                                                         

EXIT              RTS 

;*********************************************************************************************************
;LEFT deals with left alignment based on sensor readings, meaning forward movement ensues if left alignment met   *                                              |
;*********************************************************************************************************                                                                            
                                        ;For this Section:
LEFT              LDAA    SENSOR_BOW    ;the logic for left and right turns based on sensor readings. If the conditions for left or right alignment are met,                                                       
                  ADDA    BOW_VARIANCE  ;it sets the robot's state to forward (FWD) and initializes forward movtemen                                                                 
                  CMPA    BASE_BOW                                                               
                  BPL     LEFT_ALIGN_DONE                                                        
                  BMI     EXIT

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

;*********************************************************************************************************
;RIGHT deals with right alignment based on sensor readings, meaning forward movement ensues if right alignment met   *                                              |
;*********************************************************************************************************
RIGHT             LDAA    SENSOR_BOW                                                              
                  ADDA    BOW_VARIANCE                                                                
                  CMPA    BASE_BOW                                                                
                  BPL     RIGHT_ALIGN_DONE                                                        
                  BMI     EXIT 

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

;*********************************************************************************************************
;REV_TRN_ST deals with reverse turn based on sensor readings, where it's called if misalignment detected.*                                              |
;*********************************************************************************************************
                                        ;For this Section:
REV_TRN_ST        LDAA    SENSOR_BOW    ;In summary, REV_TRN_ST appears to handle the logic for reverse turning based on sensor readings,                                                           
                  ADDA    BOW_VARIANCE  ;and ALL_STOP_ST handles the condition for transitioning to the START state.                                                               
                  CMPA    BASE_BOW      ;The specific behavior depends on the actual sensor readings, variances, baselines, and the condition checked against PORTAD0                                                         
                  BMI     EXIT                                                                    
                  JSR     INIT_LEFT                                                               
                  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

;*********************************************************************************************************
;ALL_STOP_ST deals with halting the robot and transition over to the START state   *                                              |
;*********************************************************************************************************
ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP                                       
                  MOVB    #START, CRNT_STATE                                                      

NO_START_BUMP     RTS                                                                             


;***************************************************************************************************
; Initialization Subroutines - Set up hardware for ADC, motors, and sensors.
;***************************************************************************************************
INIT_RIGHT        BSET    PORTA,%00000010           ; Set right turn direction
                  BCLR    PORTA,%00000001        
                  LDAA    TOF_COUNTER               ; Load time of flight counter
                  ADDA    #T_RIGHT                  ; Append time for right turn
                  STAA    T_TURN
                  RTS

INIT_LEFT        
                  BSET    PORTA,%00000001           ;Set direction for left turn
                  BCLR    PORTA,%00000010 
          
                  LDAA    TOF_COUNTER               ; Load time of flight coutenr
                  ADDA    #T_LEFT                   ; Append time for left turn
                  STAA    T_TURN                    
                  RTS

INIT_FWD          BCLR    PORTA, %00000011          ; Set Forward Drive for both motors
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  LDY     #100 ;EDIT                ; Set delay time (KEY TO ADJUST IF ERRORS APPEAR)
                  JSR     del_50us  ;EDIT   
                  RTS 

INIT_REV          BSET    PORTA,%00000011            ; Set Reverse Direction for both motors
                  BSET    PTT,%00110000              ; Turn on the drive motors
                  RTS

INIT_STOP         BCLR    PTT, %00110000            ; Turn off the drive motors
                  RTS


;***************************************************************************************************
;       Initialize Sensors
INIT              BCLR   DDRAD,$FF ;Configure PORTAD as input for sensors
                  BSET   DDRA,$FF  ;Configure PORTA as output for motots
                  BSET   DDRB,$FF  ;Configure PORTB as output
                  BSET   DDRJ,$C0  ;Pins 7,6 of PTJ outputs
                  RTS


;***************************************************************************************************
;        Initialize ADC              
openADC           MOVB   #$80,ATDCTL2 ; Turn on ADC (ATDCTL2)
                  LDY    #1           ; Wait for 50 us for ADC to be ready
                  JSR    del_50us     ; - " -
                  MOVB   #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3)
                  MOVB   #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4)
                  RTS

;* **************************************************************************************************      
;* Guider LEDs ON (turns on LEDs for the sensor display)                                            *
                                 
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                  |
                  RTS                                                                             ; |
;* **************************************************************************************************      
;* Guider LEDs OFF (turns off LEDs )                                               |

G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                                |
                  RTS                  ;                                                            |    
; ***************************************************************************************************          
; *Reading Sensors Section                                             *
; ***************************************************************************************************  
READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #400           ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS


; *************************************************************************************************     
; *Select Sensor                                                     *
; *************************************************************************************************      
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS


; *************************************************************************************************         
; *Display Sensors                                                   *
; *************************************************************************************************    
DP_FRONT_SENSOR   EQU TOP_LINE+3     ;Represents the position in the display buffer 
DP_PORT_SENSOR    EQU BOT_LINE+0     ;Represents the position in the display buffer 
DP_MID_SENSOR     EQU BOT_LINE+3     ;Represents the position in the display buffer 
DP_STBD_SENSOR    EQU BOT_LINE+6     ;Represents the position in the display buffer 
DP_LINE_SENSOR    EQU BOT_LINE+9     ;Represents the position in the display buffer 

DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           ; ""
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS
                  
; ************************************************************************************************* 
; Update Display (Current State + Battery Voltage) 
; ************************************************************************************************* 
UPDT_DISPL      LDAA  #$87                      ; Move LCD cursor to the end of msg1
                JSR   cmd2LCD                   ; Send the command to LCD to move cursor

                LDAB  CRNT_STATE                ; Load current state into AccB
                LSLB                            ; Shift the state value to the left
                LSLB                            ; (to move it into the correct display position)
                LSLB                            ; Shift again to align it with the state names
                LDX   #tab                      ; Load the address of the state name tab
                ABX                             ; Adjust the cursor to the correct state label
                JSR   putsLCD                   ; Display the current state label (state name)
; *************************************************************************************************
                LDAA  #$C6                      ; Move LCD cursor to the end of msg3 (Battery voltage)
                JSR   cmd2LCD                   ; Send command to LCD to move cursor

                LDAA  TEN_THOUS                 ; Load the TEN_THOUS value (battery voltage conversion)
                JSR   putcLCD                   ; Display the TEN_THOUS value (thousands place)
                LDAA  THOUSANDS                 ; Load the THOUSANDS value
                JSR   putcLCD                   ; Display the THOUSANDS value
                LDAA  #$2E                      ; Display the decimal point
                JSR   putcLCD                   ; Display the decimal point
                LDAA  HUNDREDS                  ; Load the HUNDREDS value
                JSR   putcLCD                   ; Display the HUNDREDS value
; *************************************************************************************************
                MOVB  #$90,ATDCTL5              ; Start ADC conversion (battery voltage)
                BRCLR ATDSTAT0,$80,*            ; Wait until conversion is complete
                LDAA  ATDDR0L                   ; Load the ADC result (battery voltage)
                LDAB  #39                       ; AccB = 39 (for scaling)
                MUL                             ; Multiply result by 39 (battery scaling)
                ADDD  #600                      ; Add 600 to adjust the result
                JSR   int2BCD                   ; Convert result to BCD (binary-coded decimal)
                JSR   BCD2ASC                   ; Convert the BCD value to ASCII characters
                LDAA  #$C6                      ; Move LCD cursor to the end of msg3 (Battery voltage)
                JSR   cmd2LCD                   ; Send command to LCD to move cursor
                LDAA  TEN_THOUS                 ; Output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; Display the TEN_THOUS digit
                LDAA  THOUSANDS                 ; Output the THOUSANDS ASCII character
                JSR   putcLCD                   ; Display the THOUSANDS digit
                LDAA  #$2E                      ; Output the decimal point
                JSR   putcLCD                   ; Display the decimal point
                LDAA  HUNDREDS                  ; Output the HUNDREDS ASCII character
                JSR   putcLCD                   ; Display the HUNDREDS digit

UPDT_DISPL_EXIT RTS                             ; and exit
                
;***************************************************************************************************
;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI


; Subroutines for Utilities (LCD + Delay + Integer to Binanry etc.)
;***************************************************************************************************
initLCD:          BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS
;***************************************************************************************************
;Send command to LCD: Select LCD instruction register and execute data movement
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

;***************************************************************************************************
;Display string to LCD: Loop through the string chanracters, sending back to LCD
putsLCD:          LDAA  1,X+             ; get one character from  string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

;***************************************************************************************************
;Clear the LCD screen

clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS

;***************************************************************************************************
;Delay subroutine
del_50us          PSHX                   ; (2 E-clk) Protect the X register
eloop             LDX   #300             ; (2 E-clk) Initialize the inner loop counter
iloop             NOP                    ; (1 E-clk) No operation
                  DBNE X,iloop           ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX                   ; (3 E-clk) Restore the X register
                  RTS                    ; (5 E-clk) Else return


;***************************************************************************************************
; Send character to LCD, selecting the data register and execute data movement
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

;***************************************************************************************************
; In 4 bit units, sned 8-bit data to the LCD.
dataMov:          BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS

;***************************************************************************************************
; Analog to Digital Convertor intitializaiton. 
initAD            MOVB  #$C0,ATDCTL2      ;power up AD, select fast flag clear
                  JSR   del_50us          ;wait for 50 us
                  MOVB  #$00,ATDCTL3      ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4      ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C       ;configure pins AN03,AN02 as digital inputs
                  RTS

;***************************************************************************************************
; Straight from previous lab
int2BCD           XGDX                    ;Save the binary number into .X
                  LDAA #0                 ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0                  ; Check for a zero input
                  BEQ CON_EXIT            ; and if so, exit
                  XGDX                    ; Not zero, get the binary number back to .D as dividend
                  LDX #10                 ; Setup 10 (Decimal!) as the divisor
                  IDIV                    ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS              ; Store remainder
                  CPX #0                  ; If quotient is zero,
                  BEQ CON_EXIT            ; then exit
                  XGDX                    ; else swap first quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS                     ; Were done the conversion

LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS

;************************* converts a 4-bit binary number into its ASCII representation********************************************

;Convert 4-bit binary number to ASCII 
BIN2ASC               PSHA               ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the Lower nibble. Clear accumulator A.
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                
                      LDAA 0,X            ; Get the Lower nibble character
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the Lower nibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ; into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the Upper Snibble 
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                                                               
                      LDAA 0,X            ; Get the Upper Snibble character into ACCA
                      PULB                ; Retrieve the Lower Snibble character into ACCB
                      RTS
;***************************************************************************************************
;***************************************************************************************************

; From previous labs

BCD2ASC       LDAA  #0                        ; Initialize the blanking flag
              STAA  NO_BLANK

C_TTHOU       LDAA  TEN_THOUS                 ; Check the ?ten_thousands? digit
              ORAA  NO_BLANK
              BNE   NOT_BLANK1

ISBLANK1      LDAA  #' '                      ; It?s blank
              STAA  TEN_THOUS                 ; so store a space
              BRA   C_THOU                    ; and check the ?thousands? digit

NOT_BLANK1    LDAA  TEN_THOUS                 ; Get the ?ten_thousands? digit
              ORAA  #$30                      ; Convert to ascii
              STAA  TEN_THOUS
              LDAA  #$1                       ; Signal that we have seen a ?non-blank? digit
              STAA  NO_BLANK

C_THOU        LDAA  THOUSANDS                 ; Check the thousands digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK2

ISBLANK2      LDAA  #' '                      ; Thousands digit is blank
              STAA  THOUSANDS                 ; so store a space
              BRA   C_HUNS                    ; and check the hundreds digit

NOT_BLANK2    LDAA  THOUSANDS                 ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  THOUSANDS
              LDAA  #$1
              STAA  NO_BLANK

C_HUNS        LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK3

ISBLANK3      LDAA  #' '                      ; Hundreds digit is blank
              STAA  HUNDREDS                  ; so store a space
              BRA   C_TENS                    ; and check the tens digit

NOT_BLANK3    LDAA  HUNDREDS                  ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  HUNDREDS
              LDAA  #$1
              STAA  NO_BLANK

C_TENS        LDAA  TENS                      ; Check the tens digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK4

ISBLANK4      LDAA  #' '                      ; Tens digit is blank
              STAA  TENS                      ; so store a space
              BRA   C_UNITS                   ; and check the units digit

NOT_BLANK4    LDAA  TENS                      ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  TENS

C_UNITS       LDAA  UNITS                     ; No blank check necessary, convert to ascii.
              ORAA  #$30
              STAA  UNITS

              RTS                             ; Completed


;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector