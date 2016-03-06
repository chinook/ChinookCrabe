//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : Setup.h
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the setup header for the system. It contains function
//           prototypes as well as macros and the public structure of
//           ChinookLib functions.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : For ChinookLib to be useable, it must be cloned on your hard
//           drive so the path
//               "..\..\..\ChinookLib\ChinookLib.X\headers\ChinookLib.h"
//           references an existing file.
//
//           Function names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef __SETUP_H__
#define	__SETUP_H__

//%%%%%%%%%%%%%%%%%%%%%%%%%
// ChinookLib files
//%%%%%%%%%%%%%%%%%%%%%%%%%
#include "..\..\..\ChinookLib\ChinookLib.X\headers\ChinookLib.h"

//==============================================================================
// ChinookLib functions
//==============================================================================
struct sChinook Chinook;            // Contains all ChinookLib functions


//==============================================================================
// State Machine public functions prototypes
//==============================================================================
void InitTimer        (void);
void InitPwm          (void);
void InitPorts        (void);
void InitAdc          (void);
void InitCan          (void);
void InitSpi          (void);
void InitUart         (void);
void InitI2c          (void);
void InitSkadi        (void);
void InitWdt          (void);
void InitInputCapture (void);
void StartInterrupts  (void);


//==============================================================================
// Which drive to use
//==============================================================================
#define USE_DRIVE_A   1
#define USE_DRIVE_B   1

//==============================================================================
// Macro definitions
//==============================================================================
#define INIT_TIMER          InitTimer()
#define INIT_PWM            InitPwm()
#define INIT_PORTS          InitPorts()
#define INIT_CAN            InitCan()
#define INIT_SPI            InitSpi()
#define INIT_UART           InitUart()
#define INIT_I2C            InitI2c()
#define INIT_ADC            InitAdc()
#define INIT_SKADI          InitSkadi()
#define INIT_WDT            InitWdt()
#define INIT_INPUT_CAPTURE  InitInputCapture()
#define START_INTERRUPTS    StartInterrupts()

//==============================================================================
// Define OUTPUT LED
//==============================================================================
#define LED_STATUS        LATFbits.LATF3
#define LED_CAN           LATBbits.LATB13
#define LED_ERROR         LATBbits.LATB15
#define LED_DEBG4         LATBbits.LATB12
#define LED_DEBG3         LATBbits.LATB11
#define LED_DEBG2         LATBbits.LATB10
#define LED_DEBG1         LATBbits.LATB9
#define LED_DEBG0         LATBbits.LATB8

/* Setup IO LED */
#define LED_DEBUG0_ON       Port.B.ClearBits(BIT_8)
#define LED_DEBUG1_ON       Port.B.ClearBits(BIT_9)
#define LED_DEBUG2_ON       Port.B.ClearBits(BIT_10)
#define LED_DEBUG3_ON       Port.B.ClearBits(BIT_11)
#define LED_DEBUG4_ON       Port.B.ClearBits(BIT_12)
#define LED_ERROR_ON        Port.B.ClearBits(BIT_15)
#define LED_CAN_ON          Port.B.ClearBits(BIT_13)
#define LED_STATUS_ON       Port.F.ClearBits(BIT_3)

#define LED_DEBUG0_OFF      Port.B.SetBits(BIT_8)
#define LED_DEBUG1_OFF      Port.B.SetBits(BIT_9)
#define LED_DEBUG2_OFF      Port.B.SetBits(BIT_10)
#define LED_DEBUG3_OFF      Port.B.SetBits(BIT_11)
#define LED_DEBUG4_OFF      Port.B.SetBits(BIT_12)
#define LED_ERROR_OFF       Port.B.SetBits(BIT_15)
#define LED_CAN_OFF         Port.B.SetBits(BIT_13)
#define LED_STATUS_OFF      Port.F.SetBits(BIT_3)

#define LED_DEBUG0_TOGGLE   Port.B.ToggleBits(BIT_8)
#define LED_DEBUG1_TOGGLE   Port.B.ToggleBits(BIT_9)
#define LED_DEBUG2_TOGGLE   Port.B.ToggleBits(BIT_10)
#define LED_DEBUG3_TOGGLE   Port.B.ToggleBits(BIT_11)
#define LED_DEBUG4_TOGGLE   Port.B.ToggleBits(BIT_12)
#define LED_ERROR_TOGGLE    Port.B.ToggleBits(BIT_15)
#define LED_CAN_TOGGLE      Port.B.ToggleBits(BIT_13)
#define LED_STATUS_TOGGLE   Port.F.ToggleBits(BIT_3)

#define LED_ALL_OFF()     { LED_DEBUG0_OFF    ; LED_DEBUG1_OFF    ; \
                            LED_DEBUG2_OFF    ; LED_DEBUG3_OFF    ; \
                            LED_DEBUG4_OFF    ; LED_ERROR_OFF     ; \
                            LED_CAN_OFF       ; LED_STATUS_OFF    ; }

#define LED_ALL_ON()      { LED_DEBUG0_ON     ; LED_DEBUG1_ON     ; \
                            LED_DEBUG2_ON     ; LED_DEBUG3_ON     ; \
                            LED_DEBUG4_ON     ; LED_ERROR_ON      ; \
                            LED_CAN_ON        ; LED_STATUS_ON     ; }

#define LED_ALL_TOGGLE()  { LED_DEBUG0_TOGGLE ; LED_DEBUG1_TOGGLE ; \
                            LED_DEBUG2_TOGGLE ; LED_DEBUG3_TOGGLE ; \
                            LED_DEBUG4_TOGGLE ; LED_ERROR_TOGGLE  ; \
                            LED_CAN_TOGGLE    ; LED_STATUS_TOGGLE ; }

//==============================================================================
// Define INPUT SWITCH
//==============================================================================
#define SW1               PORTEbits.RE5
#define SW2               PORTEbits.RE6
#define SW3               PORTEbits.RE7


/* Setup  IO switch */
#define READ_SW1    Port.E.ReadBits(BIT_5) >> 5
#define READ_SW2    Port.E.ReadBits(BIT_6) >> 6
#define READ_SW3    Port.E.ReadBits(BIT_7) >> 7

//==============================================================================
// Define OUTPUT CONTROL DRIVE A
//==============================================================================
#define DRVA_RESET        LATEbits.LATE3
#define DRVA_SLEEP        LATEbits.LATE4
#define DRVA_DIR          LATEbits.LATE1
#define DRVA_STEP         LATEbits.LATE2
#define DRVA_BIN1         LATDbits.LATD4
#define DRVA_BIN2         LATDbits.LATD2

//==============================================================================
// Define INPUT CONTROL DRIVE A
//==============================================================================
#define DRVA_FLT          LATDbits.LATD6
#define DRVA_STALL        LATDbits.LATD5

//==============================================================================
// Define OUTPUT CONTROL DRIVE B
//==============================================================================
#define DRVB_RESET        LATGbits.LATG2
#define DRVB_SLEEP        LATDbits.LATD0
#define DRVB_DIR          LATEbits.LATE0
#define DRVB_STEP         LATCbits.LATC15
#define DRVB_BIN1         LATDbits.LATD3
#define DRVB_BIN2         LATDbits.LATD1

//==============================================================================
// Define INPUT CONTROL DRIVE B
//==============================================================================
#define DRVB_FLT          LATDbits.LATD5
//#define DRVB_STALL        LATCbits.LATC15

//==============================================================================
// Define OUTPUT SPI SCn
//==============================================================================
#define DRVB_SC           LATCbits.LATC13
#define DRVA_SC           LATCbits.LATC14

//==============================================================================
// Define UNCODER
//==============================================================================
#define DRVA_IO_CON1      PORTDbits.RD8
#define DRVA_IO_CON2      PORTDbits.RD10


enum drvRegAdress {
  regCTRLadd    = 0b000,
  regTORQUEadd  = 0b001,
  regOFFadd     = 0b010,
  regBLANKadd   = 0b011,
  regDECAYadd   = 0b100,
  regSTALLadd   = 0b101,
  regDRIVEadd   = 0b110,
  regSTATUSadd  = 0b111,
};

typedef struct{
  INT8 ENABLE;              //0 = disable, 1 = enable
  INT8 RDIR;                //0 = use DIR pin, 1 = use inverse DIR PIN
  INT8 RSETEP;             //0 = nothing, 1 : give a step on stepper = STEP pin
  INT8 MODE;                //000 = FULL step.... 1000 = 1/256, other reserved
  INT8 EXSTALL;             //0 = internal stall effect, 1 = external stall effect
  INT8 ISGAIN;              //gain AOP,  00=5, 01=10, 10=20, 11=40
  INT8 DTIME;               //dead time, 00=400ns, 01=450ns, 10=650ns, 11=850ns
}drvControl;

typedef struct{
  INT8 TORQUE;              //torque value, see equation of max current
  INT8 SMPLTH;              //EMF sample : 000 = 50us, 001 = 100us, 010 = 200us, 011 = 300us, 100 = 400us, 101 = 600us, 110 = 800us, 111 = 1000us,
}drvTorque;

typedef struct{
  INT8 TOFF;                //set off time, increment of 500ns
  INT8 PWMMODE;             //0 = use internal indexer, 1 = use xINx inputs to control output
}drvOff;

typedef struct{
  INT8 TBLANK;              //set current trip blank time
  INT8 ABT;                 //0 = disable adaptive blamking time, 1 = enable adaptive blamking time
}drvBlank;

typedef struct{
  INT8 TDECAY;              //increment of 500ns
  INT8 DECMOD;              //000 : force slow decay
                            //001 : slow decay for increasing current
                            //010 : force fast decay
                            //011 : use mexed decay
                            //100 : slow decay for increasing current
                            //101 : AUTO mixed decay
                            //110 -111 :reserved
}drvDecay;

typedef struct{
  INT8 SDTHR;               //stall detec threshold, determined experimentaly
  INT8 SDCNT;               //STALLn asserted after, 00 = 1 step, 01 = 2 step, 10 = 4 step, 11 = 8 step
  INT8 VDIV;                //back EMF devided by, 00 = 32, 01 = 16, 10 = 8, 11 = 4
}drvStall;

typedef struct{
  INT8 OCPTH;               //OCP threshold, 00 = 250mV, 01 = 500mV, 10 = 750mV, 11 = 1000mV,
  INT8 OCPDEG;              //OCP deglitch time, 00 = 1 us, 01 = 2 us, 10 = 4 us, 11 = 8 us
  INT8 TDRIVEN;             //Low-side gate drive time, 00 = 250 ns, 01 = 500 ns, 10 = 1 us, 11 = 2 us
  INT8 TDRIVEP;             //High-side gate drive time, 00 = 250 ns, 01 = 500 ns, 10 = 1 us, 11 = 2 us
  INT8 IDRIVEN;             //Low-side gate drive peak current, 00 = 100mA, 01 = 200mA, 10 = 300mA, 11 = 400mA
  INT8 IDRIVEP;             //High-side gate drive peak current, 00 = 100mA, 01 = 200mA, 10 = 300mA, 11 = 400mA
}drvDrive;

typedef struct{
  INT8 OTS;                 //0 = normale operation, 1 = overtemperature shutdown detected
  INT8 AOCP;                //0 = normale operation, 1 = channel A overcurrent shutdown
  INT8 BOCP;                //0 = normale operation, 1 = channel B overcurrent shutdown
  INT8 APDF;                //0 = normale operation, 1 = channel A predriver fault
  INT8 BPDF;                //0 = normale operation, 1 = channel B predriver fault
  INT8 UVLO;                //0 = normale operation, 1 = overvoltage lockout
  INT8 STD;                 //0 = normale operation, 1 = stall detected
  INT8 STDLAT;              //0 = normale operation, 1 = latched stall detect
}drvStatus;

typedef struct{
  drvControl  Ctl;
  drvTorque   Tor;
  drvOff      Off;
  drvBlank    Bla;
  drvDecay    Dec;
  drvStall    Sll;
  drvDrive    Dri;
  drvStatus   Sta;
}drvReg;


//==============================================================================
// Variable definitions for CAN
//==============================================================================
#define CAN_NB_CHANNELS     3 // 3 CAN channels will be used
#define CAN_BUFFER_SIZE     8

BYTE Can1MessageFifoArea [ CAN_NB_CHANNELS     // Space used by CAN
                         * CAN_BUFFER_SIZE
                         * CAN_TX_RX_MESSAGE_SIZE_BYTES
                         ];

#define CRAB_BOARD_SID                0x80
#define CRAB_DISCONNECT_SID           0x81
#define CRAB_DIRECTION_DEG_SID        0x82
#define CRAB_MODE_SID                 0x83
#define CRAB_CALIB_DONE_SID           0x84
#define CRAB_CALIB_DONE_SID           0x84
#define CRAB_DIRECTION_MM_SID         0x85

#define IDENTIFICATION_TO_BACKPLANE   0x70
#define DISCONNECT_FROM_BACKPLANE     0x71

// Message 0 : Identification
#define SEND_ID_TO_BACKPLANE          Can.SendByte(CAN1, IDENTIFICATION_TO_BACKPLANE, CRAB_BOARD_SID)

// Message 1 : Disconnect
#define SEND_DISCONNECT_TO_BACKPLANE  Can.SendByte(CAN1, DISCONNECT_FROM_BACKPLANE, CRAB_DISCONNECT_SID)

// Message 2 : Crab orientation in degree
#define SEND_CRAB_DIR_DEG             Can.SendFloat(CAN1, CRAB_DIRECTION_DEG_SID, mastAngle.currentValue)

// Message 3 : Crab mode of operation
#define SEND_MODE_TO_STEERING_WHEEL   Can.SendByte(CAN1, CRAB_MODE_SID, (BYTE) oManualMode)

// Message 4 : Crab calib done
#define SEND_CALIB_DONE               Can.SendByte(CAN1, CRAB_CALIB_DONE_SID, 1)

// Message 5 : Crab orientation in mm
#define SEND_CRAB_DIR_MM              Can.SendFloat(CAN1, CRAB_DIRECTION_MM_SID, mastAngle.currentValue)


#endif	/* __SETUP_H__ */