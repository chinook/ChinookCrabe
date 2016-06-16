//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : Setup.c
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the C file for the setup of the system. It contains the
//           initialization functions.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : For ChinookLib to be useable, it must be cloned on your hard
//           drive so the path
//               "..\..\..\ChinookLib\ChinookLib.X\headers\ChinookLib.h"
//           references an existing file.
//
//           Function names can and should be renamed by the user to improve the
//           readability of the code. Also, the LED used for testing errors in
//           TimerInit is a LED on the MAX32 development board. Developpers
//           should test for errors by the means (hardware of software) they
//           judge are the best.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\Setup.h"
#include "..\headers\Interrupts.h"
#include "..\headers\SkadiFunctions.h"


//==============================================================================
//	INIT FUNCTIONS
//==============================================================================

/***********************************
 * Table of functions used in Skadi
 **********************************/
sSkadiCommand_t skadiCommandTable[] =
{
//  User call       Function    args                     Description
   {"setwind"     , SetWind     , 1 , "\t| Set wind angle [deg].\t\t\t\t| 1 arg : Min = -179.0, Max = 179.0"            }
  ,{"setpos"      , SetPos      , 1 , "\t| Set mast angle [deg].\t\t\t\t| 1 arg : Min = -179.0, Max = 179.0"            }
  ,{"setmode"     , SetMode     , 1 , "\t| Set mast mode of operation.\t\t\t| 1 arg : 1 = Manual, 0 = Auto"             }
  ,{"setparam"    , SetParam    , 2 , "\t| Set K, KI, KP, PWM_MAX, PWM_MIN and ERROR.\t| 2 args : var and val\n"        }
  ,{"getwind"     , GetWind     , 0 , "\t| Read the wind current angle [deg].\t\t| No arg needed"                       }
  ,{"getpos"      , GetPos      , 0 , "\t| Read the mast current position [deg].\t\t| 0 arg"                            }
  ,{"getmode"     , GetMode     , 0 , "\t| Get the mast current mode of operation.\t| 0 arg"                            }
  ,{"getspeed"    , GetSpeed    , 0 , "\t| Read the mast current speed [deg/s].\t\t| 0 arg"                             }
  ,{"getparam"    , GetParam    , 0 , "\t| Print K, KI, KP, PWM_MAX, PWM_MIN and ERROR.\t| 0 arg\n"                     }
  ,{"writestatus" , WriteStatus , 0 , "\t| Write STATUS msg to drive.\t\t\t| 0 arg"                                     }
  ,{"setprint"    , SetPrint    , 1 , "\t| Print or not data from regulation.\t\t| 1 arg : 1 = Print, 0 = Don't print"  }
  ,{"clc"         , ClearScreen , 0 , "\t\t| Clear terminal window.\t\t\t| 0 arg"                                       }
};

//===========================
//	INIT TIMERS
//===========================
void InitTimer(void)
{

  INT32 timerCounterValue = 0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	Open timers
//%%%%%%%%%%%%%%%%%%%%%%%%%%%
  timerCounterValue = Timer.Open(TIMER_1, 100, SCALE_MS);   // Timer used for regulating the mast. Period = 100 ms
//  timerCounterValue = Timer.Open(TIMER_1, 300, SCALE_MS);   // Timer used for regulating the mast. Period = 300 ms
  timerCounterValue = Timer.Open(TIMER_2, 21, SCALE_MS);    // Timer used for input capture AND stopping the mast. Period = 21 ms
//  timerCounterValue = Timer.Open(TIMER_2, 11, SCALE_MS);    // Timer used for input capture AND stopping the mast. Period = 11 ms
  timerCounterValue = Timer.Open(TIMER_3, 500, SCALE_US);   // Timer used for PWM. Period = 500 ms (f = 2kHz). Also used for ADC
//  timerCounterValue = Timer.Open(TIMER_3, 1000, SCALE_US);   // Timer used for PWM. Period = 500 ms (f = 2kHz). Also used for ADC
//  timerCounterValue = Timer.Open(TIMER_4,  15, SCALE_MS);   // Timer used for sending data to other devices. Period = 15 ms (f ~ 66.67 Hz)
  timerCounterValue = Timer.Open(TIMER_4, 200, SCALE_MS);   // Timer used for sending data to other devices. Period = 200 ms
  timerCounterValue = Timer.Open(TIMER_5, 600, SCALE_MS);   // Timer used for changing mode of operation. Period = 600 ms
  
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	Configure timer interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Timer.ConfigInterrupt(TIMER_1, TIMER1_INTERRUPT_PRIORITY, TIMER1_INTERRUPT_SUBPRIORITY); // Sets the priority of the TIMER_1 to the values specified in Interrupt.h
  Timer.ConfigInterrupt(TIMER_2, TIMER2_INTERRUPT_PRIORITY, TIMER2_INTERRUPT_SUBPRIORITY); // Sets the priority of the TIMER_2 to the values specified in Interrupt.h
  Timer.ConfigInterrupt(TIMER_3, TIMER3_INTERRUPT_PRIORITY, TIMER3_INTERRUPT_SUBPRIORITY); // Sets the priority of the TIMER_3 to the values specified in Interrupt.h
  Timer.ConfigInterrupt(TIMER_4, TIMER4_INTERRUPT_PRIORITY, TIMER4_INTERRUPT_SUBPRIORITY); // Sets the priority of the TIMER_4 to the values specified in Interrupt.h
  Timer.ConfigInterrupt(TIMER_5, TIMER5_INTERRUPT_PRIORITY, TIMER5_INTERRUPT_SUBPRIORITY); // Sets the priority of the TIMER_5 to the values specified in Interrupt.h

}

//===========================
//	INIT SKADI
//===========================
void InitSkadi(void)
{
  Skadi.Init(skadiCommandTable, sizeof(skadiCommandTable)/sizeof(sSkadiCommand_t), UART6, TRUE);  // TRUE ==> Use interrupts with UART
}

//===========================
//	INIT SPI
//===========================
void InitSpi(void)
{
  INT8 err = 0;
  SpiOpenFlags_t oMasterFlags =   SPI_MASTER_MODE
                                | SPI_16_BITS_CHAR
                                | SPI_ENHANCED_BUFFER_MODE
                                | SPI_TX_EVENT_BUFFER_SR_EMPTY
                                | SPI_RX_EVENT_BUFFER_NOT_EMPTY
                                | SPI_SAMPLE_END_CLK
                                | SPI_DATA_ON_CLK_FEDGE
                                ;

  err = Spi.Open(SPI4, oMasterFlags, 5e5);   // Open the SPI4 as a master at a bitrate of 5 MHz
  if (err < 0)                // Check for errors
  {
    LED_ERROR_ON;
  }

  // SPI interrupts not functionnal as of yet
//  Spi.ConfigInterrupt(SPI4, SPI4_INTERRUPT_PRIORITY, SPI4_INTERRUPT_SUBPRIORITY);  // Configure Interrupt for SPI4
}


//===========================
//	INIT PWM
//===========================
void InitPwm(void)
{
  // DRIVE B
  //==========================================================
  if (USE_DRIVE_B == 1)
  {
    // Open PWM2 using Timer3 with 50% duty cycle and 0% offset
    Pwm.Open(PWM_2);
    Pwm.SetDutyCycle  (PWM_2, 500);
    Pwm.SetPulseOffset(PWM_2,   0);

    // Open PWM3 using Timer3 with 50% duty cycle and 50% offset
    Pwm.Open(PWM_3);
    Pwm.SetDutyCycle  (PWM_3, 500);
    Pwm.SetPulseOffset(PWM_3, 500);
  }
  //==========================================================


  // DRIVE A
  //==========================================================
  if (USE_DRIVE_A == 1)
  {
    // Open PWM4 using Timer3 with 50% duty cycle and 0% offset
    Pwm.Open(PWM_4);
    Pwm.SetDutyCycle  (PWM_4, 500);
    Pwm.SetPulseOffset(PWM_4,   0);

    // Open PWM5 using Timer3 with 50% duty cycle and 50% offset
    Pwm.Open(PWM_5);
    Pwm.SetDutyCycle  (PWM_5, 500);
    Pwm.SetPulseOffset(PWM_5, 500);
  }
  //==========================================================
}


//===========================
//	INIT PORTS
//===========================
void InitPorts(void)
{
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Set unused pins as input to protect the pins of the microcontroller
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Port.A.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3      // RA8, RA11-13 non existent
                    | BIT_4  | BIT_5  | BIT_6  | BIT_7
                    | BIT_9  | BIT_10 | BIT_14 | BIT_15 );

  Port.B.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3
                    | BIT_4  | BIT_5  | BIT_6  | BIT_7
                    | BIT_8  | BIT_9  | BIT_10 | BIT_11
                    | BIT_12 | BIT_13 | BIT_14 | BIT_15 );

  Port.C.CloseBits  ( BIT_1  | BIT_2  | BIT_3  | BIT_4      // RC0, RC5-11 non existent
                    | BIT_12 | BIT_13 | BIT_14 | BIT_15 );

  Port.D.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3
                    | BIT_4  | BIT_5  | BIT_6  | BIT_7
                    | BIT_8  | BIT_9  | BIT_10 | BIT_11
                    | BIT_12 | BIT_13 | BIT_14 | BIT_15 );

  Port.E.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3      // RE10-15 non existent
                    | BIT_4  | BIT_5  | BIT_6  | BIT_7
                    | BIT_8  | BIT_9  );

  Port.F.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3      // RF6-7, RF9-11, RF14-15 non existent
                    | BIT_4  | BIT_5  | BIT_8  | BIT_12
                    | BIT_13 );

  Port.G.CloseBits  ( BIT_0  | BIT_1  | BIT_2  | BIT_3      // RG4-5, RG10-11 non existent
                    | BIT_6  | BIT_7  | BIT_8  | BIT_9
                    | BIT_12 | BIT_13 | BIT_14 | BIT_15 );

   /* Setup IO LED */
  Port.B.SetPinsDigitalOut(BIT_8);      // LED_DEBG0
  Port.B.SetPinsDigitalOut(BIT_9);      // LED_DEBG1
  Port.B.SetPinsDigitalOut(BIT_10);     // LED_DEBG2
  Port.B.SetPinsDigitalOut(BIT_11);     // LED_DEBG3
  Port.B.SetPinsDigitalOut(BIT_12);     // LED_DEBG4
  Port.B.SetPinsDigitalOut(BIT_13);     // LED_CAN
  Port.B.SetPinsDigitalOut(BIT_15);     // LED_ERROR
  Port.F.SetPinsDigitalOut(BIT_3);      // LED_STATUS

  /* Setup  IO switch */
  Port.E.SetPinsDigitalIn(BIT_5);      // SW1
  Port.E.SetPinsDigitalIn(BIT_6);      // SW2
  Port.E.SetPinsDigitalIn(BIT_7);      // SW3

  /* Setup  Output to control Drive A */
  Port.E.SetPinsDigitalOut(BIT_3);      // DRVA_RESET
  Port.E.SetPinsDigitalOut(BIT_4);      // DRVA_SLEEP
  Port.E.SetPinsDigitalOut(BIT_1);      // DRVA_DIR
  Port.E.SetPinsDigitalOut(BIT_2);      // DRVA_STEP
  Port.D.SetPinsDigitalOut(BIT_4);      // DRVA_BIN1
  Port.D.SetPinsDigitalOut(BIT_2);      // DRVA_BIN2

  /* Setup  Input Drive A */
  Port.D.SetPinsDigitalIn(BIT_6);      // DRVA_FLT
  Port.D.SetPinsDigitalIn(BIT_5);      // DRVA_STALL

  /* Setup  Output to control Drive B */
//    Port.G.SetPinsDigitalOut(BIT_2);    // DRVB_RESET
  Port.D.SetPinsDigitalOut(BIT_0);      // DRVB_SLEEP
  Port.E.SetPinsDigitalOut(BIT_0);      // DRVB_DIR
  Port.C.SetPinsDigitalOut(BIT_15);     // DRVB_STEP
  Port.D.SetPinsDigitalOut(BIT_3);      // DRVB_BIN1
  Port.D.SetPinsDigitalOut(BIT_1);      // DRVB_BIN2

  /* Setup  Input Drive B */
  Port.D.SetPinsDigitalIn(BIT_5);       // DRVB_FLT
//    Port.C.SetPinsDigitalIn(BIT_15);    // DRVB_STALL

  /* Setup  SPI SCn */
  Port.F.SetPinsDigitalIn(BIT_4);       // SDI
  Port.F.SetPinsDigitalOut(BIT_5);      // SD0
  Port.C.SetPinsDigitalOut(BIT_13);     // DRVB_SC
  Port.C.SetPinsDigitalOut(BIT_14);     // DRVA_SC
  Port.B.SetPinsDigitalOut(BIT_14);     // CLK

  Port.D.SetPinsDigitalIn(BIT_11);      // LIMIT_SW

  Port.G.SetPinsDigitalIn (BIT_9);      // U6RX
  Port.G.SetPinsDigitalOut(BIT_6);      // U6TX

  Port.D.SetPinsDigitalIn (BIT_8);      // DRVA_IO_CON1
  Port.D.SetPinsDigitalIn (BIT_10);     // DRVA_IO_CON2

  LED_STATUS_OFF;
  LED_ERROR_OFF;
  LED_CAN_OFF;
  LED_DEBUG4_OFF;
  LED_DEBUG3_OFF;
  LED_DEBUG2_OFF;
  LED_DEBUG1_OFF;
  LED_DEBUG0_OFF;
  
  
//  DRVA_STEP = 0;
//  DRVA_BIN1 = 1;
//  DRVA_BIN2 = 0;
  DRVB_SLEEP = 0;
  DRVA_SLEEP = 0;
  DRVB_STEP = 0;
  DRVB_BIN1 = 0;
  DRVB_BIN2 = 0;
}


//===========================
//	INIT UART
//===========================
void InitUart (void)
{

  UartConfig_t        oConfig       = UART_ENABLE_PINS_TX_RX_ONLY;
  UartFifoMode_t      oFifoMode     = UART_INTERRUPT_ON_TX_BUFFER_EMPTY | UART_INTERRUPT_ON_RX_NOT_EMPTY;
  UartLineCtrlMode_t  oLineControl  = UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1;

  Uart.Open(UART6, BAUD57600, oConfig, oFifoMode, oLineControl);   // Open UART 6 as : 9600 BAUD, 1 stop bit, no parity and 8 bits data, not used as main UART for Skadi
  Uart.EnableTx(UART6);
  Uart.EnableRx(UART6);

  Uart.ConfigInterrupt(UART6, UART6_INTERRUPT_PRIORITY, UART6_INTERRUPT_SUBPRIORITY);
  
}


//===========================
//	INIT CAN BUS
//===========================
void InitCan(void)
{
  /**Setup par defaut.
   * CAN_CHANNEL0
   * Mode: TX
   * Type: SID
   *
   * CAN_CHANNEL1
   * Mode: RX
   * CAN_FILTER0: 0xC1, this configures the filter to accept with ID 0xC1
   * CAN_FILTER_MASK0: 0x00, Configure CAN1 Filter Mask 0 to comprare no bits
   * */

  INT8 err;
  err = Can.Initialize(CAN1, Can1MessageFifoArea, CAN_NB_CHANNELS, CAN_BUFFER_SIZE, FALSE);
  if (err < 0)
  {
    LED_ERROR_ON;
  }

  // Switches from steering wheel
  Can.SetChannel(CAN1, CAN_CHANNEL1, 8, RX);
  Can.SetChannelMask(CAN1, CAN_CHANNEL1, CAN_FILTER0, 0x42, CAN_FILTER_MASK0, 0x7FF);

  // Data from telemetry
  Can.SetChannel(CAN1, CAN_CHANNEL2, 8, RX);
  Can.SetChannelMask(CAN1, CAN_CHANNEL2, CAN_FILTER1, 0x20, CAN_FILTER_MASK0, 0x7FF);

  // Potentiometer from steering wheel
  Can.SetChannel(CAN1, CAN_CHANNEL3, 8, RX);
  Can.SetChannelMask(CAN1, CAN_CHANNEL3, CAN_FILTER2, 0x43, CAN_FILTER_MASK0, 0x7FF);

  Can.ConfigInterrupt(CAN1, CAN1_INTERRUPT_PRIORITY, CAN1_INTERRUPT_SUBPRIORITY);
}


//===========================
//	INIT I2C
//===========================
void InitI2c(void)
{
  INT8 err;
  I2c.Open(I2C4, I2C_FREQ_400K);
  err = I2c.ConfigInterrupt(I2C4, I2C4_INTERRUPT_PRIORITY, I2C4_INTERRUPT_SUBPRIORITY);
  if (err < 0)
  {
    LED_ERROR_ON;
  }
}


//===========================
//	INIT WATCHDOG TIMER
//===========================
void InitWdt(void)
{
  Wdt.Enable();
}


//===========================
//	INIT INPUT CAPTURE
//===========================
void InitInputCapture(void)
{
  // Capture every rising edge, 1 interrupt each capture, use the 16 bits Timer 2, capture the first rising edge, Input Capture ON
  UINT16 config = IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_CAP_16BIT | IC_FEDGE_RISE | IC_ON;

  // Capture every rising edge, 1 interrupt each capture, use the 32 bits Timer 23, capture the first rising edge, Input Capture ON
//  UINT16 config = IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_CAP_32BIT | IC_FEDGE_RISE | IC_ON;

  // DRIVE A
  //==========================================================
  if (USE_DRIVE_A == 1)
  {
    InputCapture.Open(IC1, config);
    InputCapture.Open(IC3, config);

    InputCapture.ConfigInterrupt(IC1, IC1_INTERRUPT_PRIORITY, IC1_INTERRUPT_SUBPRIORITY);
    InputCapture.ConfigInterrupt(IC3, IC3_INTERRUPT_PRIORITY, IC3_INTERRUPT_SUBPRIORITY);

    INTClearFlag(INT_IC1);
    INTClearFlag(INT_IC3);
  }
  //==========================================================


  // DRIVE B
  //==========================================================
  if (USE_DRIVE_B == 1)
  {
    InputCapture.Open(IC2, config);
    InputCapture.Open(IC4, config);

    InputCapture.ConfigInterrupt(IC2, IC2_INTERRUPT_PRIORITY, IC2_INTERRUPT_SUBPRIORITY);
    InputCapture.ConfigInterrupt(IC4, IC4_INTERRUPT_PRIORITY, IC4_INTERRUPT_SUBPRIORITY);

    INTClearFlag(INT_IC2);
    INTClearFlag(INT_IC4);
  }
  //==========================================================

}


//===========================
//	START INTERRUPTS
//===========================
void StartInterrupts(void)
{
  INT8 err;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable timer interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Timer.EnableInterrupt(TIMER_1);
  Timer.EnableInterrupt(TIMER_2);
  Timer.EnableInterrupt(TIMER_3);
  Timer.EnableInterrupt(TIMER_4);
  Timer.EnableInterrupt(TIMER_5);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable SPI interrupts             // Not functionnal yet
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  Spi.EnableRxInterrupts(SPI4);   // Enable RX Interrupts for SPI4
//  Spi.EnableTxInterrupts(SPI4);   // Enable TX Interrupts for SPI4


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable InputCapture interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // DRIVE B
  //====================================
  if (USE_DRIVE_B == 1)
  {
    InputCapture.EnableInterrupt(IC2);
    InputCapture.EnableInterrupt(IC4);
  }
  //====================================

  // DRIVE A
  //====================================
  if (USE_DRIVE_A == 1)
  {
    InputCapture.EnableInterrupt(IC1);
    InputCapture.EnableInterrupt(IC3);
  }
  //====================================


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable CAN interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Can.EnableInterrupt(CAN1);


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable I2C interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  err = I2c.EnableInterrupt (I2C4, I2C_MASTER_INTERRUPT);
  if (err < 0)
  {
    LED_ERROR_ON;
  }
  err = I2c.DisableInterrupt(I2C4, I2C_SLAVE_INTERRUPT);
  if (err < 0)
  {
    LED_ERROR_ON;
  }
  err = I2c.DisableInterrupt(I2C4, I2C_BUS_COLLISION_INTERRUPT);
  if (err < 0)
  {
    LED_ERROR_ON;
  }


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable ADC interrupts        
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Adc.EnableInterrupts ();  // Enable Interrupts for ADC


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable UART interrupts        
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Uart.EnableRxInterrupts (UART6);  // Enable RX Interrupts for UART6
  Uart.DisableTxInterrupts(UART6);  // Disable TX Interrupts for UART6


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Enable multi-vector interrupts
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
  INTEnableInterrupts();

}


//===========================
//	INIT ADC
//===========================
void InitAdc(void)
{
  // Mode of operation. Uncomment the one you need.
  //================================================
  UINT32 samplingClk = ADC_CLK_TMR;     // Timer3 used for sampling
  //================================================

  // Hardware config. These are exemples.
  //================================================
  UINT32 configHardware = ADC_VREF_EXT_AVSS      // Vref+ is AVdd and Vref- is AVss
                        | ADC_SAMPLES_PER_INT_2;  // 3 samples/interrupt (we check 3 channels)
  //================================================

  // Port config.
  //================================================

  UINT32 configPort = ENABLE_AN2_ANA
                    | ENABLE_AN3_ANA
                    ;
  
  UINT32 configScan = SKIP_SCAN_AN0
                    | SKIP_SCAN_AN1
                    | SKIP_SCAN_AN4
                    | SKIP_SCAN_AN5
                    | SKIP_SCAN_AN6
                    | SKIP_SCAN_AN7
                    | SKIP_SCAN_AN8
                    | SKIP_SCAN_AN9
                    | SKIP_SCAN_AN10
                    | SKIP_SCAN_AN11
                    | SKIP_SCAN_AN12
                    | SKIP_SCAN_AN13
                    | SKIP_SCAN_AN14
                    | SKIP_SCAN_AN15; // Don't scan the channels that are not enabled by configPort
  //================================================

  // Open ADC with parameters above
  Adc.Open(samplingClk, configHardware, configPort, configScan);

  Adc.ConfigInterrupt(ADC_INTERRUPT_PRIORITY, ADC_INTERRUPT_SUBPRIORITY);
}