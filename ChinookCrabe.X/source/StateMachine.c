//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook V
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateMachine_Mast.c
// Author  : Amaury LAINE
// Date    : 2015-03-13
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : Used to controle stepper for Chinook V
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : NA
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\StateMachine.h"
#include "..\headers\DRV8711_Para.h"
#include "..\headers\Interrupts.h"
#include "..\headers\StateFunctions.h"
#include "..\headers\CommandFunctions.h"

extern volatile sButtonStates_t buttons;

extern volatile UINT32 rxWindAngle;

volatile float crabManualCmdDeg = 0;

float  crabManualCmdMmLeft  = 0
      ,crabManualCmdMmRight = 0
      ;

extern float crabLeftZeroMm
            ,crabRightZeroMm
            ;

float  leftActPos
      ,rightActPos
      ,leftActDeg
      ,rightActDeg
      ;

BOOL oNewAdcMeasurement = 0;

volatile BOOL oNewManualCmd = 1;

BOOL oManualLeftMoving      = 0
    ,oManualRightMoving     = 0
    ,oManualLeftStopped     = 1
    ,oManualRightStopped    = 1
    ;


// Used for the average of the wind angle
//========================================
extern volatile UINT32 nWindAngleSamples;
extern volatile float  meanWindAngle;
//========================================

extern volatile sCmdValue_t windAngle
                           ,mastAngle
                           ,mastSpeed
                           ;

// Mast general value
volatile float mastCurrentSpeed   = 0   // Actual speed of Mast
              ;

extern volatile BOOL oAdcReady
                    ,oNewWindAngle
                    ,oTimerReg
                    ,oTimerSendData
                    ,oTimerChngMode
                    ;

volatile BOOL  oManualMode            = 1
              ,oCountTimeToChngMode   = 0
              ,oManualFlagChng        = 0
              ,oManualMastRight       = 0
              ,oManualMastLeft        = 0
              ;


//==============================================================================
//	STATES OF STATE MACHINE
//==============================================================================

//===============================================================
// Name     : StateScheduler_Mast
// Purpose  : Decides which state is next depending on current
//            state and flags. Used as a function
//===============================================================
void StateScheduler(void)
{

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateAcq
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if (pStateMast == &StateAcq)
  {
    if (ACQ_2_DISCONNECT)
    {
      pStateMast = &StateDisconnect;
    }
    else if (ACQ_2_MANUAL)
    {
      pStateMast = &StateManual;
    }
    else if (ACQ_2_REG)
    {
      pStateMast = &StateReg;
    }
    else if (ACQ_2_GET_MAST_DATA)
    {
      pStateMast = &StateGetMastData;
    }
    else if (ACQ_2_SEND_DATA)
    {
      pStateMast = &StateSendData;
    }
    else
    {
      pStateMast = &StateAcq;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateReg
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateReg)
  {
    if (REG_2_ACQ)
    {
      pStateMast = &StateAcq;
    }
    else
    {
      pStateMast = &StateReg;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateManual
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateManual)
  {
    if (MANUAL_2_ACQ)
    {
      pStateMast = &StateAcq;
    }
    else
    {
      pStateMast = &StateManual;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateSendData
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateSendData)
  {
    if (SEND_DATA_2_ACQ)
    {
      pStateMast = &StateAcq;
    }
    else
    {
      pStateMast = &StateSendData;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateInit
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateInit)
  {
    if (INIT_2_ACQ)
    {
      pStateMast = &StateAcq;
    }
    else
    {
      pStateMast = &StateInit;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateDisconnect
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateDisconnect)
  {
    if (DISCONNECT_2_CLOSE)
    {
      pStateMast = &StateClose;
    }
    else
    {
      pStateMast = &StateDisconnect;    // Stay in current state
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateClose
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateClose)
  {
    if (CLOSE_2_IDLE)
    {
      pStateMast = &StateIdle;
    }
    else
    {
      pStateMast = &StateClose;    // Stay in current state
    }
  }

//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Current state = undetermined
//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else
  {
    pStateMast = &StateInit;   // Go to Error state by default
  }
}


//===============================================================
// Name     : StateInit
// Purpose  : Initialization of the system.
//===============================================================
void StateInit(void)
{
  INTDisableInterrupts();   // Disable all interrupts of the system.

  INIT_PORTS;
//  INIT_WDT;
  INIT_TIMER;
  INIT_ADC;
  INIT_UART;
  INIT_SPI;
  INIT_PWM;
//  INIT_I2C;
  INIT_CAN;
  INIT_SKADI;
  START_INTERRUPTS;

  // Send ID to backplane by CAN protocol
  SEND_ID_TO_BACKPLANE;

//  Timer.DelayMs(10);
  
  // Send the mode of operation to the steering wheel
//  SEND_MODE_TO_STEERING_WHEEL;

  // Get last known position of the mast
//  ReadMastPosFromEeprom();
//  if (ABS(mastAngle.currentValue) > 360)  // Error
//  {
//    mastAngle.previousValue = 0;
//    mastAngle.currentValue  = 0;
//  }
  crabLeftZeroMm  = 367.14;
  crabRightZeroMm = 367.34;

  // Init registers for the drive
  InitDriver();
  
  DRVA_SLEEP = 1;
  DRVA_RESET = 0;
  DRVB_SLEEP = 1;
  DRVB_RESET = 0;
  
  UINT16 pwm2 = 500
        ,pwm3 = 500
        ,pwm4 = 500
        ,pwm5 = 500
        ;
  DRVA_SLEEP = 0;
  DRVB_SLEEP = 0;

  Pwm.SetDutyCycle(PWM_2, pwm2);    // DRVB
  Pwm.SetDutyCycle(PWM_3, pwm3);
  Pwm.SetDutyCycle(PWM_4, pwm4);    // DRVA
  Pwm.SetDutyCycle(PWM_5, pwm5);

  WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors at the drive
  WriteDrive(DRVA, STATUS_Mastw);   // Reset any errors at the drive
  
  // Code to adjust the actuators
  
//  Pwm.SetDutyCycle(DRV_LEFT_PWM1, 550);    // Left expand
//  Pwm.SetDutyCycle(DRV_LEFT_PWM2, 450);
////  Pwm.SetDutyCycle(DRV_LEFT_PWM1, 450);    // Left shrink
////  Pwm.SetDutyCycle(DRV_LEFT_PWM2, 550);
//  Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 550);    // Right expand
//  Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 450);
////  Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 450);    // Right shrink
////  Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 550);
//  
//  DRV_LEFT_SLEEP = 1;
//  DRV_RIGHT_SLEEP = 1;
//  
//  while(1);
}


//===============================================================
// Name     : StateManual
// Purpose  : Assess manual flags and adjust the mast in consequence
//===============================================================
void StateManual(void)
{
  oNewAdcMeasurement = 0;
  
  if (  ( leftActDeg < ACTUATOR_MIN_DEG) || ( leftActDeg > ACTUATOR_MAX_DEG) 
     || (rightActDeg < ACTUATOR_MIN_DEG) || (rightActDeg > ACTUATOR_MAX_DEG) 
     )
  {
    if ( (leftActDeg < ACTUATOR_MIN_DEG) || (leftActDeg > ACTUATOR_MAX_DEG) )
    {
      if (oManualLeftMoving)
      {
        DRV_LEFT_SLEEP = 0;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);

        oManualLeftMoving = 0;
        oManualLeftStopped = 1;
      }
    }
    
    if ( (rightActDeg < ACTUATOR_MIN_DEG) || (rightActDeg > ACTUATOR_MAX_DEG) )
    {
      if (oManualRightMoving)
      {
        DRV_RIGHT_SLEEP = 0;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);

        oManualRightMoving = 0;
        oManualRightStopped = 1;
      }
    }
  }
  else
  {
//    if ( ABS(leftActDeg - crabManualCmdDeg) <= CRAB_ERROR )
    if ( (leftActDeg <= (crabManualCmdDeg + CRAB_ERROR)) && (leftActDeg >= (crabManualCmdDeg - CRAB_ERROR)) )
    {
      if (oManualLeftMoving)
      {
        DRV_LEFT_SLEEP = 0;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);

        oManualLeftMoving = 0;
        oManualLeftStopped = 1;
      }
    }
    else if (leftActPos > crabManualCmdMmLeft)
    {
      if (oManualLeftStopped)
      {
        DRV_LEFT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 400);   // Shrink
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 600);

        oManualLeftStopped = 0;
        oManualLeftMoving  = 1;
      }
    }
    else if (leftActPos < crabManualCmdMmLeft)
    {
      if (oManualLeftStopped)
      {
        DRV_LEFT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 600);   // Expand
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 400);

        oManualLeftStopped = 0;
        oManualLeftMoving  = 1;
      }
    }

//    if ( ABS(rightActDeg - crabManualCmdDeg) <= CRAB_ERROR )
    if ( (rightActDeg <= (crabManualCmdDeg + CRAB_ERROR)) && (rightActDeg >= (crabManualCmdDeg - CRAB_ERROR)) )
    {
      if (oManualRightMoving)
      {
        DRV_RIGHT_SLEEP = 0;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);  // Stop
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);

        oManualRightMoving = 0;
        oManualRightStopped = 1;
      }
    }
    else if (rightActPos > crabManualCmdMmRight)
    {
      if (oManualRightStopped)
      {
        DRV_RIGHT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 400);   // Shrink
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 600);

        oManualRightStopped = 0;
        oManualRightMoving  = 1;
      }
    }
    else if (rightActPos < crabManualCmdMmRight)
    {
      if (oManualRightStopped)
      {
        DRV_RIGHT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 600);   // Expand
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 400);

        oManualRightStopped = 0;
        oManualRightMoving  = 1;
      }
    }
  }
}


//===============================================================
// Name     : StateReg
// Purpose  : Regulate the mast
//===============================================================
void StateReg(void)
{
  oTimerReg = 0;

  Regulator();
}


//===============================================================
// Name     : StateDisconnect
// Purpose  : Send a disconnect message to the backplane
//===============================================================
void StateDisconnect(void)
{
  if (mastCurrentSpeed != 0)
  {
    MastManualStop();
  }

  WriteMastPos2Eeprom();
  
  SEND_DISCONNECT_TO_BACKPLANE;
}


//===============================================================
// Name     : StateClose
// Purpose  : Close all peripherals and put device in sleep mode
//===============================================================
void StateClose(void)
{

  INTDisableInterrupts();   // Disable all interrupts of the system.

//  Wdt.Disable();
  
  LED_ALL_OFF();

  I2c.Close(I2C4);

  // DRIVE B
  //==========================================================
  if (USE_DRIVE_B == 1)
  {
    Pwm.Close(PWM_2);
    Pwm.Close(PWM_3);
  }
  //==========================================================

  // DRIVE A
  //==========================================================
  if (USE_DRIVE_A == 1)
  {
    Pwm.Close(PWM_4);
    Pwm.Close(PWM_5);
  }
  //==========================================================

  Spi.Close(SPI4);

//  Can.Close(CAN1);

  Uart.Close(UART6);

  Timer.Close(TIMER_1);
  Timer.Close(TIMER_2);
  Timer.Close(TIMER_3);
  Timer.Close(TIMER_5);

//  OSCCONSET = 0x10;         // Sleep mode

}


//===============================================================
// Name     : StateIdle
// Purpose  : Wait for power-off
//===============================================================
void StateIdle(void)
{
  return;
}


//===============================================================
// Name     : StateSendData
// Purpose  : Send useful data to other devices
//===============================================================
void StateSendData(void)
{
  oTimerSendData = 0;

  static UINT8 iCounterToTwoSec = 0;
  
//  SEND_CRAB_DIR_DEG;  // Via CAN bus

  // DRIVE B
  //==========================================================
  if (USE_DRIVE_B == 1)
  {
    WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors
  }
  //==========================================================

  // DRIVE A
  //==========================================================
  if (USE_DRIVE_A == 1)
  {
    WriteDrive(DRVA, STATUS_Mastw);   // Reset any errors
  }
  //==========================================================

  if (iCounterToTwoSec < 10)
  {
    iCounterToTwoSec++;
  }
  else
  {
    iCounterToTwoSec = 0;
    LED_DEBUG3_TOGGLE;

    if (SEND_DATA_TO_UART)
    {
      sUartLineBuffer_t buffer;
      buffer.length = sprintf ( buffer.buffer
//                              , "\n\rLeft angle\t= %f\n\rLeft Position\t= %f\n\rRight angle\t= %f\n\rRight Position\t= %f\n\rCommand\t\t= %f\n\r"
                              , "\n\rLeft angle\t= %f\n\rRight angle\t= %f\n\rCommand\t\t= %f\n\r"
//                              , "\n\rLeft angle\t= %f\n\rRight angle\t= %f\n\rCommand left\t= %f\n\rCommand right\t= %f\n\r"
//                              , "\n\rLeft pos\t= %f\n\rRight pos\t= %f\n\rCommand left\t= %f\n\rCommand right\t= %f\n\r"
                              , leftActDeg
//                              , leftActPos
                              , rightActDeg
//                              , rightActPos
                              , crabManualCmdDeg
//                              , crabManualCmdMmLeft
//                              , crabManualCmdMmRight
//                              , crabManualCmdMmLeft
//                              , crabManualCmdMmRight
                              );

      Uart.PutTxFifoBuffer(UART6, &buffer);
    }
    
//    WriteMastPos2Eeprom();
  }
}


//===============================================================
// Name     : StateAcq
// Purpose  : Get data from peripherals
//===============================================================
void StateAcq(void)
{
//  float tempWindAngle = 0;
  UINT32 adcLeft  = 0
        ,adcRight = 0
        ;
  
  UINT16 i = 0;
  
  static UINT16  adcMemLeft[150]  = {0}
                ,adcMemRight[150] = {0}
                ,iAdcSample       =  0
                ;
  
  if (oNewManualCmd)
  {
    oNewManualCmd = 0;
    
    if (crabManualCmdDeg < -15)
    {
      crabManualCmdDeg = -15;
    }
    
    if (crabManualCmdDeg > 15)
    {
      crabManualCmdDeg = 15;
    }
    
    CrabDegToMm(crabManualCmdDeg, &crabManualCmdMmLeft , LEFT_ACTUATOR );
    CrabDegToMm(crabManualCmdDeg, &crabManualCmdMmRight, RIGHT_ACTUATOR);
  }

//  if (oManualMode)
//  {
//    LED_DEBUG0_ON;
//  }
//  else
//  {
//    LED_DEBUG0_OFF;
//  }

//  if (oNewWindAngle)
//  {
//    nWindAngleSamples++;
//    memcpy ((void *) &tempWindAngle, (void *) &rxWindAngle, 4);  // Copy contents of UINT32 into float
//    meanWindAngle += tempWindAngle;
//  }
  
  if (oAdcReady)
  {
    oAdcReady = 0;
    
    adcMemLeft [iAdcSample] = Adc.Var.adcReadValues[3];
    adcMemRight[iAdcSample] = Adc.Var.adcReadValues[2];
    
    iAdcSample++;
    
    if (iAdcSample >= N_ADC_SAMPLES)
    {
      for (i = 0; i < iAdcSample; i++)
      {
        adcLeft   += adcMemLeft [i];
        adcRight  += adcMemRight[i];
      }
      
      adcLeft   = (float) adcLeft   / (float) iAdcSample + 0.5;
      adcRight  = (float) adcRight  / (float) iAdcSample + 0.5;
//      adcLeft  = Adc.Var.adcReadValues[3];
//      adcRight = Adc.Var.adcReadValues[2];

      CrabBitToMm(   adcLeft , &leftActPos , LEFT_ACTUATOR );
      CrabBitToMm(   adcRight, &rightActPos, RIGHT_ACTUATOR);

      CrabMmToDeg(leftActPos , &leftActDeg , LEFT_ACTUATOR );
      CrabMmToDeg(rightActPos, &rightActDeg, RIGHT_ACTUATOR);
      
      iAdcSample = 0;
      
      oNewAdcMeasurement = 1;
    }
  }

//  AssessButtons();

//  AssessMastValues();

//  UINT32 coreTickRate = Timer.Tic(1500, SCALE_US);
  Skadi.GetCmdMsgFifo();
//  INT32 time = Timer.Toc(1500, coreTickRate);
//  UINT8 test = 0;
}
