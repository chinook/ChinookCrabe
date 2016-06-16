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

float  crabManualCmdDeg     = 0
      ,crabManualCmdMmLeft  = 0
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

volatile BOOL oNewManualCmd = 0;

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
  // Current state = StateGetMastData
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateGetMastData)
  {
    if (GET_MAST_DATA_2_ACQ)
    {
      pStateMast = &StateAcq;
    }
    else
    {
      pStateMast = &StateGetMastData;    // Stay in current state
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

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateIdle
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pStateMast == &StateIdle)
  {
    if (IDLE_2_INIT)
    {
      pStateMast = &StateInit;
    }
    else
    {
      pStateMast = &StateIdle;    // Stay in current state
    }
  }

//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Current state = undetermined
//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else
  {
    pStateMast = &StateInit;   // Go to Error state by default
  }
  /*
   * DEVELOPPER CODE HERE
   */
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
//  INIT_SKADI;
  INIT_SPI;
  INIT_PWM;
//  INIT_I2C;
  INIT_CAN;
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
}


//===============================================================
// Name     : StateManual
// Purpose  : Assess manual flags and adjust the mast in consequence
//===============================================================
void StateManual(void)
{
  oNewAdcMeasurement = 0;
  
  if (  (leftActDeg < ACTUATOR_MIN_DEG) || (leftActDeg > ACTUATOR_MAX_DEG) 
     || (leftActDeg < ACTUATOR_MIN_DEG) || (leftActDeg > ACTUATOR_MAX_DEG) 
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
    
    if ( (leftActDeg < ACTUATOR_MIN_DEG) || (leftActDeg > ACTUATOR_MAX_DEG) )
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
    if ( ABS(leftActPos - crabManualCmdMmLeft) <= CRAB_ERROR )
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
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 300);   // Shrink
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 700);

        oManualLeftStopped = 0;
        oManualLeftMoving  = 1;
      }
    }
    else if (leftActPos < crabManualCmdMmLeft)
    {
      if (oManualLeftStopped)
      {
        DRV_LEFT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 700);   // Expand
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 300);

        oManualLeftStopped = 0;
        oManualLeftMoving  = 1;
      }
    }

    if ( ABS(rightActPos - crabManualCmdMmRight) <= CRAB_ERROR )
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
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 300);   // Expand
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 700);

        oManualRightStopped = 0;
        oManualRightMoving  = 1;
      }
    }
    else if (rightActPos < crabManualCmdMmRight)
    {
      if (oManualRightStopped)
      {
        DRV_RIGHT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 700);   // Shrink
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 300);

        oManualRightStopped = 0;
        oManualRightMoving  = 1;
      }
    }
  }
}


//===============================================================
// Name     : StateGetMastData
// Purpose  : Get position of mast if in manual mode
//===============================================================
void StateGetMastData(void)
{
  oTimerReg = 0;
  
  // Update wind direction
  windAngle.previousValue = windAngle.currentValue;

  float tempWind;
//  memcpy ((void *) &tempWind, (void *) &rxWindAngle, 4);

  if (nWindAngleSamples != 0)
  {
    tempWind = meanWindAngle / nWindAngleSamples;
    meanWindAngle = 0;
    nWindAngleSamples = 0;
  }
  else  // If no new sample was received
  {
    tempWind = windAngle.previousValue;   // Keep previous value as current value
  }

  /*
   * If the wind is not in the acceptable range, change the command to the MAX
   * or MIN. Mast must not go beyond these values.
   */
  if (tempWind > MAST_MAX)
  {
    windAngle.currentValue = MAST_MAX;
  }
  else if (tempWind < MAST_MIN)
  {
    windAngle.currentValue = MAST_MIN;
  }
  else if (tempWind != windAngle.currentValue)
  {
    windAngle.currentValue = tempWind;
  }

  // Update mast speed
  mastSpeed.previousValue = mastSpeed.currentValue;
  mastSpeed.currentValue  = mastCurrentSpeed;

  // Get mast position from mast speed
  TustinZ((void *) &mastSpeed, (void *) &mastAngle);    // Discrete integrator

  /*
   * Some kind of modulo
   */
  if (mastAngle.currentValue > 180)
  {
    mastAngle.currentValue -= 360;
  }
  else if (mastAngle.currentValue < -180)
  {
    mastAngle.currentValue += 360;
  }

  /*
   * Check mast limits
   */
  if (mastSpeed.currentValue != 0)
  {
    if ( (SIGN(mastSpeed.currentValue) == MAST_DIR_LEFT) && (!MAST_MIN_OK) )        // Mast too far
    {
      MastManualStop();
    }
    else if ( (SIGN(mastSpeed.currentValue) == MAST_DIR_RIGHT) && (!MAST_MAX_OK) )  // Mast too far
    {
      MastManualStop();
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

//  static UINT8 iCounterToTwoSec = 0;
  
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

//  if (iCounterToTwoSec < 10)
//  {
//    iCounterToTwoSec++;
//  }
//  else
//  {
//    iCounterToTwoSec = 0;
//    LED_DEBUG3_TOGGLE;
//
//    if (SEND_DATA_TO_UART)
//    {
//      sUartLineBuffer_t buffer;
//      buffer.length = sprintf ( buffer.buffer
//                              , "\n\rCurrent pos\t\t= %f\n\rCurrent wind\t\t= %f\n\r"
//  //                            , "\n\rCurrent speed\t\t= %f\n\rCurrent pos\t\t= %f\n\rCurrent wind\t\t= %f\n\r"
//  //                            , mastSpeed.currentValue
//                              , mastAngle.currentValue
//                              , windAngle.currentValue
//                              );
//
//      Uart.PutTxFifoBuffer(UART6, &buffer);
//    }
//    
//    WriteMastPos2Eeprom();
//  }
}


//===============================================================
// Name     : StateAcq
// Purpose  : Get data from peripherals
//===============================================================
void StateAcq(void)
{
//  float tempWindAngle = 0;
  UINT16 adcLeft
        ,adcRight
        ;
  
  if (oNewManualCmd)
  {
    oNewManualCmd = 0;
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
    oNewAdcMeasurement = 1;
    
    adcLeft  = Adc.Var.adcReadValues[2];
    adcRight = Adc.Var.adcReadValues[3];
    
    CrabBitToMm(   adcLeft , &leftActPos , LEFT_ACTUATOR );
    CrabBitToMm(   adcRight, &rightActPos, RIGHT_ACTUATOR);
    
    CrabMmToDeg(leftActPos , &leftActDeg , LEFT_ACTUATOR );
    CrabMmToDeg(rightActPos, &rightActDeg, RIGHT_ACTUATOR);
  }

//  AssessButtons();

//  AssessMastValues();

//  UINT32 coreTickRate = Timer.Tic(1500, SCALE_US);
//  Skadi.GetCmdMsgFifo();
//  INT32 time = Timer.Toc(1500, coreTickRate);
//  UINT8 test = 0;
}
