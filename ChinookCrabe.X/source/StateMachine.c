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

extern volatile float tempCrabManualCmdDeg;

float crabManualCmdDeg = 0;

extern float crabLeftZeroMm
            ,crabRightZeroMm
            ;

float  crabManualCmdMmLeft  = 370.12
      ,crabManualCmdMmRight = 372.38
      ,leftActPosMm         = 0
      ,rightActPosMm        = 0
      ,leftActDeg           = 0
      ,rightActDeg          = 0
      ,leftActMinPosMm      = 0
      ,leftActMaxPosMm      = 0
      ,rightActMinPosMm     = 0
      ,rightActMaxPosMm     = 0
      ;

BOOL oNewAdcMeasurement = 0;

volatile BOOL oNewManualCmd = 1;

BOOL oManualLeftLowerLim    = 0
    ,oManualRightLowerLim   = 0
    ,oManualLeftUpperLim    = 0
    ,oManualRightUpperLim   = 0
    ,oManualLeftStopped     = 1
    ,oManualRightStopped    = 1
    ;

ActuatorMoveFlags_t  leftActMoves  = NEEDS_TO_STOP
                    ,rightActMoves = NEEDS_TO_STOP
                    ;

// Used for the average of the wind angle
//========================================
extern volatile UINT32 nWindAngleSamples;
extern volatile float  meanWindAngle;
//========================================

extern volatile sCmdValue_t windAngle
                           ;

extern volatile BOOL oAdcReady
                    ,oNewWindAngle
                    ,oTimerSendData
                    ;

volatile BOOL  oManualMode            = 1
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
    if (ACQ_2_MANUAL)
    {
      pStateMast = &StateManual;
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
  if (SEND_DATA_TO_UART)
  {
    INIT_UART;
  }
  INIT_SPI;
  INIT_PWM;
//  INIT_I2C;
  INIT_CAN;
//  INIT_SKADI;
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
  
  LED_DEBUG0_OFF;
  LED_DEBUG1_OFF;
  
  CrabDegToMm(ACTUATOR_MIN_DEG, &leftActMinPosMm , LEFT_ACTUATOR );
  CrabDegToMm(ACTUATOR_MAX_DEG, &leftActMaxPosMm , LEFT_ACTUATOR );
  CrabDegToMm(ACTUATOR_MIN_DEG, &rightActMinPosMm, RIGHT_ACTUATOR);
  CrabDegToMm(ACTUATOR_MAX_DEG, &rightActMaxPosMm, RIGHT_ACTUATOR);
}


//===============================================================
// Name     : StateManual
// Purpose  : Assess manual flags and adjust the mast in consequence
//===============================================================
void StateManual(void)
{
  oNewAdcMeasurement = 0;
  
  // Check for limits
  // =====================================
  if (leftActDeg <= ACTUATOR_MIN_DEG)
//  if (leftActPosMm <= leftActMinPosMm)
  {
    oManualLeftLowerLim = 1;
  }
  else
  {
    oManualLeftLowerLim = 0;
  }
  if (leftActDeg >= ACTUATOR_MAX_DEG)
//  if (leftActPosMm >= leftActMaxPosMm)
  {
    oManualLeftUpperLim = 1;
  }
  else
  {
    oManualLeftUpperLim = 0;
  }
  
  if (rightActDeg <= ACTUATOR_MIN_DEG)
//  if (rightActPosMm <= rightActMinPosMm)
  {
    oManualRightLowerLim = 1;
  }
  else
  {
    oManualRightLowerLim = 0;
  }
  if (rightActDeg >= ACTUATOR_MAX_DEG)
//  if (rightActPosMm >= rightActMaxPosMm)
  {
    oManualRightUpperLim = 1;
  }
  else
  {
    oManualRightUpperLim = 0;
  }
  // *************************************
  
  // Moving cases
  // =====================================
  if (AbsFloat(leftActDeg - crabManualCmdDeg) <= CRAB_ERROR)
  {
    leftActMoves = NEEDS_TO_STOP;
  }
  else
  {
    if (leftActDeg > crabManualCmdDeg)
    {
      if (!oManualLeftLowerLim)
      {
        leftActMoves = NEEDS_TO_SHRINK;
      }
      else
      {
        leftActMoves = NEEDS_TO_STOP;
      }
    }
    else
    {
      if (!oManualLeftUpperLim)
      {
        leftActMoves = NEEDS_TO_EXPAND;
      }
      else
      {
        leftActMoves = NEEDS_TO_STOP;
      }
    }
  }
  
  if (AbsFloat(rightActDeg - crabManualCmdDeg) <= CRAB_ERROR)
  {
    rightActMoves = NEEDS_TO_STOP;
  }
  else
  {
    if (rightActDeg > crabManualCmdDeg)
    {
      if (!oManualRightLowerLim)
//      if (!oManualRightUpperLim)
      {
        rightActMoves = NEEDS_TO_EXPAND;
      }
      else
      {
        rightActMoves = NEEDS_TO_STOP;
      }
    }
    else
    {
      if (!oManualRightUpperLim)
//      if (!oManualRightLowerLim)
      {
        rightActMoves = NEEDS_TO_SHRINK;
      }
      else
      {
        rightActMoves = NEEDS_TO_STOP;
      }
    }
  }
  // *************************************
  
  // Move left actuator
  // =====================================
  switch (leftActMoves)
  {
    case NEEDS_TO_SHRINK :
      if (oManualLeftStopped)
      {
        DRV_LEFT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 400);   // Shrink
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 600);

        oManualLeftStopped = 0;
      }
      break;
    
    case NEEDS_TO_EXPAND :
      if (oManualLeftStopped)
      {
        DRV_LEFT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 600);   // Expand
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 400);

        oManualLeftStopped = 0;
      }
      break;
      
    case NEEDS_TO_STOP :
    default :
      DRV_LEFT_SLEEP = 0;
      if (!oManualLeftStopped)
      {
        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);

        oManualLeftStopped = 1;
      }
      break;
  }
  
//  if (oLeftActNeedsToStop)
//  {
//    DRV_LEFT_SLEEP = 0;
//    if (!oManualLeftStopped)
//    {
//      Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
//      Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);
//
//      oManualLeftStopped = 1;
//    }
//  }
//  else if (oLeftActNeedsToShrink)
//  {
//    if (oManualLeftStopped)
//    {
//      DRV_LEFT_SLEEP = 1;
//      Pwm.SetDutyCycle(DRV_LEFT_PWM1, 400);   // Shrink
//      Pwm.SetDutyCycle(DRV_LEFT_PWM2, 600);
//
//      oManualLeftStopped = 0;
//    }
//  }
//  else if (oLeftActNeedsToExpand)
//  {
//    if (oManualLeftStopped)
//    {
//      DRV_LEFT_SLEEP = 1;
//      Pwm.SetDutyCycle(DRV_LEFT_PWM1, 600);   // Expand
//      Pwm.SetDutyCycle(DRV_LEFT_PWM2, 400);
//
//      oManualLeftStopped = 0;
//    }
//  }
//  else
//  {
//    DRV_LEFT_SLEEP = 0;
//    if (!oManualLeftStopped)
//    {
//      Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
//      Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);
//
//      oManualLeftStopped = 1;
//    }
//  }
  // *************************************
  
  // Move Right actuator
  // =====================================
  switch (rightActMoves)
  {
    case NEEDS_TO_SHRINK :
      if (oManualRightStopped)
      {
        DRV_RIGHT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 400);   // Shrink
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 600);

        oManualRightStopped = 0;
      }
      break;
      
    case NEEDS_TO_EXPAND :
      if (oManualRightStopped)
      {
        DRV_RIGHT_SLEEP = 1;
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 600);   // Expand
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 400);

        oManualRightStopped = 0;
      }
      break;
      
    case NEEDS_TO_STOP :
    default :
      DRV_RIGHT_SLEEP = 0;
      if (!oManualRightStopped)
      {
        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);

        oManualRightStopped = 1;
      }
      break;
  }
//  if (oRightActNeedsToStop)
//  {
//    DRV_RIGHT_SLEEP = 0;
//    if (!oManualRightStopped)
//    {
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);
//
//      oManualRightStopped = 1;
//    }
//  }
//  else if (oRightActNeedsToShrink)
//  {
//    if (oManualRightStopped)
//    {
//      DRV_RIGHT_SLEEP = 1;
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 400);   // Shrink
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 600);
//
//      oManualRightStopped = 0;
//    }
//  }
//  else if (oRightActNeedsToExpand)
//  {
//    if (oManualRightStopped)
//    {
//      DRV_RIGHT_SLEEP = 1;
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 600);   // Expand
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 400);
//
//      oManualRightStopped = 0;
//    }
//  }
//  else
//  {
//    DRV_RIGHT_SLEEP = 0;
//    if (!oManualRightStopped)
//    {
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);
//
//      oManualRightStopped = 1;
//    }
//  }
  // *************************************
  
//  // Left actuator
//  // =====================================
//  if ( (leftActDeg <= (crabManualCmdDeg + CRAB_ERROR)) && (leftActDeg >= (crabManualCmdDeg - CRAB_ERROR)) )
//  {
//    if (!oManualLeftStopped)
//    {
//      DRV_LEFT_SLEEP = 0;
//      Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
//      Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);
//
//      oManualLeftStopped = 1;
//    }
//  }
//  else if (leftActPosMm > crabManualCmdMmLeft)
//  {
//    if (oManualLeftLowerLim)
//    {
//      if (!oManualLeftStopped)
//      {
//        DRV_LEFT_SLEEP = 0;
//        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
//        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);
//
//        oManualLeftStopped = 1;
//      }
//    }
//    else
//    {
//      if (oManualLeftStopped)
//      {
//        DRV_LEFT_SLEEP = 1;
//        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 400);   // Shrink
//        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 600);
//
//        oManualLeftStopped = 0;
//      }
//    }
//  }
//  else if (leftActPosMm < crabManualCmdMmLeft)
//  {
//    if (oManualLeftUpperLim)
//    {
//      if (!oManualLeftStopped)
//      {
//        DRV_LEFT_SLEEP = 0;
//        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 500);
//        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 500);
//
//        oManualLeftStopped = 1;
//      }
//    }
//    else
//    {
//      if (oManualLeftStopped)
//      {
//        DRV_LEFT_SLEEP = 1;
//        Pwm.SetDutyCycle(DRV_LEFT_PWM1, 600);   // Expand
//        Pwm.SetDutyCycle(DRV_LEFT_PWM2, 400);
//
//        oManualLeftStopped = 0;
//      }
//    }
//  }
//  // *************************************
//
//  // Right actuator
//  // =====================================
//  if ( (rightActDeg <= (crabManualCmdDeg + CRAB_ERROR)) && (rightActDeg >= (crabManualCmdDeg - CRAB_ERROR)) )
//  {
//    if (!oManualRightStopped)
//    {
//      DRV_RIGHT_SLEEP = 0;
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);  // Stop
//      Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);
//
//      oManualRightStopped = 1;
//    }
//  }
//  else if (rightActPosMm > crabManualCmdMmRight)
//  {
//    if (oManualRightUpperLim)
//    {
//      if (!oManualRightStopped)
//      {
//        DRV_RIGHT_SLEEP = 0;
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);
//
//        oManualRightStopped = 1;
//      }
//    }
//    else
//    {
//      if (oManualRightStopped)
//      {
//        DRV_RIGHT_SLEEP = 1;
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 400);   // Shrink
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 600);
//
//        oManualRightStopped = 0;
//      }
//    }
//  }
//  else if (rightActPosMm < crabManualCmdMmRight)
//  {
//    if (oManualRightLowerLim)
//    {
//      if (!oManualRightStopped)
//      {
//        DRV_RIGHT_SLEEP = 0;
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 500);
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 500);
//
//        oManualRightStopped = 1;
//      }
//    }
//    else
//    {
//      if (oManualRightStopped)
//      {
//        DRV_RIGHT_SLEEP = 1;
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM1, 600);   // Expand
//        Pwm.SetDutyCycle(DRV_RIGHT_PWM2, 400);
//
//        oManualRightStopped = 0;
//      }
//    }
//  }
//  // *************************************
}


//===============================================================
// Name     : StateSendData
// Purpose  : Send useful data to other devices
//===============================================================
void StateSendData(void)
{
  oTimerSendData = 0;

  static UINT8 iCounterToTwoSec = 0;
  
  SEND_CRAB_LEFT_WHEEL_DEG;  // Via CAN bus
  SEND_CRAB_RIGHT_WHEEL_DEG; // Via CAN bus

  // DRIVE B
  //==========================================================
  WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors
  //==========================================================

  // DRIVE A
  //==========================================================
  WriteDrive(DRVA, STATUS_Mastw);   // Reset any errors
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
//                              , "\n\rLeft angle\t= %f\n\rRight angle\t= %f\n\rCommand\t\t= %f\n\r"
                              , "\n\rLeft angle\t= %f\n\rRight angle\t= %f\n\rCommand\t\t= %f\n\rRightActLowLim\t\t= %d\n\rRightActUpLim\t\t= %d\n\rLeftActLowLim\t\t= %d\n\rLeftActUpLim\t\t= %d\n\rRightStopped\t\t= %d\n\rLeftStopped\t\t= %d\n\r"
//                              , "\n\rLeft angle\t= %f\n\rRight angle\t= %f\n\rCommand left\t= %f\n\rCommand right\t= %f\n\r"
//                              , "\n\rLeft pos\t= %f\n\rRight pos\t= %f\n\rCommand left\t= %f\n\rCommand right\t= %f\n\r"
                              , leftActDeg
//                              , leftActPosMm
                              , rightActDeg
//                              , rightActPosMm
                              , crabManualCmdDeg
                              , oManualRightLowerLim
                              , oManualRightUpperLim
                              , oManualLeftLowerLim
                              , oManualLeftUpperLim
                              , oManualRightStopped
                              , oManualLeftStopped
//                              , crabManualCmdMmLeft
//                              , crabManualCmdMmRight
//                              , crabManualCmdMmLeft
//                              , crabManualCmdMmRight
                              );

      Uart.PutTxFifoBuffer(UART6, &buffer);
    }
  }
}


//===============================================================
// Name     : StateAcq
// Purpose  : Get data from peripherals
//===============================================================
void StateAcq(void)
{
//  float tempWindAngle = 0;
//  UINT32 adcLeft  = 0
//        ,adcRight = 0
//        ;
  UINT16 adcLeft  = 0
        ,adcRight = 0
        ;
  
  if (oNewManualCmd && oManualLeftStopped && oManualRightStopped)
  {
    oNewManualCmd = 0;
    
    crabManualCmdDeg = tempCrabManualCmdDeg;
    
    if (crabManualCmdDeg < ACTUATOR_MIN_DEG)
    {
      crabManualCmdDeg = ACTUATOR_MIN_DEG;
    }
    
    if (crabManualCmdDeg > ACTUATOR_MAX_DEG)
    {
      crabManualCmdDeg = ACTUATOR_MAX_DEG;
    }
    
    CrabDegToMm(crabManualCmdDeg, &crabManualCmdMmLeft , LEFT_ACTUATOR );
    CrabDegToMm(crabManualCmdDeg, &crabManualCmdMmRight, RIGHT_ACTUATOR);
  }
  
  if (oAdcReady)
  {
    oAdcReady = 0;
    
    adcLeft  = Adc.Var.adcReadValues[3];
    adcRight = Adc.Var.adcReadValues[2];

    CrabBitToMm(   adcLeft , &leftActPosMm , LEFT_ACTUATOR );
    CrabBitToMm(   adcRight, &rightActPosMm, RIGHT_ACTUATOR);

    CrabMmToDeg(leftActPosMm , &leftActDeg , LEFT_ACTUATOR );
    CrabMmToDeg(rightActPosMm, &rightActDeg, RIGHT_ACTUATOR);

    oNewAdcMeasurement = 1;
  }

//  AssessButtons();

//  UINT32 coreTickRate = Timer.Tic(1500, SCALE_US);
//  Skadi.GetCmdMsgFifo();
//  INT32 time = Timer.Toc(1500, coreTickRate);
//  UINT8 test = 0;
}
