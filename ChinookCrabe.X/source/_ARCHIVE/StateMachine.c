//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateMachine.c
// Author  : Frederic Chasse
// Date    : 2015-02-25
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the C file for the state machine of the system.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : State names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\StateMachine.h"


//==============================================================================
//	STATES OF STATE MACHINE
//==============================================================================

//===============================================================
// Name     : StateScheduler
// Purpose  : Decides which state is next depending on current
//            state and flags. Used as a function
//===============================================================
void StateScheduler(void)
{
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateInit
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if (pState == &StateInit)
  {
    if (INIT_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (INIT_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (INIT_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (INIT_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (INIT_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else if (INIT_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateInit;   // Go to Error state by default
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateStop
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateStop)
  {
    if (STOP_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (STOP_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (STOP_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (STOP_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (STOP_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else if (STOP_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateStop;   // Go to Error state by default
    }
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateCalib
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateCalib)
  {
    if (CALIB_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (CALIB_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (CALIB_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (CALIB_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (CALIB_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else if (CALIB_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateCalib;   // Go to Error state by default
    }
  }

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateDown
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateDown)
  {
    if (DOWN_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (DOWN_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (DOWN_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (DOWN_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (DOWN_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else if (DOWN_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateDown;   // Go to Error state by default
    }
  }
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateUp
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateUp)
  {
    if (UP_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (UP_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (UP_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (UP_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (UP_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else if (UP_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateUp;   // Go to Error state by default
    }
  }
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Current state = StateError
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateError)
  {
    if (ERROR_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (ERROR_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (ERROR_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (ERROR_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (ERROR_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (ERROR_2_BREAK)
    {
      pState = &StateBreak;     // Error state
    }
    else
    {
      pState = &StateError;   // Go to Error state by default
    }
  }
//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Current state = StateBreak
//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else if (pState == &StateBreak)
  {
    if (BREAK_2_INIT)
    {
      pState = &StateInit;       // Stay in inition state
    }
    else if (BREAK_2_STOP)
    {
      pState = &StateStop;       // stop state
    }
    else if (BREAK_2_CALIB)
    {
      pState = &StateCalib;     // Error state
    }
    else if (BREAK_2_DOWN)
    {
      pState = &StateDown;     // Error state
    }
    else if (BREAK_2_UP)
    {
      pState = &StateUp;     // Error state
    }
    else if (BREAK_2_ERROR)
    {
      pState = &StateError;     // Error state
    }
    else
    {
      pState = &StateBreak;   // Go to Error state by default
    }
  }



//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Current state = undetermined
//  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  else
  {
    pState = &StateError;   // Go to Error state by default
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
//  INIT_INPUT_CAPTURE;
//  INIT_PWM;
//  INIT_ADC;
//  INIT_UART;
//  INIT_CAN;
  INIT_SPI;
//  INIT_I2C;
  START_INTERRUPTS;

}

//===============================================================
// Name     : StateStop
// Purpose  : TBD.
//===============================================================
void StateStop(void)
{
  LED_DEBG3 = 1;
  LED_DEBG4 = 1;

}

//===============================================================
// Name     : StateCalib
// Purpose  : TBD.
//===============================================================
void StateCalib(void)
{


}

//===============================================================
// Name     : StateDown
// Purpose  : Error state of the system. Used to assess and
//            correct errors in the system.
//===============================================================
void StateDown(void)
{
  DRVA_DIR = PITCH_DOWN;
  DRVA_STEP = 1;
  Pitch_now--;
  LED_DEBG3 = 0;
  LED_DEBG4 = 1;

}

//===============================================================
// Name     : StateUp
// Purpose  : Error state of the system. Used to assess and
//            correct errors in the system.
//===============================================================
void StateUp(void)
{
  DRVA_DIR = PITCH_UP;
  DRVA_STEP = 1;
  Pitch_now++;
  LED_DEBG3 = 1;
  LED_DEBG4 = 0;

}

//===============================================================
// Name     : StateError
// Purpose  : Error state of the system. Used to assess and
//            correct errors in the system.
//===============================================================
void StateError(void)
{
  LED_ERROR = LED_ACTIVE;

}

//===============================================================
// Name     : StateBreak
// Purpose  : Error state of the system. Used to assess and
//            correct errors in the system.
//===============================================================
void StateBreak(void)
{


}