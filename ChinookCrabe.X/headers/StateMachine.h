//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook V
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateMachine_Mast.h
// Author  : Amaury LAINE
// Date    : 2015-03-13
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : Header with all condition/transition between all states
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : NA
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef STATE_MACHINE_MAST_H
#define	STATE_MACHINE_MAST_H

#include "Setup.h"
//#include "StateFunctions.h"


//==============================================================================
// State Machine public function prototypes
//==============================================================================
void StateInit        (void);     // Initialization state of the system
void StateManual      (void);     // Assess manual flags and adjust the mast in consequence
void StateAcq         (void);     // Get data from peripherals
void StateSendData    (void);     // Send various data to other devices
void StateScheduler   (void);     // State Scheduler. Decides which state is next



//==============================================================================
// Variable declarations
//==============================================================================
void (*pStateMast)(void);       // State pointer, used to navigate between states

//==============================================================================
// Macro definitions
//==============================================================================

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// State scheduler flags
// The state scheduler is at the end of each state and
// decides which state is next. Following flags
// are used in this decision. The names used can and
// should be renamed to improve readability. Also, the
// conditions tested in the defines should be changed
// to proper tests
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/*********** LIMITS **/
#define ACTUATOR_MIN_DEG      -15.0f
#define ACTUATOR_MAX_DEG       15.0f

/*********** BASIC CONDITION *****************/
#define MANUAL_MODE           oManualMode
#define NEW_MEASUREMENT       oNewAdcMeasurement
#define SEND_DATA_TIMER_OK    oTimerSendData

#define N_ADC_SAMPLES         100

//==============================================================================
// STATE TRANSITIONS
//==============================================================================

/******* TRANSITION CONDITION INIT **********/
#define INIT_2_ACQ            1


/******* TRANSITION CONDITION CALIB **********/
#define CALIB_2_ACQ           0//1


/******* TRANSITION CONDITION ACQ **********/
#define ACQ_2_MANUAL           MANUAL_MODE && NEW_MEASUREMENT
#define ACQ_2_SEND_DATA        SEND_DATA_TIMER_OK


/******* TRANSITION CONDITION SEND DATA **********/
#define SEND_DATA_2_ACQ        1


/******* TRANSITION CONDITION MANUAL **********/
#define MANUAL_2_ACQ           1


#endif	/* STATE_MACHINE_MAST_H */

