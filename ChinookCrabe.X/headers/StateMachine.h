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
void StateDisconnect  (void);     // Send disconnect msg to backplane
void StateClose       (void);     // Close all peripheral and enter sleep mode
void StateSendData    (void);     // Send various data to other devices
void StateReg         (void);     // Regulate the mast
void StateIdle        (void);     // Wait for power-down
void StateGetMastData (void);     // Get position of mast if in manual mode
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
#define MAST_MAX               90.0f
#define MAST_MIN              -90.0f

/*********** BASIC CONDITION *****************/
#define MAST_LEFT             (oManualMastLeft)
#define MAST_RIGHT            (oManualMastRight)
#define MAST_OK               (!oManualMastLeft && !oManualMastRight)
#define MAST_MAX_OK           (mastAngle.currentValue <= MAST_MAX)
#define MAST_MIN_OK           (mastAngle.currentValue >= MAST_MIN)
#define MAST_DIR_DOWN         SW1
#define MAST_DIR_UP           !MAST_DIR_DOWN
#define MANUAL_MODE           oManualMode
#define MANUAL_FLAG_CHANGE    oManualFlagChng
#define DISCONNECT_OK         0                                   // Need to be coded
#define REG_TIMER_OK          oTimerReg
#define SEND_DATA_TIMER_OK    oTimerSendData

#define PULSE_PER_DEGREE      30

//==============================================================================
// STATE TRANSITIONS
//==============================================================================

/******* TRANSITION CONDITION INIT **********/
#define INIT_2_ACQ            1


/******* TRANSITION CONDITION CALIB **********/
#define CALIB_2_ACQ           1


/******* TRANSITION CONDITION ACQ **********/
#define ACQ_2_MANUAL           MANUAL_MODE && MANUAL_FLAG_CHANGE
#define ACQ_2_DISCONNECT       DISCONNECT_OK
#define ACQ_2_REG             !MANUAL_MODE && REG_TIMER_OK
#define ACQ_2_GET_MAST_DATA    MANUAL_MODE && REG_TIMER_OK
#define ACQ_2_SEND_DATA        SEND_DATA_TIMER_OK


/******* TRANSITION CONDITION GET MAST DATA **********/
#define GET_MAST_DATA_2_ACQ    1


/******* TRANSITION CONDITION SEND DATA **********/
#define SEND_DATA_2_ACQ        1


/******* TRANSITION CONDITION MANUAL **********/
#define MANUAL_2_ACQ           1


/******* TRANSITION CONDITION REG **********/
#define REG_2_ACQ              1


/******* TRANSITION CONDITION DISCONNECT **********/
#define DISCONNECT_2_CLOSE     1


/******* TRANSITION CONDITION CLOSE **********/
#define CLOSE_2_IDLE           1


/******* TRANSITION CONDITION IDLE **********/
#define IDLE_2_INIT            0


#endif	/* STATE_MACHINE_MAST_H */

