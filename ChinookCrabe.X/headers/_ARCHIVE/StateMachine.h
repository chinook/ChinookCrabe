//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateMachine.h
// Author  : Frederic Chasse
// Date    : 2015-02-25
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the header file for the state machine of the system.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : State names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef STATE_MACHINE_H
#define	STATE_MACHINE_H

#include "Setup.h"
#include "StateFunctions.h"


//==============================================================================
// State Machine public function prototypes
//==============================================================================
void StateInit(void);           // Initialization state of the system
void StateStop(void);           // Stop pitch if pitch is on good position
void StateCalib(void);          // Second state. User can rename it as needed
void StateDown(void);           // Error state. User should assess and corret errors in this state
void StateUp(void);             // Error state. User should assess and corret errors in this state
void StateError(void);          // Error state. User should assess and corret errors in this state
void StateBreak(void);          // Error state. User should assess and corret errors in this state
void StateScheduler(void);      // State Scheduler. Decides which state is next

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

INT32 Pitch_now;                    // Position of real of pitch in number of pitch
INT32 Pitch_consigne;               // COnsigne for pitch, in number of pitch

INT32 AERO_BRAKE;               // pour test
INT32 calibration;

/*********** PITCH STATE MACHINE PARAMETERS **/
#define PITCH_HYST_EXTREM    10
#define PITCH_MIN            -5
#define PITCH_MAX            60
#define PITCH_UP             1
#define PITCH_DOWN           !PITCH_UP
#define LED_ACTIVE           0
#define LED_UNACTIVE         !LED_ACTIVE



/*********** BASIC CONDITION *****************/
#define POS_DOWN             (Pitch_now > Pitch_consigne)
#define POS_UP               (Pitch_now < Pitch_consigne)
#define POS_OK               (Pitch_now = Pitch_consigne)
#define POS_MIN              (Pitch_now <= PITCH_MIN+PITCH_HYST_EXTREM)
#define POS_MAX              (Pitch_now >= PITCH_MAX-PITCH_HYST_EXTREM)
#define MODE_CALIB           PORTEbits.RE5     //calibration     // From volant
#define AERO_BRAKE           PORTEbits.RE6       //!calibration    // From volant


/******* TRANSITION CONDITION  INIT **********/
#define INIT_2_STOP          SW3 && SW3 //!AERO_BRAKE && !MODE_CALIB && POS_OK
#define INIT_2_CALIB         0 //!AERO_BRAKE &&  MODE_CALIB
#define INIT_2_DOWN          !SW2//!AERO_BRAKE && !MODE_CALIB && POS_DOWN
#define INIT_2_UP            !SW3//!AERO_BRAKE && !MODE_CALIB && POS_UP
#define INIT_2_ERROR         0 //!AERO_BRAKE
#define INIT_2_BREAK         0 //AERO_BRAKE

/******* TRANSITION CONDITION  STOP **********/
#define STOP_2_INIT          0
#define STOP_2_CALIB         0 //!AERO_BRAKE && MODE_CALIB
#define STOP_2_DOWN          !SW2 //!AERO_BRAKE && !MODE_CALIB && POS_DOWN
#define STOP_2_UP            !SW3 //!AERO_BRAKE && !MODE_CALIB && POS_UP
#define STOP_2_ERROR         0 //!AERO_BRAKE
#define STOP_2_BREAK         0 //AERO_BRAKE

/******* TRANSITION CONDITION  CALIB *********/
#define CALIB_2_INIT         0
#define CALIB_2_STOP         0 //!AERO_BRAKE & POS_OK
#define CALIB_2_DOWN         0 //!AERO_BRAKE
#define CALIB_2_UP           0 //!AERO_BRAKE
#define CALIB_2_ERROR        0 //!AERO_BRAKE
#define CALIB_2_BREAK        0 //AERO_BRAKE

/******* TRANSITION CONDITION  DOWN **********/
#define DOWN_2_INIT          0
#define DOWN_2_STOP          0 //!AERO_BRAKE && POS_OK
#define DOWN_2_CALIB         0 //!AERO_BRAKE && MODE_CALIB
#define DOWN_2_UP            !SW3//!AERO_BRAKE && !MODE_CALIB && POS_UP
#define DOWN_2_ERROR         0 //!AERO_BRAKE
#define DOWN_2_BREAK         0 //AERO_BRAKE

/******* TRANSITION CONDITION  UP ************/
#define UP_2_INIT            0 //0
#define UP_2_STOP            0 //!AERO_BRAKE && POS_OK
#define UP_2_CALIB           0 //!AERO_BRAKE && MODE_CALIB
#define UP_2_DOWN            !SW2//!AERO_BRAKE && !MODE_CALIB && POS_DOWN
#define UP_2_ERROR           0 //!AERO_BRAKE
#define UP_2_BREAK           0 //AERO_BRAKE

/******* TRANSITION CONDITION  ERROR *********/
#define ERROR_2_INIT         0
#define ERROR_2_STOP         0 //!AERO_BRAKE
#define ERROR_2_CALIB        0 //!AERO_BRAKE && MODE_CALIB
#define ERROR_2_DOWN         0 //!AERO_BRAKE
#define ERROR_2_UP           0 //!AERO_BRAKE
#define ERROR_2_BREAK        0 //AERO_BRAKE

/******* TRANSITION CONDITION  BREAK *********/
#define BREAK_2_INIT         0
#define BREAK_2_STOP         0 //!AERO_BRAKE && POS_OK
#define BREAK_2_CALIB        0 //!AERO_BRAKE && MODE_CALIB
#define BREAK_2_DOWN         0 //!AERO_BRAKE
#define BREAK_2_UP           0 //!AERO_BRAKE
#define BREAK_2_ERROR        0 //AERO_BRAKE

//==============================================================================
// Variable declarations
//==============================================================================
void (*pState)(void);       // State pointer, used to navigate between states
volatile  INT8  breakFlag   // Flag indicating if the emergency break has been pressed
               ,errorFlag   // Flag indicating an error
               ;

#endif	/* STATE_MACHINE_H */

