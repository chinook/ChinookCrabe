//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateFunctions.h
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the header file for the functions of the state machine
//           of the system.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : Function names can and should be renamed by the user to improve the
//           readability of the code. User must set the functions here and adjust
//           the structure skadiCommandTable located in Setup.c.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef __STATE_FUNCTIONS_H__
#define	__STATE_FUNCTIONS_H__

#include "Setup.h"
#include "DRV8711_Para.h"


//==============================================================================
// State Machine public functions prototypes
//==============================================================================

// EEPROM functions
// =======================================
void  WriteMastPos2Eeprom   (void);
void ReadMastPosFromEeprom (void);
// =======================================


// Mast manual functions
// =======================================
void MastManualStop  (void);
void MastManualLeft  (void);
void MastManualRight (void);
// =======================================


// Buttons functions
// =======================================
void AssessButtons (void);
// =======================================


//==============================================================================
// Macro definitions
//==============================================================================

// Define this at one if you want to send the mast values via UART each (TIMER_4). Else, 0
#define SEND_DATA_TO_UART     0

// Typedef for mapping the steering wheel switches and the buttons on the board
// All the buttons used : 3 on the steering wheel, 3 on the board
typedef union
{
  struct
  {
    UINT8  steerWheelSw1  : 1
          ,steerWheelSw3  : 1
          ,steerWheelSw10 : 1
          ,boardSw1       : 1
          ,boardSw2       : 1
          ,boardSw3       : 1
          ,               : 2
          ;
  } bits;

  UINT8 byte;

} Buttons_t;

typedef union
{
  struct
  {
    UINT8  steerWheelSw1  : 1
          ,steerWheelSw3  : 1
          ,steerWheelSw10 : 1
          ,boardSw1       : 1
          ,boardSw2       : 1
          ,boardSw3       : 1
          ,               : 2
          ;
  } bits;

  UINT8 byte;

} Chng_t;

typedef struct sButtonStates
{
  Buttons_t buttons;
  Chng_t    chng;
} sButtonStates_t;


//==============================================================================
// Variable declarations
//==============================================================================


#endif	/* __STATE_FUNCTIONS_H__ */

