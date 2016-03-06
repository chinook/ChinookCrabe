//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Project : ChinookMast
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : CommandFunctions.h
// Author  : Frederic Chasse
// Date    : 2015-06-21
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the header file for the functions of the Mast regulation
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : Function names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef __COMMAND_FUNCTIONS__
#define	__COMMAND_FUNCTIONS__

#include "Setup.h"
#include "StateMachine.h"


//==============================================================================
// Macro definitions
//==============================================================================
#define MAST_DIR_LEFT               -1
#define MAST_DIR_RIGHT               1

#define MOTOR_ENCODER_RATIO         49.0f
#define MAST_MOTOR_RATIO            50.0f

#define MOTOR_DEG_PER_PULSE         360.0f/245.0f

#define N_DATA_TO_ACQ               300   // Used when debugging with SKADI

#define VREF_PLUS                   3.0f
#define ACTUATOR_MAX_POS            167.57f   // in mm
#define ACTUATOR_MAX_VOLT           2.702f    // 2.702f for the other
#define ACTUATOR_MIN_POS            0.116f    // 0.119f for the other
#define ACTUATOR_MIN_VOLT           0.3f


/*
 * Structure used for the Tustin discrete integrators
 */
typedef struct sCmdValue
{
  float  previousValue
        ,currentValue
        ;
} sCmdValue_t;


/*
 * Actuator degree to mm table
 */
typedef struct sDegToMm
{
  float deg [71];
  
  float leftMm  [71];
  float leftZero;
  
  float rightMm [71];
  float rightZero;
  
} sDegToMm_t;


//==============================================================================
// Mast regulation public functions prototypes
//==============================================================================
void TustinZ    (sCmdValue_t *input, sCmdValue_t *output);
void SetPwm     (float cmd);
void Regulator  (void);

// Input capture functions
// =======================================
void AssessMastValues (void);
// =======================================

// Various MATH functions
#define ABS(x)  ( (x >= 0)?  x : -x )
#define SIGN(x) ( (x >= 0)?  1 : -1 )


// Used for debugging with SKADI
typedef struct
{
  float windPrevious  [N_DATA_TO_ACQ];
  float windCurrent   [N_DATA_TO_ACQ];
  float posPrevious   [N_DATA_TO_ACQ];
  float posCurrent    [N_DATA_TO_ACQ];
  float speedPrevious [N_DATA_TO_ACQ];
  float speedCurrent  [N_DATA_TO_ACQ];
  float error         [N_DATA_TO_ACQ];
  float inPiPrevious  [N_DATA_TO_ACQ];
  float inPiCurrent   [N_DATA_TO_ACQ];
  float outPiPrevious [N_DATA_TO_ACQ];
  float outPiCurrent  [N_DATA_TO_ACQ];
  float cmd           [N_DATA_TO_ACQ];
  UINT16 length;
} sCmdData_t;

#endif	/* __COMMAND_FUNCTIONS__ */

