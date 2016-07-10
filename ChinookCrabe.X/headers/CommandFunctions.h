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

#define VREF_PLUS                   2.992f
#define ACTUATOR_MAX_POS            167.57f   // in mm
#define ACTUATOR_MAX_VOLT           2.695f    // 2.702f for the other
#define ACTUATOR_MIN_POS            0.116f    // 0.119f for the other
#define ACTUATOR_MIN_VOLT           0.3f

#define CRAB_ERROR                  0.85f      // Degrees


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
typedef struct sMmToDeg
{
  float deg [71];
  
  float leftMm  [71];
  
  float rightMm [71];
  
} sMmToDeg_t;

typedef struct sVoltToMm
{
  float leftVolt[174];
  float rightVolt[196];
  UINT16 leftBit[174];
  UINT16 rightBit[196];
  float leftMm[174];
  float rightMm[196];
  float leftOffset;
  float rightOffset;
} sVoltToMm_t;

typedef enum
{
  LEFT_ACTUATOR
 ,RIGHT_ACTUATOR
} CrabActuator_t;


//==============================================================================
// Regulation public functions prototypes
//==============================================================================
void TustinZ    (sCmdValue_t *input, sCmdValue_t *output, float acqTime);


//==============================================================================
// Crab public functions prototypes
//==============================================================================
void Interpol2D (float x0, float y0, float x1, float y1, float x, float *y);
void CrabVoltToMm (float volt, float *mm, CrabActuator_t act);
INT8 CrabDegToMm (INT8 deg, float *mm, CrabActuator_t act);
void CrabMmToDeg (float mm, float *deg, CrabActuator_t act);
void CrabBitToMm (UINT16 bit, float *mm, CrabActuator_t act);


// Various MATH functions
#define ABS(x)  ( (x >= 0)?  x : -x )
#define SIGN(x) ( (x >= 0)?  1 : -1 )

typedef enum
{
  NEEDS_TO_STOP
 ,NEEDS_TO_SHRINK
 ,NEEDS_TO_EXPAND
} ActuatorMoveFlags_t;


#endif	/* __COMMAND_FUNCTIONS__ */

