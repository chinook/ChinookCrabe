//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : SourceTemplate.c
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This C file gathers the functions used for Skadi.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : Function names can and should be renamed by the user to improve the
//           readability of the code. User must set the functions here and adjust
//           the structure skadiCommandTable located in Setup.c.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\SkadiFunctions.h"
#include "..\headers\CommandFunctions.h"
#include "..\headers\StateFunctions.h"


//==============================================================================
// Private functions prototypes
//==============================================================================


//==============================================================================
// Variable definitions
//==============================================================================
extern volatile sCmdValue_t  windAngle
                            ;

extern volatile float  crabManualCmdDeg
                      ;

extern volatile UINT32 rxWindAngle;

extern volatile BOOL   oManualMode
                      ,oNewWindAngle
                      ,oNewManualCmd
                      ;

//==============================================================================
// Functions
//==============================================================================

/**************************************************************
 * Function name  : LedDebug
 * Purpose        : Toggle LedDebug depending on the arguments
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void LedDebug(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  int led = atoi(args.elements[0]);   // Convert argument to int

  if (led == 0)
  {
    LED_DEBUG0_TOGGLE;
  }
  else if (led == 1)
  {
    LED_DEBUG1_TOGGLE;
  }
  else if (led == 2)
  {
    LED_DEBUG2_TOGGLE;
  }
  else if (led == 3)
  {
    LED_DEBUG3_TOGGLE;
  }
  else if (led == 4)
  {
    LED_DEBUG4_TOGGLE;
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Cette led n'existe pas!\r\n\n");
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
}


/**************************************************************
 * Function name  : GetMode
 * Purpose        : Send the current mast mode of operation
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void GetMode(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;
  
  if (oManualMode)
  {
    buffer.length = sprintf(buffer.buffer, "Mast is in Manual Mode (oManualMode == 1)\r\n\n");
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mast is in Automatic Mode (oManualMode == 0)\r\n\n");
  }

  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : ClearScreen
 * Purpose        : Clear the terminal window
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void ClearScreen(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;
  buffer.buffer[0] = '\n';
  buffer.buffer[1] = '\r';
  UINT8 i;

  for (i = 2; i < 50; i++)
  {
    buffer.buffer[i] = '\n';
  }
  buffer.length = i;
  
  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : WriteStatus
 * Purpose        : Write STATUS msg to the drive
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void WriteStatus(sSkadi_t *skadi, sSkadiArgs_t args)
{
  WriteDrive(DRVB, STATUS_Mastw);
  sUartLineBuffer_t buffer;
  buffer.length = sprintf(buffer.buffer, "STATUS msg written to drive\r\n\n");
  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : GetWind
 * Purpose        : Send the wind angle
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void GetWind(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;
  buffer.length = sprintf(buffer.buffer, "WindAngle = %f\r\n\n", windAngle.currentValue);
  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : SetCrabManualCmd
 * Purpose        : Set the manual cmd for the crab [angle]
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void SetCrabManualCmd(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  float wind = atof(args.elements[0]);   // Convert argument to float

  if ((wind >= -15) && (wind <= 15))
  {
    memcpy((void *) &crabManualCmdDeg, (void *) &wind, 4);
    oNewManualCmd = 1;
    buffer.length = sprintf(buffer.buffer, "Crab manual cmd = %f\r\n\n", wind);
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
}


/**************************************************************
 * Function name  : SetWind
 * Purpose        : Set the wind angle
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void SetWind(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  float wind = atof(args.elements[0]);   // Convert argument to float

  if ((wind >= -179) && (wind <= 179))
  {
    memcpy((void *) &rxWindAngle, (void *) &wind, 4);
    oNewWindAngle = 1;
    buffer.length = sprintf(buffer.buffer, "WindAngle = %f\r\n\n", wind);
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
}


/**************************************************************
 * Function name  : LedError
 * Purpose        : Toggle LedError
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void LedError(sSkadi_t *skadi, sSkadiArgs_t args)
{
  LED_ERROR_TOGGLE;
}


/**************************************************************
 * Function name  : LedStatus
 * Purpose        : Toggle LedStatus
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void LedStatus(sSkadi_t *skadi, sSkadiArgs_t args)
{
  LED_STATUS_TOGGLE;
}


/**************************************************************
 * Function name  : LedCan
 * Purpose        : Toggle LedCan
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void LedCan(sSkadi_t *skadi, sSkadiArgs_t args)
{
  LED_CAN_TOGGLE;
}


/**************************************************************
 * Function name  : ReInitSystem
 * Purpose        : Redo StateInit()
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void ReInitSystem(sSkadi_t *skadi, sSkadiArgs_t args)
{
  StateInit();
  pStateMast = &StateInit;
}