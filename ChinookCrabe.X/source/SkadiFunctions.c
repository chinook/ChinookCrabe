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
extern volatile sCmdValue_t  mastAngle
                            ,windAngle
                            ;

extern volatile float  mastCurrentSpeed
                      ,KP
                      ,KI
                      ,K
                      ,PWM_MAX_DUTY_CYCLE
                      ,PWM_MIN_DUTY_CYCLE
                      ,ERROR_THRESHOLD
                      ;

extern volatile UINT32 rxWindAngle;

extern volatile BOOL   oManualMode
                      ,oPrintData
                      ,oNewWindAngle
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
 * Function name  : SetMode
 * Purpose        : Set the mast mode of operation
 * Arguments      : 1 : Manual mode
 *                  0 : Auto mode
 * Returns        : None.
 *************************************************************/
void SetMode(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  UINT8 mode = atoi(args.elements[0]);   // Convert argument to int

  if (mode == 0)    // Auto mode
  {
    if (oManualMode)
    {
      oManualMode = 0;
      SEND_MODE_TO_STEERING_WHEEL;
      buffer.length = sprintf(buffer.buffer, "Mast is in Automatic Mode (oManualMode == 0)\r\n\n");
    }
    else
    {
      buffer.length = sprintf(buffer.buffer, "Mast is already in Automatic Mode!! (oManualMode == 0)\r\n\n");
    }
  }
  else if (mode == 1)    // Manual mode
  {
    if (!oManualMode)
    {
      oManualMode = 1;
      if (mastCurrentSpeed != 0)
      {
        MastManualStop();
      }
      SEND_MODE_TO_STEERING_WHEEL;
      buffer.length = sprintf(buffer.buffer, "Mast is in Manual Mode (oManualMode == 1)\r\n\n");
    }
    else
    {
      buffer.length = sprintf(buffer.buffer, "Mast is already in Manual Mode!! (oManualMode == 1)\r\n\n");
    }
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
  }
  
  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : GetParam
 * Purpose        : Send the current mast parameters K, KI, KP
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void GetParam(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);

  Uart.PutTxFifoBuffer(UART6, &buffer);
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
 * Function name  : GetSpeed
 * Purpose        : Send the current mast speed [deg/s]
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void GetSpeed(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;
  buffer.length = sprintf(buffer.buffer, "MastSpeed = %f\r\n\n", mastCurrentSpeed);
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
 * Function name  : GetPos
 * Purpose        : Send the mast angle
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void GetPos(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;
  buffer.length = sprintf(buffer.buffer, "MastAngle = %f\r\n\n", mastAngle.currentValue);
  Uart.PutTxFifoBuffer(UART6, &buffer);
}


/**************************************************************
 * Function name  : SetPos
 * Purpose        : Set the mast angle
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void SetPos(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

//  float mast = atoi(args.elements[0]);   // Convert argument to int
  float mast = atof(args.elements[0]);   // Convert argument to float

//  if ((mast >= -1790) && (mast <= 1790))
  if ((mast >= -179) && (mast <= 179))
  {
//    mast /= 10.0f;
    mastAngle.currentValue  = mast;
    mastAngle.previousValue = mast;
    mastCurrentSpeed = 0;
    if (mast == 0)
    {
      SEND_CALIB_DONE;
    }
    buffer.length = sprintf(buffer.buffer, "MastAngle = %f\r\n\n", mastAngle.currentValue);
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
    Uart.PutTxFifoBuffer(UART6, &buffer);
  }
}


/**************************************************************
 * Function name  : SetParam
 * Purpose        : Adjust one of the regulator parameters
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void SetParam(sSkadi_t *skadi, sSkadiArgs_t args)
{
  sUartLineBuffer_t buffer;

  float value = atof(args.elements[1]);   // Convert argument to float

  UINT8 kStr[]  = "K\0"
       ,kiStr[] = "KI\0"
       ,kpStr[] = "KP\0"
       ,pmaxStr[] = "PWM_MAX\0"
       ,pminStr[] = "PWM_MIN\0"
       ,errStr[] = "ERROR\0"
       ;

  if (value < 0)
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
  }
  else if (!strcmp(kStr, args.elements[0]))
  {
    K = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else if (!strcmp(kiStr, args.elements[0]))
  {
    KI = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else if (!strcmp(kpStr, args.elements[0]))
  {
    KP = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else if (!strcmp(pmaxStr, args.elements[0]))
  {
    PWM_MAX_DUTY_CYCLE = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else if (!strcmp(pminStr, args.elements[0]))
  {
    PWM_MIN_DUTY_CYCLE = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else if (!strcmp(errStr, args.elements[0]))
  {
    ERROR_THRESHOLD = value;
    buffer.length = sprintf(buffer.buffer, "\nK\t= %.4f\r\nKI\t= %.4f\r\nKP\t= %.4f\r\nERROR\t= %.4f\r\nPWM_MAX\t= %.4f\r\nPWM_MIN\t= %.4f\r\n\n", K, KI, KP, ERROR_THRESHOLD, PWM_MAX_DUTY_CYCLE, PWM_MIN_DUTY_CYCLE);
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
  }

  Uart.PutTxFifoBuffer(UART6, &buffer);
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
 * Function name  : SetPrint
 * Purpose        : Tells the software to print or not the
 *                  regulation data.
 * Arguments      : Received from Skadi functions
 * Returns        : None.
 *************************************************************/
void SetPrint(sSkadi_t *skadi, sSkadiArgs_t args)
{
  UINT8 oPrint = atoi(args.elements[0]);   // Convert argument to int
  sUartLineBuffer_t buffer;

  if (oPrint == 1)
  {
    if (oPrintData)
    {
      buffer.length = sprintf(buffer.buffer, "Already in printing mode!!\r\n\n");
    }
    else
    {
      oPrintData = 1;
      buffer.length = sprintf(buffer.buffer, "Program will print data, now.\r\n\n");
    }
  }
  else if (oPrint == 0)
  {
    if (!oPrintData)
    {
      buffer.length = sprintf(buffer.buffer, "Already not printing data!!\r\n\n");
    }
    else
    {
      oPrintData = 0;
      buffer.length = sprintf(buffer.buffer, "Program won't print data, now.\r\n\n");
    }
  }
  else
  {
    buffer.length = sprintf(buffer.buffer, "Mauvais argument!\r\n\n");
  }

  Uart.PutTxFifoBuffer(UART6, &buffer);
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