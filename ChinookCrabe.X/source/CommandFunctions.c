//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : StateFunctions.c
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the C file for the functions of the state machine of
//           the system.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : Function names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\CommandFunctions.h"
#include "..\headers\StateFunctions.h"


//==============================================================================
// Mast regulation variables
//==============================================================================

// Received from CAN
//=====================================
extern volatile UINT32 rxWindAngle;
//=====================================

// Actuator mm to deg table
//=====================================
const sDegToMm_t DEG_TO_MM = 
{
   .deg       = {   -35,    -34,    -33,    -32,    -31,    -30,    -29,    -28,   -27,    -26,    -25,    -24,    -23,    -22,    -21,    -20,    -19,    -18,    -17,  -16,     -15,    -14,    -13,    -12,    -11,    -10,     -9,     -8,     -7,     -6,     -5,     -4,     -3,     -2,     -1,      0,      1,      2,      3,      4,      5,      6,      7,      8,      9,     10,     11,     12,     13,     14,     15,    16,     17,     18,     19,     20,     21,     22,     23,     24,     25,     26,    27,     28,     29,     30,     31,     32,     33,     34,     35}
  ,.leftMm    = { 36.39,  38.83,  41.26,  43.68,   46.1,  48.51,  50.91,   53.3, 55.68,  58.05,  60.41,  62.75,  65.08,   67.4,   69.7,  71.99,  74.26,  76.51,  78.75, 80.97,  83.17,  85.36,  87.53,  89.67,   91.8,  93.91,     96,  98.07, 100.12, 102.15, 104.16, 106.15, 108.12, 110.06, 111.99, 113.89, 115.78, 117.64, 119.48,  121.3, 123.09, 124.87, 126.63, 128.36, 130.07, 131.76, 133.43, 135.08, 136.71, 138.32, 139.91, 141.7, 143.02, 144.55, 146.05, 147.54, 149.01, 150.46, 151.88, 153.29, 154.68, 156.05, 157.4, 158.74, 160.05, 161.35, 162.63, 163.89, 165.14, 166.36, 167.57}
  ,.rightMm   = {167.57, 166.36, 165.14, 163.89, 162.63, 161.35, 160.05, 158.74, 157.4, 156.05, 154.68, 153.29, 151.88, 150.46, 149.01, 147.54, 146.05, 144.55, 143.02, 141.7, 139.91, 138.32, 136.71, 135.08, 133.43, 131.76, 130.07, 128.36, 126.63, 124.87, 123.09,  121.3, 119.48, 117.64, 115.78, 113.89, 111.99, 110.06, 108.12, 106.15, 104.16, 102.15, 100.12,  98.07,     96,  93.91,   91.8,  89.67,  87.53,  85.36,  83.17, 80.97,  78.75,  76.51,  74.26,  71.99,   69.7,   67.4,  65.08,  62.75,  60.41,  58.05, 55.68,   53.3,  50.91,  48.51,   46.1,  43.68,  41.26,  38.83,  36.39}
  ,.leftZero  = 113.89
  ,.rightZero = 113.89
};
//=====================================


// Used for the average of the wind angle
//========================================
volatile UINT32 nWindAngleSamples = 0;
volatile float  meanWindAngle = 0;
//========================================


// Used when acquiring data from regulator
//=========================================
sCmdData_t    data        = {0};
volatile BOOL oPrintData  =  0;
//=========================================


// Mast general values
//=====================================
extern volatile float  mastCurrentSpeed      // Actual speed of Mast
                      ;

volatile sCmdValue_t windAngle          = {0}
                    ,mastAngle          = {0}
                    ,mastSpeed          = {0}
                    ;
//=====================================


// PI values
//=====================================
sCmdValue_t  inPi   = {0}
            ,outPi  = {0}
            ;

// Regulator parameters

/*
 * These are the tested working values WITH the mast attached to the motor
 * shaft, but WITHOUT the blades.
 */
volatile float KP = 0.010f
              ,KI = 0.010f
              ,K  = 0.100f
              ,PWM_MAX_DUTY_CYCLE = 0.900f
              ,PWM_MIN_DUTY_CYCLE = 0.030f
              ,ERROR_THRESHOLD    = 4.000f
              ,T                  = 0.100f    // Same as TIMER_1
              ;
/*
 * These are the tested working values WITHOUT the mast attached to the motor
 * shaft.
 */
//volatile float KP = 0.015f
//              ,KI = 0.030f
//              ,K  = 0.300f
//              ,PWM_MAX_DUTY_CYCLE = 0.980f
//              ,PWM_MIN_DUTY_CYCLE = 0.010f
//              ,ERROR_THRESHOLD    = 0.100f
////              ,T                  = 0.100f    // Same as TIMER_1
//              ,T                  = 1.000f    // Same as TIMER_1
//              ;
//=====================================


// Flags
//=====================================
extern volatile BOOL oCapture1
                    ,oCapture2
                    ,oCapture3
                    ,oCapture4
                    ,oTimerReg
                    ;

BOOL  oFirstTimeInMastStop    = 0
     ,oEmergencyStop          = 0
     ;
//=====================================


//==============================================================================
// Mast regulation functions
//==============================================================================

/*
 * Function : TustinZ
 * Desc :     Discrete integrator using Tustin's method
 * Graphic example :
 *
 *   1     T     z + 1
 *  --- = --- * -------
 *   s     2     z - 1
 *
 *         _______
 *  x(n)  |   1   | y(n)
 * ------>| ----- |------>
 *        |   s   |
 *        |_______|
 *
 *  iLaplace => y(n) = y(n-1) + T/2 * ( x(n-1) + x(n) )
 *
 */
void TustinZ (sCmdValue_t *input, sCmdValue_t *output)
{
  output->previousValue = output->currentValue;
  output->currentValue  = output->previousValue + T/2 * (input->currentValue + input->previousValue);
}


// Adjust the PWM of the motor
void SetPwm (float cmd)
{
  if (cmd == 0)
  {
    if (oEmergencyStop)   // When mast has gone beyond the acceptable limits
    {
      MastManualStop();
      oEmergencyStop = 0;

      // Reset PI internal values
      inPi.currentValue = 0;
      inPi.previousValue = 0;
      outPi.currentValue = 0;
      outPi.previousValue = 0;
      LED_DEBUG4_TOGGLE;
    }
    else if (oFirstTimeInMastStop)  // Do this procedure only once after every movement of the mast
    {

      // DRIVE B
      //==========================================================
      if (USE_DRIVE_B == 1)
      {
        DRVB_SLEEP = 0;
        Pwm.SetDutyCycle(PWM_2, 500);
        Pwm.SetDutyCycle(PWM_3, 500);
      }
      //==========================================================

      // DRIVE A
      //==========================================================
      if (USE_DRIVE_A == 1)
      {
        DRVA_SLEEP = 0;
        Pwm.SetDutyCycle(PWM_4, 500);
        Pwm.SetDutyCycle(PWM_5, 500);
      }
      //==========================================================

      mastCurrentSpeed = 0;
      mastSpeed.previousValue = 0;
      mastSpeed.currentValue = 0;

      oFirstTimeInMastStop = 0;

      // Reset PI internal values
      inPi.currentValue = 0;
      inPi.previousValue = 0;
      outPi.currentValue = 0;
      outPi.previousValue = 0;

      // DRIVE B
      //==========================================================
      if (USE_DRIVE_B == 1)
      {
        WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors
      }
      //==========================================================

      // DRIVE A
      //==========================================================
      if (USE_DRIVE_A == 1)
      {
        WriteDrive(DRVA, STATUS_Mastw);   // Reset any errors
      }
      //==========================================================
      
      WriteMastPos2Eeprom();

      if (oPrintData)   // Used for debugging with skadi
      {
        UINT16 i;
        INT32 err = 0;
        sUartLineBuffer_t buffer;
        buffer.length = sprintf(buffer.buffer, "\n\ri\tpSeed\tSpeed\tpPos\tPos\tpWind\tWind\tError\tpInPi\tinPi\tpOutPi\tOutPi\tcmd\n\r");
        Uart.PutTxFifoBuffer(UART6, &buffer);
        for (i = 0; i < data.length; i++)
        {
          buffer.length = sprintf(buffer.buffer, "%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n\r", i, data.speedPrevious[i], data.speedCurrent[i], data.posPrevious[i]
                  ,data.posCurrent[i],data.windPrevious[i], data.windCurrent[i], data.error[i], data.inPiPrevious[i], data.inPiCurrent[i], data.outPiPrevious[i]
                  ,data.outPiCurrent[i], data.cmd[i]);
          do
          {
            err = Uart.PutTxFifoBuffer(UART6, &buffer);
          } while (err < 0);
        }
        buffer.buffer[0] = '\n';
        buffer.length = 1;
        do
        {
          err = Uart.PutTxFifoBuffer(UART6, &buffer);
        } while (err < 0);
        
        data.length = 0;
      }
    }
  }
  else // if (cmd != 0)
  {
    oFirstTimeInMastStop = 1;
    
    UINT16 pwm = ABS(cmd) * 500;

    if (SIGN(cmd) == MAST_DIR_LEFT)
    {
      // DRIVE B
      //==========================================================
      if (USE_DRIVE_B == 1)
      {
        DRVB_SLEEP = 1;
        Pwm.SetDutyCycle(PWM_2, 500 + pwm);
        Pwm.SetDutyCycle(PWM_3, 500 - pwm);
      }
      //==========================================================

      // DRIVE A
      //==========================================================
      if (USE_DRIVE_A == 1)
      {
        DRVA_SLEEP = 1;
        Pwm.SetDutyCycle(PWM_4, 500 + pwm);
        Pwm.SetDutyCycle(PWM_5, 500 - pwm);
      }
      //==========================================================
    }
    else if (SIGN(cmd) == MAST_DIR_RIGHT)
    {
      // DRIVE B
      //==========================================================
      if (USE_DRIVE_B == 1)
      {
        DRVB_SLEEP = 1;
        Pwm.SetDutyCycle(PWM_2, 500 - pwm);
        Pwm.SetDutyCycle(PWM_3, 500 + pwm);
      }
      //==========================================================

      // DRIVE A
      //==========================================================
      if (USE_DRIVE_A == 1)
      {
        DRVA_SLEEP = 1;
        Pwm.SetDutyCycle(PWM_4, 500 - pwm);
        Pwm.SetDutyCycle(PWM_5, 500 + pwm);
      }
      //==========================================================
    }
  }
}


/*
 *  ____________     ___  error  ___     ___  inPi  _______ outPi ____         ___     ________   __________      _______
 * | wind angle |-> /+  \ ----->| K |-> /+  \ ---> | 1 / s |---->| KI |-----> /+  \ ->| Motor |->| Encodeur |--->| 1 / s |__
 * |____________|   \_-_/       |___|   \_-_/  |   |_______|     |____|       \_+_/   |_______|  |__________| |  |_______|  |
 *                    ^                   ^    |                  ____          ^                             |             |
 *                    |                   |    |                 | KP |         |                             |             |
 *                    |                   |     ---------------->|____|---------             mastCurrentSpeed |             |
 *                    |                   |___________________________________________________________________|             |
 *                    |_____________________________________________________________________________________________________|
 *                                                                                        mastCurrentPos
 */
void Regulator (void)
{
  float  cmd
        ,error
        ,tempWind
        ;

  // Update wind angle
  windAngle.previousValue = windAngle.currentValue;
  
//  memcpy ((void *) &tempWind, (void *) &rxWindAngle, 4);  // Copy contents of UINT32 into float

  if (nWindAngleSamples != 0)
  {
    tempWind = meanWindAngle / nWindAngleSamples;
    meanWindAngle = 0;
    nWindAngleSamples = 0;
  }
  else  // If no new sample was received
  {
    tempWind = windAngle.previousValue;   // Keep previous value as current value
  }

  /*
   * If the wind is not in the acceptable range, change the command to the MAX
   * or MIN. Mast must not go beyond these values.
   */
  if (tempWind > MAST_MAX)
  {
    windAngle.currentValue = MAST_MAX;
  }
  else if (tempWind < MAST_MIN)
  {
    windAngle.currentValue = MAST_MIN;
  }
  else // if (tempWind != windAngle.currentValue)
  {
    windAngle.currentValue = tempWind;
  }

  // Update mast speed
  mastSpeed.previousValue = mastSpeed.currentValue;
  mastSpeed.currentValue  = mastCurrentSpeed;

  // Get mast position from mast speed
  TustinZ((void *) &mastSpeed, (void *) &mastAngle);  // Discrete integrator

  /*
   * Some kind of modulo
   */
  if (mastAngle.currentValue > 180)
  {
    mastAngle.currentValue -= 360;
  }
  else if (mastAngle.currentValue < -180)
  {
    mastAngle.currentValue += 360;
  }

  /*
   * The rest of the code goes like the graphic at the top of this function.
   * The only parts added adjust the command from PWM_MIN_DUTY_CYCLE to
   * PWM_MAX_DUTY_CYCLE. The algorithm also checks if the mast has passed the
   * limits MAST_MIN and MAST_MAX.
   */

  error = windAngle.currentValue - mastAngle.currentValue;
  
  if (ABS(error) <= ERROR_THRESHOLD)  // Don't need to move the mast
  {
    cmd = 0;
  }
  else if ( (SIGN(mastSpeed.currentValue) == MAST_DIR_LEFT) && (!MAST_MIN_OK) )   // Mast too far
  {
    oEmergencyStop = 1;
    cmd = 0;
  }
  else if ( (SIGN(mastSpeed.currentValue) == MAST_DIR_RIGHT) && (!MAST_MAX_OK) && (mastSpeed.currentValue != 0) )  // Mast too far
  {
    oEmergencyStop = 1;
    cmd = 0;
  }
  else
  {
    oEmergencyStop = 0;
    oFirstTimeInMastStop = 1;

    error -= SIGN(error) * ERROR_THRESHOLD;   // Substract the ERROR_THRESHOLD to reduce the risk of an abrupt stop by the mast

    inPi.previousValue = inPi.currentValue;
    inPi.currentValue  = K * error - mastSpeed.currentValue;

    TustinZ((void *) &inPi, (void *) &outPi);

    cmd = inPi.currentValue * KP + outPi.currentValue * KI;

    if      (ABS(cmd) > PWM_MAX_DUTY_CYCLE)
    {
      cmd = SIGN(cmd) * PWM_MAX_DUTY_CYCLE;
    }
    else if (ABS(cmd) < PWM_MIN_DUTY_CYCLE)
    {
      cmd = SIGN(cmd) * PWM_MIN_DUTY_CYCLE;
    }
  }

  if (oPrintData && oFirstTimeInMastStop)   // Used for debugging with skadi
  {
    if (data.length < N_DATA_TO_ACQ)
    {
      data.cmd            [data.length] = cmd;
      data.error          [data.length] = error;
      data.inPiCurrent    [data.length] = inPi.currentValue;
      data.inPiPrevious   [data.length] = inPi.previousValue;
      data.outPiCurrent   [data.length] = outPi.currentValue;
      data.outPiPrevious  [data.length] = outPi.previousValue;
      data.posCurrent     [data.length] = mastAngle.currentValue;
      data.posPrevious    [data.length] = mastAngle.previousValue;
      data.speedCurrent   [data.length] = mastSpeed.currentValue;
      data.speedPrevious  [data.length] = mastSpeed.previousValue;
      data.windCurrent    [data.length] = windAngle.currentValue;
      data.windPrevious   [data.length] = windAngle.previousValue;
      data.length++;
    }
  }

  SetPwm(cmd);
}


//==============================================================================
// Input Capture functions
//==============================================================================
void AssessMastValues (void)
{
  INT64  rx1
        ,rx2
        ,rx3
        ,rx4
        ;
  INT8 firstIc;
  UINT64 meanTime   = 0;
  INT8 dir    = 0;

  /*
   * If a capture event has occured on both inputs, check the time of both and
   * get the direction of the mast (MAST_DIR_LEFT or MAST_DIR_RIGHT). Then, using
   * the known ratios of ENCODER / MOTOR / MAST, calculate the speed of the mast
   * in degrees / second [deg/s].
   */

  // DRIVE B
  //============================================================================
  if (USE_DRIVE_B == 1)
  {
    if (oCapture2 && oCapture4)
    {
      oCapture2 = 0;
      oCapture4 = 0;

      rx2 = InputCapture.GetTimeBetweenCaptures(IC2, SCALE_US);
      rx4 = InputCapture.GetTimeBetweenCaptures(IC4, SCALE_US);

      if ( !((rx2 > 2000000) || (rx4 > 2000000)) )  // It would mean 0.34 deg/s for the motor shaft, consider it zero
      {
        firstIc = InputCapture.GetDirection(IC2, IC4, rx4, SCALE_US);

        if (firstIc == IC2)
        {
          dir = MAST_DIR_LEFT;
        }
        else if (firstIc == IC4)
        {
          dir = MAST_DIR_RIGHT;
        }

  //      meanTime = (rx2 + rx4) / 2;
        meanTime = (rx2 + rx4) >> 1;   // Divide by 2

        float mastTime = SIGN(dir) * (float) meanTime * TIMER_SCALE_US;

        if (mastTime == 0)
        {
          mastCurrentSpeed = 0;
        }
        else
        {
          mastCurrentSpeed = MOTOR_DEG_PER_PULSE / (mastTime * MAST_MOTOR_RATIO);
        }
      }
      else
      {
        mastCurrentSpeed = 0;
      }
    }
  }
  //============================================================================


  // DRIVE A
  //============================================================================
  if (USE_DRIVE_A == 1)
  {
    if (oCapture1 && oCapture3)
    {
      oCapture1 = 0;
      oCapture3 = 0;

      rx1 = InputCapture.GetTimeBetweenCaptures(IC1, SCALE_US);
      rx3 = InputCapture.GetTimeBetweenCaptures(IC3, SCALE_US);

      if ( !((rx1 > 2000000) || (rx3 > 2000000)) )  // It would mean 0.34 deg/s for the motor shaft, consider it zero
      {
        firstIc = InputCapture.GetDirection(IC1, IC3, rx3, SCALE_US);

        if (firstIc == IC1)
        {
          dir = MAST_DIR_LEFT;
        }
        else if (firstIc == IC3)
        {
          dir = MAST_DIR_RIGHT;
        }

  //      meanTime = (rx1 + rx3) / 2;
        meanTime = (rx1 + rx3) >> 1;   // Divide by 2

        float mastTime = SIGN(dir) * (float) meanTime * TIMER_SCALE_US;

        if (mastTime == 0)
        {
          mastCurrentSpeed = 0;
        }
        else
        {
          mastCurrentSpeed = MOTOR_DEG_PER_PULSE / (mastTime * MAST_MOTOR_RATIO);
        }
      }
      else
      {
        mastCurrentSpeed = 0;
      }
    }
  }
  //============================================================================
}

void CrabMmToDeg (float mm, float *deg)
{
  UINT8 i, iMm;
  float slope;
  
  for (i = 0; i < 71; i++)
  {
    if (DEG_TO_MM.leftMm[i] >= mm)
    {
      iMm = i;
      i = 72;
    }
  }
  
//  if (iMm > 0)
//  {
//    deg = (mm - DEG_TO_MM.deg[iMm - 1]) / (DEG_TO_MM.leftMm[iMm] - DEG_TO_MM.leftMm[iMm - 1]) + DEG_TO_MM.deg[iMm - 1];
//  }
//  else
//  {
//    deg = DEG_TO_MM.deg[0];
//  }
  
  *deg = DEG_TO_MM.deg[iMm];
}