//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : main.c
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This is the main C file of the template project used by all
//           developpers of Chinook. It uses ChinookLib, which is another
//           repository on Github.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : For ChinookLib to be useable, it must be cloned on your hard
//           drive so the path
//               "..\..\..\ChinookLib\ChinookLib.X\headers\ChinookLib.h"
//           references an existing file.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#include "..\headers\Setup.h"
#include "..\headers\Interrupts.h"
#include "..\headers\StateMachine.h"
#include "..\headers\HardwareProfile.h"
#include "..\headers\DRV8711_Para.h"
#include "..\headers\SkadiFunctions.h"
#include "..\headers\CommandFunctions.h"
#include "..\headers\StateFunctions.h"

  UINT8 nSamples = 0;
  
  extern volatile BOOL oAdcReady;
  
  UINT32 adcData1[10], adcData2[10];
  
  UINT32 adcMeanValue1 = 0, adcMeanValue2 = 0;
  UINT8 i;
  
  float crabMm1, crabMm2, crabDeg1, crabDeg2;
  
  float adcReal1, adcReal2;

//==============================================================================
// MAIN CODE
//==============================================================================
void main(void)
{

//==============================================================================
// Following memcopies are used to be able to use the form 
// Chinook.LibraryX.FunctionX in case the developper wants to see the available
// functions.
//==============================================================================
  memcpy( &Chinook.Port  , &Port  , sizeof ( struct sChinookPort  ) );
  memcpy( &Chinook.Uart  , &Uart  , sizeof ( struct sChinookUart  ) );
  memcpy( &Chinook.Pwm   , &Pwm   , sizeof ( struct sChinookPwm   ) );
  memcpy( &Chinook.Timer , &Timer , sizeof ( struct sChinookTimer ) );
  memcpy( &Chinook.Spi   , &Port  , sizeof ( struct sChinookSpi   ) );
  memcpy( &Chinook.Wdt   , &Wdt   , sizeof ( struct sChinookWdt   ) );
  memcpy( &Chinook.Adc   , &Adc   , sizeof ( struct sChinookAdc   ) );
  memcpy( &Chinook.Can   , &Can   , sizeof ( struct sChinookCan   ) );
  memcpy( &Chinook.I2c   , &I2c   , sizeof ( struct sChinookI2c   ) );
  memcpy( &Chinook.Skadi , &Skadi , sizeof ( struct sChinookSkadi ) );
  memcpy( &Chinook.InputCapture , &InputCapture , sizeof ( struct sChinookInputCapture ) );
//==============================================================================


//==============================================================================
// The next line disables the JTAG for the PIC. If the JTAG is enabled, pins
// RB10-13 can't be used as I/Os. If you want to use the JTAG, comment or
// remove this line.
//==============================================================================
  DDPCONbits.JTAGEN = 0;
//==============================================================================

  
//==============================================================================
// Configure the device for maximum performance but do not change the PBDIV
// Given the options, this function will change the flash wait states, RAM
// wait state and enable prefetch cache but will not change the PBDIV.
// The PBDIV value is already set via the pragma FPBDIV option in HardwareProfile.h.
//==============================================================================
  SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
//==============================================================================


// State machine init
//============================
	pStateMast = &StateInit;
//============================
  
  StateInit();
  
  Adc.EnableInterrupts ();
  Can.EnableInterrupt(CAN1);
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
  INTEnableInterrupts();
  
  DRVA_SLEEP = 1;
  DRVA_RESET = 0;
  DRVB_SLEEP = 1;
  DRVB_RESET = 0;
  
  UINT16 pwm2 = 500;
  UINT16 pwm3 = 500;
  DRVA_SLEEP = 0;
  DRVB_SLEEP = 0;

  Pwm.SetDutyCycle(PWM_2, pwm2);
  Pwm.SetDutyCycle(PWM_3, pwm3);
  Pwm.SetDutyCycle(PWM_4, pwm2);
  Pwm.SetDutyCycle(PWM_5, pwm3);

  WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors at the drive
  WriteDrive(DRVA, STATUS_Mastw);   // Reset any errors at the drive
  LED_ERROR_ON;
  UINT8 buffer[100] = {0};
  UINT16 size = 0;
  
  
  

	while(1)  //infinite loop
	{
    if (oAdcReady)
    {
      oAdcReady = 0;
      adcData1[nSamples] = Adc.Var.adcReadValues[2];
      adcData2[nSamples] = Adc.Var.adcReadValues[3];
      
      nSamples++;
      
      if (nSamples >= 10)
      {
        nSamples = 0;
        
        adcMeanValue1 = 0;
        adcMeanValue2 = 0;
        
        for (i = 0; i < 10; i++)
        {
          adcMeanValue1 += adcData1[i];
          adcMeanValue2 += adcData2[i];
        }
        
        adcMeanValue1 = adcMeanValue1 / 10.0f + 0.5;
        adcMeanValue2 = adcMeanValue2 / 10.0f + 0.5;
        
//        adcReal1 = ACTUATOR_MAX_VOLT * (adcMeanValue1 >> 10);
//        adcReal2 = ACTUATOR_MAX_VOLT * (adcMeanValue2 >> 10);
        adcReal1 = VREF_PLUS * adcMeanValue1 / 1024.0;
        adcReal2 = VREF_PLUS * adcMeanValue2 / 1024.0;
        
        crabMm1 = adcReal1 / ACTUATOR_MAX_VOLT * (ACTUATOR_MAX_POS - ACTUATOR_MIN_POS);
        crabMm2 = adcReal2 / ACTUATOR_MAX_VOLT * (ACTUATOR_MAX_POS - ACTUATOR_MIN_POS);
        CrabMmToDeg(crabMm1, &crabDeg1);
        CrabMmToDeg(crabMm2, &crabDeg2);
        
        Can.SendFloat(CAN1, 0x21, crabMm1);
        Can.SendFloat(CAN1, 0x13, crabDeg1);
      }
    }
    
//    if (Uart.GetDataByte(UART6) == 'p')
//    {
//      Uart.SendDataByte(UART6, 'p');
//      Uart.SendDataByte(UART6, '\n');
//      Uart.SendDataByte(UART6, '\r');
//      if ( (pwm2 <= 700) && (pwm3 >= 300) )
//      {
//        pwm2 += 50;
//        pwm3 -= 50;
//        Pwm.SetDutyCycle(PWM_2, pwm2);
//        Pwm.SetDutyCycle(PWM_3, pwm3);
//        Pwm.SetDutyCycle(PWM_4, pwm2);
//        Pwm.SetDutyCycle(PWM_5, pwm3);
//        if (pwm2 != 500)
//        {
//          DRVB_SLEEP = 1;
//          DRVA_SLEEP = 1;
//        }
//        else
//        {
//          DRVB_SLEEP = 0;
//          DRVA_SLEEP = 0;
//        }
//      }
//      size = sprintf(buffer, "pwm2 = %d, pwm3 = %d\n\r", pwm2, pwm3);
//      Uart.SendDataBuffer(UART6, buffer, size);
//    }
//    
//    if (Uart.GetDataByte(UART6) == 'm')
//    {
//      Uart.SendDataByte(UART6, 'm');
//      Uart.SendDataByte(UART6, '\n');
//      Uart.SendDataByte(UART6, '\r');
//      if ( (pwm3 <= 700) && (pwm2 >= 300) )
//      {
//        pwm3 += 50;
//        pwm2 -= 50;
//        Pwm.SetDutyCycle(PWM_2, pwm2);
//        Pwm.SetDutyCycle(PWM_3, pwm3);
//        Pwm.SetDutyCycle(PWM_4, pwm2);
//        Pwm.SetDutyCycle(PWM_5, pwm3);
//        if (pwm2 != 500)
//        {
//          DRVB_SLEEP = 1;
//          DRVA_SLEEP = 1;
//        }
//        else
//        {
//          DRVB_SLEEP = 0;
//          DRVA_SLEEP = 0;
//        }
//      }
//      size = sprintf(buffer, "pwm2 = %d, pwm3 = %d\n\r", pwm2, pwm3);
//      Uart.SendDataBuffer(UART6, buffer, size);
//    }

//    Timer.DelayMs(200);
//    WriteDrive(DRVB, STATUS_Mastw);   // Reset any errors at the drive
    
    
    //======================================
    // Mast State machine with Drive A
    //======================================
//    (*pStateMast)();    // jump to next state
//    StateScheduler();   // Decides which state will be next

	}  // end while
} // end main