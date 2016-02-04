//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : Interrupts.c
// Author  : Frederic Chasse
// Date    : 2015-01-03
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Purpose : This file declares all interrupt subroutines used.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : All priorities must be set and verified by the developper. All
//           priorities and subpriorities used are there as example. Also, note
//           that interrupt can be defined as iplnauto, iplnsrs and iplnsoft,
//           where n can be from 0 to 7. 7 is the highest priority, while 1 is
//           the lowest. A priority of 0 will disable the interrupt.
//           Subpriorities can go from (lowest to highest) 0 to 3.
//           SRS mode uses the shadow registers, soft uses the software
//           registers and auto is an automatic assignment.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\Interrupts.h"
#include "..\headers\StateFunctions.h"
#include "..\headers\CommandFunctions.h"

volatile BOOL  oCapture1      = 0
              ,oCapture2      = 0
              ,oCapture3      = 0
              ,oCapture4      = 0
              ,oNewWindAngle  = 0
              ,oTimerReg      = 0
              ,oTimerSendData = 0
              ,oTimerChngMode = 0
              ,oEnableMastStopProcedure = 0
              ,oFirstCapture1 = 1
              ,oFirstCapture2 = 1
              ,oFirstCapture3 = 1
              ,oFirstCapture4 = 1
              ;

UINT8 iMastStop = 0;
INT8  mastDir = 0;

volatile UINT32 rxWindAngle = 0;  // Received from CAN

extern volatile BOOL  oManualMode
                     ,oCountTimeToChngMode
                     ;

extern volatile float mastCurrentSpeed;

extern volatile sButtonStates_t buttons;

//==============================================================================
//	TIMER INTERRUPTS
//==============================================================================

//=============================================
// Configure the Timer 1 interrupt handler
//=============================================
void __ISR(_TIMER_1_VECTOR, T1_INTERRUPT_PRIORITY) Timer1InterruptHandler(void)
{
  oTimerReg = 1;

  // Increment the number of overflows from this timer. Used primarily by Input Capture
  Timer.Var.nOverflows[0]++;

  mT1ClearIntFlag();
}

//=============================================
// Configure the Timer 2 interrupt handler
//=============================================
void __ISR(_TIMER_2_VECTOR, T2_INTERRUPT_PRIORITY) Timer2InterruptHandler(void)
{
  /*
   * Following code performs a smooth stop. It goes from 30% speed to 0 in order
   * to have a smooth curve. Used in manual mode and for emergency stops in
   * automatic mode (if the mast is too far).
   */
  if (oEnableMastStopProcedure)
  {
    if (iMastStop == 0)
    {
      mastDir = SIGN(mastCurrentSpeed);
    }

    if (iMastStop < 16)
    {
      DRVB_SLEEP = 1;

      if (mastDir == MAST_DIR_LEFT)
      {
        // DRIVE B
        //==========================================================
        if (USE_DRIVE_B == 1)
        {
          Pwm.SetDutyCycle(PWM_2, 500 + (150 - iMastStop*10));
          Pwm.SetDutyCycle(PWM_3, 500 - (150 - iMastStop*10));
        }
        //==========================================================
          
        // DRIVE A
        //==========================================================
        if (USE_DRIVE_A == 1)
        {
          Pwm.SetDutyCycle(PWM_4, 500 + (150 - iMastStop*10));
          Pwm.SetDutyCycle(PWM_5, 500 - (150 - iMastStop*10));
        //==========================================================
        }
      }
      else if (mastDir == MAST_DIR_RIGHT)
      {
        // DRIVE B
        //==========================================================
        if (USE_DRIVE_B == 1)
        {
          Pwm.SetDutyCycle(PWM_2, 500 - (150 - iMastStop*10));
          Pwm.SetDutyCycle(PWM_3, 500 + (150 - iMastStop*10));
        }
        //==========================================================

        // DRIVE A
        //==========================================================
        if (USE_DRIVE_A == 1)
        {
          Pwm.SetDutyCycle(PWM_4, 500 - (150 - iMastStop*10));
          Pwm.SetDutyCycle(PWM_5, 500 + (150 - iMastStop*10));
        }
        //==========================================================
      }
      iMastStop++;
    }
    else
    {
      iMastStop = 0;
      mastDir   = 0;

      // DRIVE B
      //==========================================================
      if (USE_DRIVE_B == 1)
      {
        Pwm.SetDutyCycle(PWM_2, 500);
        Pwm.SetDutyCycle(PWM_3, 500);

        DRVB_SLEEP = 0;
      }
      //==========================================================

      // DRIVE A
      //==========================================================
      if (USE_DRIVE_A == 1)
      {
        Pwm.SetDutyCycle(PWM_4, 500);
        Pwm.SetDutyCycle(PWM_5, 500);

        DRVA_SLEEP = 0;
      }
      //==========================================================

      oEnableMastStopProcedure = 0;

      mastCurrentSpeed = 0;
    }
  }

  // Increment the number of overflows from this timer. Used primarily by Input Capture
  Timer.Var.nOverflows[1]++;

  mT2ClearIntFlag();
}

//=============================================
// Configure the Timer 3 interrupt handler
//=============================================
void __ISR(_TIMER_3_VECTOR, T3_INTERRUPT_PRIORITY) Timer3InterruptHandler(void)
{

  // Increment the number of overflows from this timer. Used primarily by Input Capture
  Timer.Var.nOverflows[2]++;

  mT3ClearIntFlag();
}

//=============================================
// Configure the Timer 4 interrupt handler
//=============================================
void __ISR(_TIMER_4_VECTOR, T4_INTERRUPT_PRIORITY) Timer4InterruptHandler(void)
{
  oTimerSendData = 1;

  // Increment the number of overflows from this timer. Used primarily by Input Capture
  Timer.Var.nOverflows[3]++;

  mT4ClearIntFlag();
}

//=============================================
// Configure the Timer 5 interrupt handler
//=============================================
void __ISR(_TIMER_5_VECTOR, T5_INTERRUPT_PRIORITY) Timer5InterruptHandler(void)
{
  oTimerChngMode = 1;

  // Increment the number of overflows from this timer. Used primarily by Input Capture
  Timer.Var.nOverflows[4]++;

  mT5ClearIntFlag();
}


/*******************************************************************************
 ***********************                               *************************
 ********************           UART INTERRUPTS           **********************
 ***********************                               *************************
 *******************************************************************************/

//=============================================
// Configure the UART 6 interrupt handler
//=============================================

void __ISR(_UART_6_VECTOR, U6_INTERRUPT_PRIORITY) Uart6InterruptHandler(void)
{
  UINT8  i
        ,iMax   // Read/write max 8 bytes/interrupt
        ,data   // used in UartFifoWrite/Read functions
        ;

  if ( INTGetFlag ( INT_SOURCE_UART_ERROR(UART6)) )
  {
    LED_ERROR_ON;
    INTClearFlag(INT_SOURCE_UART_ERROR(UART6));
  }
  
	// TX interrupt handling
  //===========================================================
  if ( INTGetEnable ( INT_SOURCE_UART_TX(UART6) ) )               // If TX interrupts enabled
  {
    if ( INTGetFlag ( INT_SOURCE_UART_TX(UART6) ) )               // If TX interrupt occured
    {
      if ( UARTTransmitterIsReady(UART6) && !Uart.Var.uartTxFifo[UART6].bufEmpty )  // If TX buffer is ready to receive data and the user's TX buffer is not empty
      {
        if (Uart.Var.uartTxFifo[UART6].lineBuffer.length < 8)     // Write max 8 bytes/interrupt
        {
          iMax = Uart.Var.uartTxFifo[UART6].lineBuffer.length;
        }
        else
        {
          iMax = 8;
        }

        for (i = 0; i < iMax; i++)
        {
          UartFifoRead((void *) &Uart.Var.uartTxFifo[UART6], &data);  // Copy from user
          U6TXREG = data;                                         // Put data in PIC32's TX buffer
        }
      }

      if (Uart.Var.uartTxFifo[UART6].bufEmpty)                    // If User's TX buffer is empty
      {
        Uart.DisableTxInterrupts(UART6);                          // Disable TX interrupts
      }

      INTClearFlag(INT_SOURCE_UART_TX(UART6));                    // Clear the TX interrupt Flag
    }
  }
  //===========================================================
  

	// RX interrupt handling
  //===========================================================
  if ( INTGetEnable ( INT_SOURCE_UART_RX(UART6) ) )               // If RX interrupts enabled
  {
    if ( INTGetFlag ( INT_SOURCE_UART_RX(UART6) ) )               // If RX interrupt occured
    {
      i = 0;
      iMax = 8;                                                   // Read max 8 bytes/interrupt
      while (   UARTReceivedDataIsAvailable(UART6)                // While RX data available
            && !Uart.Var.uartRxFifo[UART6].bufFull                // and user's RX buffer not full
            && (i < iMax)                                         // and under 8 bytes read
            )
      { // while ^
        data = UARTGetDataByte(UART6);                            // Get data for PIC32's RX FIFO buffer and copy it to user (next line)
        if ( UartFifoWrite((void *) &Uart.Var.uartRxFifo[UART6], &data) < 0 ) // If copy to user did not work
        {
          break;                                                  // Exit while loop
        }
        i++;
      } // end while

      if (!Uart.Var.uartRxFifo[UART6].bufEmpty)                   // If there is data in the user's RX buffer
      {
        Uart.Var.oIsRxDataAvailable[UART6] = 1;                   // Set according flag
      }

      INTClearFlag (INT_SOURCE_UART_RX(UART6) );                  // Clear the RX interrupt Flag

    }
	}
  //===========================================================
}
//=============================================


/*******************************************************************************
 ***********************                               *************************
 ********************      INPUT CAPTURE INTERRUPTS       **********************
 ***********************                               *************************
 ******************************************************************************/

//================================================
// Configure the InputCapture 1 interrupt handler
//================================================
void __ISR(_INPUT_CAPTURE_1_VECTOR, IC1_INT_PRIORITY) InputCapture1InterruptHandler(void)
{
  /*
   * DEVELOPPER CODE HERE
   */

  // Get the timer used by this Input Capture
  TimerNum_t numTimer = InputCapture.Var.timerUsed[IC1];

  // Wait for capture data to be ready
  while(!(InputCapture.IsCaptureReady(IC1)));

  // Update values of timer overflows
  InputCapture.Var.previousTimerOverflows[IC1] = InputCapture.Var.currentTimerOverflows[IC1];
  InputCapture.Var.currentTimerOverflows[IC1]  = Timer.ReadOverflows(numTimer);

  // Store the timer value of the capture event
  InputCapture.Var.previousCaptureCountValue[IC1] = InputCapture.Var.currentCaptureCountValue[IC1];
  InputCapture.Var.currentCaptureCountValue [IC1] = InputCapture.ReadCapture(IC1);

  if (!oFirstCapture1)
  {
    oCapture1 = 1;   // Flag that tells that a new Capture event occured
  }
  else
  {
    oFirstCapture1 = 0;
  }

  // Clear the interrupt flag
  INTClearFlag(INT_IC1);
}
//================================================

//================================================
// Configure the InputCapture 2 interrupt handler
//================================================
void __ISR(_INPUT_CAPTURE_2_VECTOR, IC2_INT_PRIORITY) InputCapture2InterruptHandler(void)
{
  // Get the timer used by this Input Capture
  TimerNum_t numTimer = InputCapture.Var.timerUsed[IC2];

  // Wait for capture data to be ready
  while(!(InputCapture.IsCaptureReady(IC2)));

  // Update values of timer overflows
  InputCapture.Var.previousTimerOverflows[IC2] = InputCapture.Var.currentTimerOverflows[IC2];
  InputCapture.Var.currentTimerOverflows[IC2]  = Timer.ReadOverflows(numTimer);

  // Store the timer value of the capture event
  InputCapture.Var.previousCaptureCountValue[IC2] = InputCapture.Var.currentCaptureCountValue[IC2];
  InputCapture.Var.currentCaptureCountValue [IC2] = InputCapture.ReadCapture(IC2);

  if (!oFirstCapture2)
  {
    oCapture2 = 1;   // Flag that tells that a new Capture event occured
  }
  else
  {
    oFirstCapture2 = 0;
  }

  // Clear the interrupt flag
  INTClearFlag(INT_IC2);
}
//================================================

//================================================
// Configure the InputCapture 3 interrupt handler
//================================================
void __ISR(_INPUT_CAPTURE_3_VECTOR, IC3_INT_PRIORITY) InputCapture3InterruptHandler(void)
{
  /*
   * DEVELOPPER CODE HERE
   */

  // Get the timer used by this Input Capture
  TimerNum_t numTimer = InputCapture.Var.timerUsed[IC3];

  // Wait for capture data to be ready
  while(!(InputCapture.IsCaptureReady(IC3)));

  // Update values of timer overflows
  InputCapture.Var.previousTimerOverflows[IC3] = InputCapture.Var.currentTimerOverflows[IC3];
  InputCapture.Var.currentTimerOverflows[IC3]  = Timer.ReadOverflows(numTimer);

  // Store the timer value of the capture event
  InputCapture.Var.previousCaptureCountValue[IC3] = InputCapture.Var.currentCaptureCountValue[IC3];
  InputCapture.Var.currentCaptureCountValue [IC3] = InputCapture.ReadCapture(IC3);

  if (!oFirstCapture3)
  {
    oCapture3 = 1;   // Flag that tells that a new Capture event occured
  }
  else
  {
    oFirstCapture3 = 0;
  }

  // Clear the interrupt flag
  INTClearFlag(INT_IC3);
}
//================================================

//================================================
// Configure the InputCapture 4 interrupt handler
//================================================
void __ISR(_INPUT_CAPTURE_4_VECTOR, IC4_INT_PRIORITY) InputCapture4InterruptHandler(void)
{
  // Get the timer used by this Input Capture
  TimerNum_t numTimer = InputCapture.Var.timerUsed[IC4];

  // Wait for capture data to be ready
  while(!(InputCapture.IsCaptureReady(IC4)));

  // Update values of timer overflows
  InputCapture.Var.previousTimerOverflows[IC4] = InputCapture.Var.currentTimerOverflows[IC4];
  InputCapture.Var.currentTimerOverflows[IC4]  = Timer.ReadOverflows(numTimer);

  // Store the timer value of the capture event
  InputCapture.Var.previousCaptureCountValue[IC4] = InputCapture.Var.currentCaptureCountValue[IC4];
  InputCapture.Var.currentCaptureCountValue [IC4] = InputCapture.ReadCapture(IC4);

  if (!oFirstCapture4)
  {
    oCapture4 = 1;   // Flag that tells that a new Capture event occured
  }
  else
  {
    oFirstCapture4 = 0;
  }

  // Clear the interrupt flag
  INTClearFlag(INT_IC4);
}
//================================================


/*******************************************************************************
 ***********************                               *************************
 ********************           I2C INTERRUPTS            **********************
 ***********************                               *************************
 *******************************************************************************/

//=============================================
// Configure the I2C4 interrupt handler
//=============================================
void __ISR(_I2C_4_VECTOR, I2C4_INT_PRIORITY) I2c4InterruptHandler(void)
{
  sI2cCmdBuffer_t masterData;
  
  if (INTGetFlag(INT_I2C4B))  //Bus Collision interrupt
  {
    INTClearFlag(INT_I2C4B);
  }
  
  if (INTGetFlag(INT_I2C4M))  // Master interrupt
  {
    INTClearFlag(INT_I2C4M);

    if (I2c.Var.oReadDataInNextInterrupt[I2C4])   // If a read was started last interrupt
    {
      masterData.data  = I2C4RCV;   // Read from I2C buffer
      masterData.state = I2C_MASTER_RECEIVE_DATA;
      I2cFifoWrite((void *) &I2c.Var.i2cUserFifo[I2C4], &masterData);
      I2c.Var.oRxDataAvailable[I2C4] = 1;
      I2c.Var.oReadDataInNextInterrupt[I2C4] = 0;
    }
    
    if (I2c.Var.oI2cWriteIsRunning[I2C4])
    {
      I2cFifoRead((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
      switch (masterData.state)
      {
      //======================================================  
        case I2C_MASTER_RECEIVE_DATA : 
          I2C4CONbits.RCEN = 1;                         //Receive byte sequence
          I2c.Var.oReadDataInNextInterrupt[I2C4] = 1;   // Flag for the next interrupt to read the rx buffer
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_START_CONDITION : 
          I2C4CONbits.SEN = 1;                          //start condition sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_STOP_CONDITION : 
          I2C4CONbits.PEN = 1;
          
          if (I2c.Var.oPoolSlaveAcknowledge[I2C4])
          {
            if (!I2c.Var.oSecondStopAfterPooling[I2C4])
            {
              masterData.state = I2C_MASTER_START_CONDITION;
              I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
              
              masterData.state = I2C_MASTER_TRANSMIT_DATA;
              I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
              
              masterData.state = I2C_MASTER_STOP_CONDITION;
              I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
              
              I2c.Var.oSecondStopAfterPooling[I2C4] = 1;
            }
            else
            {
              if (!I2CByteWasAcknowledged(I2C4))
              {
                masterData.state = I2C_MASTER_START_CONDITION;
                I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);

                masterData.state = I2C_MASTER_TRANSMIT_DATA;
                I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);

                masterData.state = I2C_MASTER_STOP_CONDITION;
                I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
              }
              else
              {
                masterData.state = I2C_MASTER_DONE;
                I2cFifoWrite((void *) &I2c.Var.i2cWriteQueue[I2C4], &masterData);
                I2c.Var.oSecondStopAfterPooling[I2C4] = 0;
              }
            }
          }
          break;
      //====================================================== 

      //======================================================    
        case I2C_MASTER_TRANSMIT_DATA : 
          I2C4TRN = masterData.data;
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_DONE : 
          if (I2c.Var.i2cWriteQueue[I2C4].bufEmpty)   // Nothing more to send
          {
            I2c.Var.oI2cWriteIsRunning[I2C4] = 0;       // Turn off writing process
          }
          else
          {
            INTSetFlag(INT_I2C4M);                    // Start another writing process
          }
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_REPEAT_START : 
          I2C4CONbits.RSEN = 1;   //repeated start condition sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SLAVE_SENT_STOP :
          LED_ERROR_ON;
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SEND_ACK : 
          I2C4CONbits.ACKDT = 0;  //ACK
          I2C4CONbits.ACKEN = 1;  //Send ACK sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SEND_NACK : 
          I2C4CONbits.ACKDT = 1;  //NACK
          I2C4CONbits.ACKEN = 1;  //Send NACK sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_CMD_ERROR : 
          LED_ERROR_ON;
          break;
      //====================================================== 

      //======================================================  
        default : 
          break;
      //======================================================  
      } // end switch
    } // end if
    
    

    if (I2c.Var.oI2cReadIsRunning[I2C4])
    {
      I2cFifoRead((void *) &I2c.Var.i2cReadQueue[I2C4], &masterData);
      switch (masterData.state)
      {
      //======================================================  
        case I2C_MASTER_RECEIVE_DATA : 
          I2C4CONbits.RCEN = 1;   //Receive byte sequence
          I2c.Var.oReadDataInNextInterrupt[I2C4] = 1;
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_START_CONDITION : 
          I2C4CONbits.SEN = 1;    //start condition sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_REPEAT_START : 
          I2C4CONbits.RSEN = 1;   //repeated start condition sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_STOP_CONDITION : 
          I2C4CONbits.PEN = 1;
          break;
      //====================================================== 

      //======================================================    
        case I2C_MASTER_TRANSMIT_DATA : 
          I2C4TRN = masterData.data;
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_DONE : 
          if (I2c.Var.i2cReadQueue[I2C4].bufEmpty)            // Nothing more to send
          {
            I2c.Var.oI2cReadIsRunning[I2C4] = 0;              // Turn off reading process
          }
          else
          {
            INTSetFlag(INT_I2C4M);                    // Start another reading process
          }
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SLAVE_SENT_STOP :
          LED_ERROR_ON;
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SEND_ACK : 
          I2C4CONbits.ACKDT = 0;  //ACK
          I2C4CONbits.ACKEN = 1;  //Send ACK sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_MASTER_SEND_NACK : 
          I2C4CONbits.ACKDT = 1;  //NACK
          I2C4CONbits.ACKEN = 1;  //Send NACK sequence
          break;
      //====================================================== 

      //======================================================  
        case I2C_CMD_ERROR : 
          LED_ERROR_ON;
          break;
      //======================================================

      //======================================================  
        default : 
          break;
      //======================================================  
      } // end switch
    } // end if
  }  // end if
}   // end if
//=============================================


/*******************************************************************************
 ***********************                               *************************
 ********************           CAN INTERRUPTS            **********************
 ***********************                               *************************
 ******************************************************************************/

//================================================
// Configure the CAN1 interrupt handler
//================================================
void __ISR(_CAN_1_VECTOR, CAN1_INT_PRIORITY) Can1InterruptHandler(void)
{
  // Check if the source of the interrupt is RX_EVENT. This is redundant since
  // only this event is enabled in this example but this shows one scheme for
  // handling events

  if ((CANGetModuleEvent(CAN1) & CAN_RX_EVENT) != 0)
  {

    CANRxMessageBuffer *message;

    /*
     * CHANNEL 1 = SWITCHES STATES
     */
    if (CANGetPendingEventCode(CAN1) == CAN_CHANNEL1_EVENT)
    {

      CANEnableChannelEvent(CAN1, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, FALSE);

      message = CANGetRxMessage(CAN1, CAN_CHANNEL1);

      CanSwitches_t switches;
      switches.bytes.low  = message->data[0];
      switches.bytes.high = message->data[1];

      if (buttons.buttons.bits.steerWheelSw1  != switches.bits.sw1 )
      {
        buttons.buttons.bits.steerWheelSw1  = switches.bits.sw1;
        buttons.chng.bits.steerWheelSw1     = 1;
      }

      if (buttons.buttons.bits.steerWheelSw3  != switches.bits.sw3 )
      {
        buttons.buttons.bits.steerWheelSw3  = switches.bits.sw3;
        buttons.chng.bits.steerWheelSw3     = 1;
      }

      if (buttons.buttons.bits.steerWheelSw10 != switches.bits.sw10)
      {
        buttons.buttons.bits.steerWheelSw10 = switches.bits.sw10;
        buttons.chng.bits.steerWheelSw10    = 1;
      }

      CANUpdateChannel(CAN1, CAN_CHANNEL1);
      CANEnableChannelEvent(CAN1, CAN_CHANNEL1, CAN_RX_CHANNEL_NOT_EMPTY, TRUE);

    }

    /*
     * CHANNEL 2 = WIND ANGLE
     */
    if (CANGetPendingEventCode(CAN1) == CAN_CHANNEL2_EVENT)
    {

      CANEnableChannelEvent(CAN1, CAN_CHANNEL2, CAN_RX_CHANNEL_NOT_EMPTY, FALSE);

      message = CANGetRxMessage(CAN1, CAN_CHANNEL2);

      memcpy((void *) &rxWindAngle, &message->data[0], 4);
      
      oNewWindAngle = 1;

      CANUpdateChannel(CAN1, CAN_CHANNEL2);
      CANEnableChannelEvent(CAN1, CAN_CHANNEL2, CAN_RX_CHANNEL_NOT_EMPTY, TRUE);
    }
  }

  // The CAN1 Interrupt flag is  cleared at the end of the interrupt routine.
  // This is because the event source that could have caused this interrupt to
  // occur (CAN_RX_CHANNEL_NOT_EMPTY) is disabled. Attempting to clear the
  // CAN1 interrupt flag when the the CAN_RX_CHANNEL_NOT_EMPTY interrupt is
  // enabled will not have any effect because the base event is still present.
  INTClearFlag(INT_CAN1);
}