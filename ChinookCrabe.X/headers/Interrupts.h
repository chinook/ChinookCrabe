//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : Interrupts.h
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


#ifndef __INTERRUPT_H__
#define	__INTERRUPT_H__

#include "Setup.h"


//==============================================================================
// VARIABLES
//==============================================================================

// Typedef for mapping the steering wheel switches
typedef union
{
  struct
  {
    UINT16 sw1  : 1
          ,sw2  : 1
          ,sw3  : 1
          ,sw4  : 1
          ,sw5  : 1
          ,sw6  : 1
          ,sw7  : 1
          ,sw8  : 1
          ,sw9  : 1
          ,sw10 : 1
          ,sw11 : 1
          ,sw12 : 1
          ,     : 4
          ;
  } bits;

  struct
  {
    UINT8 low;
    UINT8 high;
  } bytes;

  UINT16 word;

} CanSwitches_t;

//==============================================================================
// DEFINITIONS
//==============================================================================

/*
 *  ==========================================
 *            Table of interrupts
 *  ==========================================
 * 
 * +--------------------------------------------+
 * | Priority | Subpriority | Interrupt handler |
 * |----------+-------------+-------------------|
 * | 1        | 0           | UART 6            |
 * | 1        | 1           |                   |
 * | 1        | 2           |                   |
 * | 1        | 3           |                   |
 * |----------+-------------+-------------------|
 * | 2        | 0           |                   |
 * | 2        | 1           |                   |
 * | 2        | 2           | Timer 4           |
 * | 2        | 3           |                   |
 * |----------+-------------+-------------------|
 * | 3        | 0           |                   |
 * | 3        | 1           | Timer 3           |
 * | 3        | 2           |                   |
 * | 3        | 3           |                   |
 * |----------+-------------+-------------------|
 * | 4        | 0           | Timer 2           |
 * | 4        | 1           | Timer 5           |
 * | 4        | 2           |                   |
 * | 4        | 3           | SPI 4             |
 * |----------+-------------+-------------------|
 * | 5        | 0           | ADC               |
 * | 5        | 1           |                   |
 * | 5        | 2           | I2C 4             |
 * | 5        | 3           |                   |
 * |----------+-------------+-------------------|
 * | 6        | 0           |                   |
 * | 6        | 1           | CAN 1             |
 * | 6        | 2           |                   |
 * | 6        | 3           | Timer 1           |
 * |----------+-------------+-------------------|
 * | 7        | 0           |                   |
 * | 7        | 1           |                   |
 * | 7        | 2           |                   |
 * | 7        | 3           |                   |
 * +----------+-------------+-------------------+
 * 
 */

//===============================================
// Timer interrupts priorities and subpriorities
//===============================================
#define TIMER1_INTERRUPT_PRIORITY       6         // Used in ChinookLib function
#define T1_INTERRUPT_PRIORITY           ipl6auto  // Used in ISR
#define TIMER2_INTERRUPT_PRIORITY       4         // Used in ChinookLib function
#define T2_INTERRUPT_PRIORITY           ipl4auto  // Used in ISR
#define TIMER3_INTERRUPT_PRIORITY       3         // Used in ChinookLib function
#define T3_INTERRUPT_PRIORITY           ipl3auto  // Used in ISR
#define TIMER4_INTERRUPT_PRIORITY       2         // Used in ChinookLib function
#define T4_INTERRUPT_PRIORITY           ipl2auto  // Used in ISR
#define TIMER5_INTERRUPT_PRIORITY       4         // Used in ChinookLib function
#define T5_INTERRUPT_PRIORITY           ipl4auto  // Used in ISR

#define TIMER1_INTERRUPT_SUBPRIORITY    3         // Highest subpriority
#define TIMER2_INTERRUPT_SUBPRIORITY    0
#define TIMER3_INTERRUPT_SUBPRIORITY    1         // Highest subpriority
#define TIMER4_INTERRUPT_SUBPRIORITY    2         // Highest subpriority
#define TIMER5_INTERRUPT_SUBPRIORITY    1         // Highest subpriority
//===============================================


//===============================================
// UART interrupts priorities and subpriorities
//===============================================
#define UART6_INTERRUPT_PRIORITY        1           // Used in ChinookLib function
#define U6_INTERRUPT_PRIORITY           ipl1auto    // Used in ISR

#define UART6_INTERRUPT_SUBPRIORITY     0           // Lowest subpriority
//===============================================


//===============================================
// SPI interrupts priorities and subpriorities
//===============================================
#define SPI4_INTERRUPT_PRIORITY         4           // Used in ChinookLib function
#define S4_INTERRUPT_PRIORITY           ipl4auto    // Used in ISR

#define SPI4_INTERRUPT_SUBPRIORITY      3           // Lowest subpriority
//===============================================


//===============================================
// ADC interrupts priorities and subpriorities
//===============================================
#define ADC_INTERRUPT_PRIORITY          5           // Used in ChinookLib function
#define ADC_INT_PRIORITY                ipl5auto    // Used in ISR

#define ADC_INTERRUPT_SUBPRIORITY       0           
//===============================================


//===============================================
// CAN interrupts priorities and subpriorities
//===============================================
#define CAN1_INTERRUPT_PRIORITY         6           // Used in ChinookLib function
#define CAN1_INT_PRIORITY               ipl6auto    // Used in ISR

#define CAN1_INTERRUPT_SUBPRIORITY      1           
//===============================================


//===============================================
// I2C interrupts priorities and subpriorities
//===============================================
#define I2C4_INTERRUPT_PRIORITY         5           // Used in ChinookLib function
#define I2C4_INT_PRIORITY               ipl5auto    // Used in ISR

#define I2C4_INTERRUPT_SUBPRIORITY      2
//===============================================


#endif	/* __INTERRUPT_H__ */

