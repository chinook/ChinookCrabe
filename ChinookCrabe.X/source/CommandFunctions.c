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

// Actuator relative mm to deg table
//=====================================
const sMmToDeg_t mm2degRel = 
{
   .deg       = {   -35,    -34,    -33,    -32,    -31,    -30,    -29,    -28,   -27,    -26,    -25,    -24,    -23,    -22,    -21,    -20,    -19,    -18,    -17,  -16,     -15,    -14,    -13,    -12,    -11,    -10,     -9,     -8,     -7,     -6,     -5,     -4,     -3,     -2,     -1,      0,      1,      2,      3,      4,      5,      6,      7,      8,      9,     10,     11,     12,     13,     14,     15,    16,     17,     18,     19,     20,     21,     22,     23,     24,     25,     26,    27,     28,     29,     30,     31,     32,     33,     34,     35}
  ,.leftMm    = { -77.5, -75.06, -72.63, -70.21, -67.79, -65.38, -62.98, -60.59, -58.21, -55.84, -53.48, -51.14, -48.81, -46.49, -44.19, -41.9, -39.63, -37.38, -35.14, -32.92, -30.72, -28.53, -26.36, -24.22, -22.09, -19.98, -17.89, -15.82, -13.77, -11.74, -9.73, -7.74, -5.77, -3.83, -1.9, 0, 1.89, 3.75, 5.59, 7.41, 9.2, 10.98, 12.74, 14.47, 16.18, 17.87, 19.54, 21.19, 22.82, 24.43, 26.02, 27.81, 29.13, 30.66, 32.16, 33.65, 35.12, 36.57, 37.99, 39.4, 40.79, 42.16, 43.51, 44.85, 46.16, 47.46, 48.74, 50, 51.25, 52.47, 53.68}
  ,.rightMm   = { 53.68,  52.47,  51.25,     50, 48.74, 47.46, 46.16, 44.85, 43.51, 42.16, 40.79, 39.4, 37.99, 36.57, 35.12, 33.65, 32.16, 30.66, 29.13, 27.81, 26.02, 24.43, 22.82, 21.19, 19.54, 17.87, 16.18, 14.47, 12.74, 10.98, 9.2, 7.41, 5.59, 3.75, 1.89, 0, -1.9, -3.83, -5.77, -7.74, -9.73, -11.74, -13.77, -15.82, -17.89, -19.98, -22.09, -24.22, -26.36, -28.53, -30.72, -32.92, -35.14, -37.38, -39.63, -41.9, -44.19, -46.49, -48.81, -51.14, -53.48, -55.84, -58.21, -60.59, -62.98, -65.38, -67.79, -70.21, -72.63, -75.06, -77.5}

};
//=====================================

//float  crabLeftZeroMm  = 367.14
//      ,crabRightZeroMm = 367.34
//      ;
float  crabLeftZeroMm  = 370.12
      ,crabRightZeroMm = 372.38
      ;

// Actuator V to mm table
//=====================================
const sVoltToMm_t volt2mm = 
{
   .leftVolt    = {0.54985, 0.59957, 0.64929, 0.69609 , 0.74873 , 0.81015 , 0.86572, 0.91837, 0.96809, 1.0149, 1.0675, 1.1085, 1.1553, 1.2021, 1.2313, 1.2723, 1.3044, 1.3658, 1.4039, 1.439, 1.477, 1.5121, 1.553, 1.5852, 1.6203, 1.6525, 1.6817, 1.7139, 1.7373, 1.7578, 1.7782, 1.8016, 1.8163, 1.8367, 1.8514, 1.8748, 1.8894, 1.904, 1.9186, 1.9303, 1.942, 1.9654, 1.98, 1.9947, 2.0034, 2.0151, 2.0268, 2.0356, 2.0473, 2.059, 2.0707, 2.0795, 2.0912, 2.1, 2.1087, 2.1175, 2.1263, 2.1351, 2.1438, 2.1555, 2.1643, 2.1731, 2.1818, 2.1906, 2.1965, 2.2023, 2.2082, 2.2169, 2.2257, 2.2316, 2.2403, 2.2491, 2.2579, 2.2637, 2.2725, 2.2784, 2.2842, 2.2901, 2.2959, 2.3018, 2.3105, 2.3164, 2.3222, 2.3281, 2.3339, 2.3398, 2.3456, 2.3515, 2.3573, 2.3632, 2.369, 2.3749, 2.3807, 2.3866, 2.3924, 2.3983, 2.4071, 2.4129, 2.4158, 2.4188, 2.4246, 2.4275, 2.4305, 2.4334, 2.4363, 2.4392, 2.4422, 2.4451, 2.4509, 2.4539, 2.4568, 2.4597, 2.4626, 2.4655, 2.4685, 2.4714, 2.4743, 2.4772, 2.4802, 2.4831, 2.486, 2.4889, 2.4919, 2.4948, 2.4977, 2.5036, 2.5065, 2.5094, 2.5123, 2.5153, 2.5182, 2.5211, 2.524, 2.527, 2.5299, 2.5328, 2.5357, 2.5387, 2.5416, 2.5445, 2.5474, 2.5504, 2.5533, 2.5562, 2.5591, 2.5621, 2.565, 2.5679, 2.5708, 2.5738, 2.5767, 2.5796, 2.5825, 2.5855, 2.5884, 2.5913, 2.5942, 2.5972, 2.6001, 2.603, 2.6059, 2.6089, 2.6118, 2.6147, 2.6176, 2.6206, 2.6235, 2.6264, 2.6293, 2.6323, 2.6352, 2.6381, 2.641, 2.644}
  ,.leftMm      = {298.56 , 299.22 , 300.16 , 300.62  , 301.72  , 302.76  , 303.94, 304.92, 305.92, 306.84, 308.12, 309, 310.28, 311.38, 311.94, 313.14, 314.18, 316.82, 317, 318.26, 319.52, 320.62, 322.42, 323.84, 324.82, 326.18, 327.34, 328.82, 329.82, 331.14, 332.32, 333.18, 333.84, 335.04, 336.08, 337.26, 337.84, 338.42, 339.3, 340.32, 341.38, 342.72, 343.26, 344.14, 344.82, 345.62, 346.28, 346.98, 348, 349.02, 349.52, 350.62, 351.38, 352, 352.94, 353.62, 354.08, 355.22, 355.62, 357.1, 357.88, 358.48, 359.12, 359.96, 360.58, 361.32, 361.88, 363.72, 363.56, 364.14, 365.12, 366.34, 367.14, 367.92, 369, 369.36, 369.86, 370.38, 371.1, 371.78, 372.82, 373.74, 374.14, 375.16, 375.9, 376.46, 377.48, 378.36, 379.02, 379.82, 380.68, 381.84, 382.36, 383.04, 384.12, 384.92, 385.96, 386.86, 387.44, 387.82, 388.52, 389.1, 389.6, 390.36, 390.58, 391.08, 391.58, 392.62, 393.28, 393.64, 394.32, 394.64, 395, 395.64, 396.26, 396.8, 396.88, 397.82, 398.26, 398.8, 399.38, 399.68, 400.4, 400.72, 401.8, 402.36, 403.04, 403.72, 404.34, 405.5, 405.8, 406, 406.3, 407.56, 407.9, 408.46, 408.94, 409.48, 409.82, 410.5, 410.84, 411.8, 412, 413.3, 413.74, 414.44, 415.02, 415.82, 416.1, 417.1, 417.56, 418.62, 418.94, 419.92, 420.7, 421.28, 421.66, 423.14, 423.42, 424.08, 424.8, 426.26, 426.68, 427.52, 428.7, 429.72, 430.6, 431.1, 431.9, 432.7, 433.02, 434.2, 435, 436}
  ,.leftBit     = {188    , 205    , 222    , 238     , 256     , 277     , 296, 314, 331, 347, 365, 379, 395, 411, 421, 435, 446, 467, 480, 492, 505, 517, 531, 542, 554, 565, 575, 586, 594, 601, 608, 616, 621, 628, 633, 641, 646, 651, 656, 660, 664, 672, 677, 682, 685, 689, 693, 696, 700, 704, 708, 711, 715, 718, 721, 724, 727, 730, 733, 737, 740, 743, 746, 749, 751, 753, 755, 758, 761, 763, 766, 769, 772, 774, 777, 779, 781, 783, 785, 787, 790, 792, 794, 796, 798, 800, 802, 804, 806, 808, 810, 812, 814, 816, 818, 820, 823, 825, 826, 827, 829, 830, 831, 832, 833, 834, 835, 836, 838, 839, 840, 841, 842, 843, 844, 845, 846, 847, 848, 849, 850, 851, 852, 853, 854, 856, 857, 858, 859, 860, 861, 862, 863, 864, 865, 866, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879, 880, 881, 882, 883, 884, 885, 886, 887, 888, 889, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899, 900, 901, 902, 903, 904}
  ,.rightVolt   = {0.50013, 0.54985, 0.59957, 0.64929 , 0.69901 , 0.74873 , 0.79845, 0.8511, 0.89789, 0.93884, 0.97394, 1.0149, 1.0588, 1.0909, 1.1202, 1.1553, 1.1845, 1.2489, 1.2781, 1.3074, 1.3395, 1.3717, 1.398, 1.4185, 1.4536, 1.4799, 1.5004, 1.5296, 1.556, 1.5794, 1.5998, 1.6145, 1.6466, 1.6729, 1.6993, 1.7197, 1.7431, 1.7578, 1.7782, 1.7987, 1.8163, 1.8338, 1.8572, 1.8835, 1.904, 1.9215, 1.9449, 1.9625, 1.9859, 2.0005, 2.0181, 2.0385, 2.0561, 2.0736, 2.0912, 2.1087, 2.1175, 2.1263, 2.138, 2.1468, 2.1555, 2.1643, 2.1731, 2.1818, 2.1906, 2.1994, 2.2052, 2.2111, 2.2199, 2.2257, 2.2345, 2.2433, 2.2462, 2.255, 2.2637, 2.2696, 2.2784, 2.2842, 2.2901, 2.2959, 2.3018, 2.3076, 2.3135, 2.3164, 2.3222, 2.3252, 2.3281, 2.331, 2.3339, 2.3369, 2.3398, 2.3427, 2.3486, 2.3544, 2.3573, 2.3603, 2.3632, 2.3661, 2.369, 2.372, 2.3749, 2.3778, 2.3837, 2.3895, 2.3924, 2.3954, 2.3983, 2.4041, 2.41, 2.4129, 2.4158, 2.4188, 2.4217, 2.4246, 2.4275, 2.4305, 2.4334, 2.4363, 2.4392, 2.4422, 2.4451, 2.448, 2.4509, 2.4539, 2.4568, 2.4597, 2.4626, 2.4655, 2.4685, 2.4714, 2.4772, 2.4802, 2.4831, 2.486, 2.4889, 2.4919, 2.4948, 2.4977, 2.5006, 2.5036, 2.5065, 2.5094, 2.5123, 2.5153, 2.5182, 2.5211, 2.524, 2.527, 2.5299, 2.5328, 2.5357, 2.5387, 2.5416, 2.5445, 2.5474, 2.5504, 2.5533, 2.5562, 2.5591, 2.5621, 2.565, 2.5679, 2.5708, 2.5738, 2.5767, 2.5796, 2.5825, 2.5855, 2.5884, 2.5913, 2.5942, 2.5972, 2.603, 2.6059, 2.6089, 2.6118, 2.6147, 2.6176, 2.6206, 2.6235, 2.6264, 2.6293, 2.6323, 2.6352, 2.6381, 2.641, 2.644, 2.6469, 2.6498, 2.6527, 2.6557, 2.6586, 2.6615, 2.6644, 2.6674, 2.6732}
  ,.rightMm     = {297.86 , 298.68 , 299.36 , 300.2   , 300.86  , 301.74  , 302.42, 303.62, 304.56, 305.28, 306, 307, 307.98, 308.72, 309.38, 310.1, 310.96, 312.62, 313.62, 314.48, 315.24, 316.22, 317, 317.62, 319, 319.72, 320.44, 321.6, 322.54, 323.4, 324.14, 324.8, 326.12, 327.6, 328.36, 329.46, 330.6, 331.28, 332.36, 333.18, 334.2, 335.3, 336.42, 337.96, 339.22, 340.14, 341.56, 342.78, 344.28, 345.32, 346.56, 348.06, 349.42, 350.74, 352.06, 353.34, 354.22, 354.94, 355.7, 356.74, 357.4, 358.26, 359.12, 360.06, 360.56, 361.64, 362.22, 362.7, 363.52, 364.3, 365.1, 366, 366.52, 367.34, 368.46, 369.12, 369.92, 370.56, 371.46, 372.38, 372.82, 373.54, 374.14, 374.44, 375.1, 375.36, 376.28, 376.34, 376.88, 377.26, 377.66, 378.12, 378.78, 379.32, 379.98, 380.64, 380.82, 381.18, 381.38, 381.66, 382.38, 382.72, 383.38, 383.88, 384.64, 384.74, 385.48, 386, 387.16, 387.48, 388, 388.38, 388.84, 389.42, 389.86, 390.42, 390.98, 391.58, 391.76, 392.26, 392.94, 393.46, 393.66, 394.06, 394.54, 395, 395.74, 396.2, 396.78, 397.26, 398.24, 398.86, 399.08, 400, 400.48, 400.92, 401.18, 401.96, 402.36, 403.14, 403.72, 404.32, 404.72, 405.52, 405.9, 406.28, 406.72, 407.52, 408.18, 409.08, 409.52, 410.2, 410.88, 411.08, 412.12, 412.92, 413.24, 414.22, 414.74, 415.52, 416.2, 416.68, 417.42, 418.18, 418.82, 419.36, 420.14, 420.64, 421.08, 421.9, 423.28, 423.86, 424.66, 425.52, 426.64, 427.38, 428, 428.8, 429.98, 430.92, 431.38, 432.08, 433.58, 434, 434.52, 435.54, 436.72, 437, 438.1, 438.62, 439.92, 440.36, 441.68, 443.52, 444.04, 444.8}
  ,.rightBit    = {171    , 188    , 205    , 222     , 239     , 256     , 273   , 291, 307, 321, 333, 347, 362, 373, 383, 395, 405, 427, 437, 447, 458, 469, 478, 485, 497, 506, 513, 523, 532, 540, 547, 552, 563, 572, 581, 588, 596, 601, 608, 615, 621, 627, 635, 644, 651, 657, 665, 671, 679, 684, 690, 697, 703, 709, 715, 721, 724, 727, 731, 734, 737, 740, 743, 746, 749, 752, 754, 756, 759, 761, 764, 767, 768, 771, 774, 776, 779, 781, 783, 785, 787, 789, 791, 792, 794, 795, 796, 797, 798, 799, 800, 801, 803, 805, 806, 807, 808, 809, 810, 811, 812, 813, 815, 817, 818, 819, 820, 822, 824, 825, 826, 827, 828, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839, 840, 841, 842, 843, 844, 845, 847, 848, 849, 850, 851, 852, 853, 854, 855, 856, 857, 858, 859, 860, 861, 862, 863, 864, 865, 866, 867, 868, 869, 870, 871, 872, 873, 874, 875, 876, 877, 878, 879, 880, 881, 882, 883, 884, 885, 886, 887, 888, 890, 891, 892, 893, 894, 895, 896, 897, 898, 899, 900, 901, 902, 903, 904, 905, 906, 907, 908, 909, 910, 911, 912, 914}
  ,.leftOffset  = 0
  ,.rightOffset = 0
};


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

void CrabMmToDeg (float mm, float *deg, CrabActuator_t act)
{
  UINT8 i, iMm, max;
  
  if (act == LEFT_ACTUATOR)
  {
    max = 174;
    for (i = 0; i < max; i++)
    {
      if ((mm2degRel.leftMm[i] + crabLeftZeroMm) >= mm)
      {
        iMm = i;
        i = max+1;
      }
    }
    if (iMm < (max - 1))
    {
      Interpol2D(mm2degRel.leftMm[iMm] + crabLeftZeroMm, mm2degRel.deg[iMm], mm2degRel.leftMm[iMm + 1] + crabLeftZeroMm, mm2degRel.deg[iMm + 1], mm, deg);
    }
    else
    {
      *deg = mm2degRel.deg[iMm]; 
    }
  }
  else
  {
    max = 196;
    for (i = 0; i < max; i++)
    {
      if ((mm2degRel.rightMm[i] + crabRightZeroMm) <= mm)
      {
        iMm = i;
        i = max+1;
      }
    }
    if (iMm < (max - 1))
    {
      Interpol2D(mm2degRel.rightMm[iMm] + crabRightZeroMm, mm2degRel.deg[iMm], mm2degRel.rightMm[iMm + 1] + crabRightZeroMm, mm2degRel.deg[iMm + 1], mm, deg);
    }
    else
    {
      *deg = mm2degRel.deg[iMm]; 
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
  
//  *deg = mm2degRel.deg[iMm];
}

INT8 CrabDegToMm (INT8 deg, float *mm, CrabActuator_t act)
{
  UINT8 iMm;
  
  if ( (deg < -35) || (deg > 35) )
  {
    return -1;
  }
  
  iMm = deg + 35;
  
  if (act == LEFT_ACTUATOR)
  {
    *mm = mm2degRel.leftMm[iMm] + crabLeftZeroMm;
  }
  else
  {
    *mm = mm2degRel.rightMm[iMm] + crabRightZeroMm;
  }
  
  return 0;
}

void CrabBitToMm (UINT16 bitNum, float *mm, CrabActuator_t act)
{
  UINT8 i, iBit;
  UINT8 max;
  
  if (act == LEFT_ACTUATOR)
  {
    max = 174;
    for (i = 0; i < max; i++)
    {
      if (volt2mm.leftBit[i] >= bitNum)
      {
        iBit = i;
        i = max + 1;
      }
    }
    if (iBit < (max - 1))
    {
      Interpol2D(volt2mm.leftBit[iBit], volt2mm.leftMm[iBit], volt2mm.leftBit[iBit + 1], volt2mm.leftMm[iBit + 1], bitNum, mm);
    }
    else
    {
      *mm = volt2mm.leftMm[iBit]; 
    }
  }
  else
  {
    max = 196;
    for (i = 0; i < max; i++)
    {
      if (volt2mm.rightBit[i] >= bitNum)
      {
        iBit = i;
        i = max + 1;
      }
    }
    
    if (iBit < (max - 1))
    {
      Interpol2D(volt2mm.rightBit[iBit], volt2mm.rightMm[iBit], volt2mm.rightBit[iBit + 1], volt2mm.rightMm[iBit + 1], bitNum, mm);
    }
    else
    {
      *mm = volt2mm.rightMm[iBit]; 
    }
  }
}

void CrabVoltToMm (float volt, float *mm, CrabActuator_t act)
{
  UINT8 i, iVolt;
  UINT8 max;
  
  if (act == LEFT_ACTUATOR)
  {
    max = 174;
    for (i = 0; i < max; i++)
    {
      if (volt2mm.leftVolt[i] >= volt)
      {
        iVolt = i;
        i = max + 1;
      }
    }
    if (iVolt < (max - 1))
    {
      Interpol2D(volt2mm.leftVolt[iVolt], volt2mm.leftMm[iVolt], volt2mm.leftVolt[iVolt + 1], volt2mm.leftMm[iVolt + 1], volt, mm);
    }
    else
    {
      *mm = volt2mm.leftMm[iVolt]; 
    }
  }
  else
  {
    max = 196;
    for (i = 0; i < max; i++)
    {
      if (volt2mm.rightVolt[i] >= volt)
      {
        iVolt = i;
        i = max + 1;
      }
    }
    
    if (iVolt < (max - 1))
    {
      Interpol2D(volt2mm.rightVolt[iVolt], volt2mm.rightMm[iVolt], volt2mm.rightVolt[iVolt + 1], volt2mm.rightMm[iVolt + 1], volt, mm);
    }
    else
    {
      *mm = volt2mm.rightMm[iVolt]; 
    }
  }
}

void Interpol2D (float x0, float y0, float x1, float y1, float x, float *y)
{
  *y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}