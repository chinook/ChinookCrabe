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
// Purpose : This is a template header file that every developper should use as
//           a starter when developping code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Notes   : Function names can and should be renamed by the user to improve the
//           readability of the code.
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#include "..\headers\DRV8711_Para.h"



//==============================================================================
// Private functions prototypes
//==============================================================================


//==============================================================================
// Variable definitions
//==============================================================================


//==============================================================================
// Functions
//==============================================================================

/**************************************************************
 * Function name  : WriteDrive
 * Purpose        : Write a cmd to the drive.
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
void WriteDrive(INT32 DRV, INT32 msg)
{
  int i=0;

  if(DRV == DRVA)
  {
    while(SpiChnIsBusy(SPI4+1));
    DRVA_SC = 1;
    Spi.SendCharacter(SPI4, msg);
    for (i = 0; i < 1000; i++);   // Small delay necessary for the drive
    while(SpiChnIsBusy(SPI4+1));
    DRVA_SC = 0;
  }
  else if(DRV == DRVB)
  {
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 1;
    Spi.SendCharacter(SPI4, msg);
    for (i = 0; i < 1000; i++);   // Small delay necessary for the drive
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 0;
  }
  else if(DRV == DRVAB)
  {
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 1;
    DRVA_SC = 1;
    Spi.SendCharacter(SPI4, msg);
    for (i = 0; i < 1000; i++);   // Small delay necessary for the drive
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 0;
    DRVA_SC = 0;
  }
}


/**************************************************************
 * Function name  : ReadDrive
 * Purpose        : Read a msg from the drive. Not used
 * Arguments      : None.
 * Returns        : None.
 *************************************************************/
INT32 ReadDrive(INT32 DRV, INT32 msg)
{
  int i=0;
  INT32 data;
  if(DRV == DRVA)
  {
    while(SpiChnIsBusy(SPI4+1));
    DRVA_SC = 1;
//    Spi.SendCharacter(SPI4, msg | (1 << 15));
    Spi.SendCharacter(SPI4, msg);
    while(SpiChnIsBusy(SPI4+1));
    data = Spi.GetCharacter(SPI4);
    while(SpiChnIsBusy(SPI4+1));
    data = Spi.GetCharacter(SPI4);
    DRVA_SC = 0;
    
  }
  else if(DRV == DRVB){
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 1;
//    Spi.SendCharacter(SPI4, msg | (1 << 15));
    Spi.SendCharacter(SPI4, msg);
    while(SpiChnIsBusy(SPI4+1));
    data = Spi.GetCharacter(SPI4);
    while(SpiChnIsBusy(SPI4+1));
    data = Spi.GetCharacter(SPI4);
    DRVB_SC = 0;
  }
  else if(DRV == DRVAB){
    while(SpiChnIsBusy(SPI4+1));
    DRVB_SC = 1;
    DRVA_SC = 1;
//    Spi.SendCharacter(SPI4, msg | (1 << 15));
    Spi.SendCharacter(SPI4, msg);
    while(SpiChnIsBusy(SPI4+1));
    data = Spi.GetCharacter(SPI4);
    DRVB_SC = 0;
    DRVA_SC = 0;
  }
  return data;
}


void InitDriver(void)
{
  // CTRL register
  Drive_Mast_Setup.Ctl.ENABLE   = 0b1;            //0 = disable, 1 = enable
  Drive_Mast_Setup.Ctl.RDIR     = 0b0;            //0 = use DIR pin, 1 = use inverse DIR PIN
  Drive_Mast_Setup.Ctl.RSETEP   = 0b0;            //0 = nothing, 1 : give a step on stepper = STEP pin
  Drive_Mast_Setup.Ctl.MODE     = 0b0000;         //000 = FULL step.... 1000 = 1/256, other reserved
  Drive_Mast_Setup.Ctl.EXSTALL  = 0b0;            //0 = internal stall effect, 1 = external stall effect
  Drive_Mast_Setup.Ctl.ISGAIN   = 0b00;           //gain AOP,  00=5, 01=10, 10=20, 11=40
  Drive_Mast_Setup.Ctl.DTIME    = 0b00;           //dead time, 00=400ns, 01=450ns, 10=650ns, 11=850ns

  // TORQUE register
  Drive_Mast_Setup.Tor.TORQUE   = 0x80;           //torque value, see equation of max current
  Drive_Mast_Setup.Tor.SMPLTH   = 0b111;          //EMF sample : 000 = 50us, 001 = 100us, 010 = 200us, 011 = 300us, 100 = 400us, 101 = 600us, 110 = 800us, 111 = 1000us

  // OFF register
  Drive_Mast_Setup.Off.TOFF     = 0xFF;           //set off time, increment of 500ns
  Drive_Mast_Setup.Off.PWMMODE  = 0b1;            //0 = use internal indexer, 1 = use xINx inputs to control output

  // BLANK register
  Drive_Mast_Setup.Bla.TBLANK   = 0x80;           //set current trip blank time
  Drive_Mast_Setup.Bla.ABT      = 0b0;            //0 = disable adaptive blamking time, 1 = enable adaptive blamking time

  // DECAY register
  Drive_Mast_Setup.Dec.TDECAY   = 0x80;           //increment of 500ns
//  Drive_Mast_Setup.Dec.DECMOD   = 0b101;          //000 : force slow decay
  Drive_Mast_Setup.Dec.DECMOD   = 0b010;          //000 : force slow decay
                                                  //001 : slow decay for increasing current
                                                  //010 : force fast decay
                                                  //011 : use mixed decay
                                                  //100 : slow decay for increasing current
                                                  //101 : AUTO mixed decay
                                                  //110 -111 :reserved

  // STALL register
  Drive_Mast_Setup.Sll.SDTHR    = 0xFF;           //stall detec threshold, determined experimentaly
  Drive_Mast_Setup.Sll.SDCNT    = 0b11;           //STALLn asserted after, 00 = 1 step, 01 = 2 step, 10 = 4 step, 11 = 8 step
  Drive_Mast_Setup.Sll.VDIV     = 0b00;           //back EMF devided by, 00 = 32, 01 = 16, 10 = 8, 11 = 4

  // DRIVE register
  Drive_Mast_Setup.Dri.OCPTH    = 0b11;           //OCP threshold, 00 = 250mV, 01 = 500mV, 10 = 750mV, 11 = 1000mV,
  Drive_Mast_Setup.Dri.OCPDEG   = 0b11;           //OCP deglitch time, 00 = 1 us, 01 = 2 us, 10 = 4 us, 11 = 8 us
  Drive_Mast_Setup.Dri.TDRIVEN  = 0b11;           //Low-side gate drive time, 00 = 250 ns, 01 = 500 ns, 10 = 1 us, 11 = 2 us
  Drive_Mast_Setup.Dri.TDRIVEP  = 0b11;           //High-side gate drive time, 00 = 250 ns, 01 = 500 ns, 10 = 1 us, 11 = 2 us
  Drive_Mast_Setup.Dri.IDRIVEN  = 0b11;           //Low-side gate drive peak current, 00 = 100mA, 01 = 200mA, 10 = 300mA, 11 = 400mA
  Drive_Mast_Setup.Dri.IDRIVEP  = 0b11;           //High-side gate drive peak current, 00 = 100mA, 01 = 200mA, 10 = 300mA, 11 = 400mA

  // STATUS register
  Drive_Mast_Setup.Sta.OTS      = 0b0;            //0 = normale operation, 1 = overtemperature shutdown detected
  Drive_Mast_Setup.Sta.AOCP     = 0b0;            //0 = normale operation, 1 = channel A overcurrent shutdown
  Drive_Mast_Setup.Sta.BOCP     = 0b0;            //0 = normale operation, 1 = channel B overcurrent shutdown
  Drive_Mast_Setup.Sta.APDF     = 0b0;            //0 = normale operation, 1 = channel A predriver fault
  Drive_Mast_Setup.Sta.BPDF     = 0b0;            //0 = normale operation, 1 = channel B predriver fault
  Drive_Mast_Setup.Sta.UVLO     = 0b0;            //0 = normale operation, 1 = overvoltage lockout
  Drive_Mast_Setup.Sta.STD      = 0b0;            //0 = normale operation, 1 = stall detected
  Drive_Mast_Setup.Sta.STDLAT   = 0b0;            //0 = normale operation, 1 = latched stall detect


  // DRIVE B
  //==========================================================
  if (USE_DRIVE_B == 1)
  {
    Timer.DelayMs(1);
    DRVB_RESET = 0;
    Timer.DelayMs(1);
    DRVB_SLEEP = 0;
    Timer.DelayMs(1);

    WriteDrive(DRVB, CONTROL_Mastw);
    WriteDrive(DRVB, TORQUE_Mastw);
    WriteDrive(DRVB, OFF_Mastw);
    WriteDrive(DRVB, BLANK_Mastw);
    WriteDrive(DRVB, STALL_Mastw);
    WriteDrive(DRVB, DRIVE_Mastw);
    WriteDrive(DRVB, 0x2180);
    WriteDrive(DRVB, STATUS_Mastw);
  }
  //==========================================================


  // DRIVE A
  //==========================================================
  if (USE_DRIVE_A == 1)
  {
    Timer.DelayMs(1);
    DRVA_RESET = 0;
    Timer.DelayMs(1);
    DRVA_SLEEP = 0;
    Timer.DelayMs(1);

    WriteDrive(DRVA, CONTROL_Mastw);
    WriteDrive(DRVA, TORQUE_Mastw);
    WriteDrive(DRVA, OFF_Mastw);
    WriteDrive(DRVA, BLANK_Mastw);
    WriteDrive(DRVA, STALL_Mastw);
    WriteDrive(DRVA, DRIVE_Mastw);
    WriteDrive(DRVA, 0x2180);
    WriteDrive(DRVA, STATUS_Mastw);
  }
  //==========================================================

  DRVA_SC = 0;
  DRVB_SC = 0;
}