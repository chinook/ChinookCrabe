//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// Chinook Project Template
//
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// File    : HeaderTemplate.h
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


#ifndef HEADER_TEMPLATE_H
#define	HEADER_TEMPLATE_H

#include "Setup.h"


//==============================================================================
// Public functions prototypes
//==============================================================================

void  WriteDrive(INT32 DRV, INT32 msg);
INT32 ReadDrive (INT32 DRV, INT32 msg);
void  InitDriver(void);


//==============================================================================
// Macro definitions
//==============================================================================
drvReg Drive_Mast_Setup;
drvReg Drive_Pitch_Setup;

typedef union
{
  struct
  {
    UINT16  enable  : 1
           ,rdir    : 1
           ,rstep   : 1
           ,mode    : 4
           ,exstall : 1
           ,isgain  : 2
           ,dtime   : 2
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} CtrlReg_t;

typedef union
{
  struct
  {
    UINT16  torque  : 8
           ,smplth  : 3
           ,        : 1
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} TorqueReg_t;

typedef union
{
  struct
  {
    UINT16  toff    : 8
           ,pwmmode : 1
           ,        : 3
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} OffReg_t;

typedef union
{
  struct
  {
    UINT16  tblank  : 8
           ,abt     : 1
           ,        : 3
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} BlankReg_t;

typedef union
{
  struct
  {
    UINT16  tdecay  : 8
           ,decmod  : 3
           ,        : 1
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} DecayReg_t;

typedef union
{
  struct
  {
    UINT16  sdthr   : 8
           ,sdcnt   : 2
           ,vdiv    : 2
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} StallReg_t;

typedef union
{
  struct
  {
    UINT16  ocpth   : 2
           ,ocpdeg  : 2
           ,tdriven : 2
           ,trivep  : 2
           ,idriven : 2
           ,idrivep : 2
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} DriveReg_t;

typedef union
{
  struct
  {
    UINT16  ots     : 1
           ,aocp    : 1
           ,bocp    : 1
           ,apdf    : 1
           ,bpdf    : 1
           ,uvlo    : 1
           ,std     : 1
           ,stdlat  : 1
           ,        : 4
           ,reg     : 3
           ,rw      : 1
           ;
  } bits;
  UINT16 word;
} StatusReg_t;


///* Register adresse */
#define regCTRLadd   0b000
#define regTORQUEadd 0b001
#define regOFFadd    0b010
#define regBLANKadd  0b011
#define regDECAYadd  0b100
#define regSTALLadd  0b101
#define regDRIVEadd  0b110
#define regSTATUSadd 0b111

/* Concatenation to build SPI message*/
  /**** write message 1|adresse(3)|configuration(12) ****/
#define CONTROL_Mastw      0<<15|regCTRLadd<<12|Drive_Mast_Setup.Ctl.DTIME<<10|Drive_Mast_Setup.Ctl.ISGAIN<<8|Drive_Mast_Setup.Ctl.EXSTALL<<7|Drive_Mast_Setup.Ctl.MODE<<3|Drive_Mast_Setup.Ctl.RSETEP<<2|Drive_Mast_Setup.Ctl.RDIR<<1|Drive_Mast_Setup.Ctl.ENABLE
#define TORQUE_Mastw       0<<15|regTORQUEadd<<12|Drive_Mast_Setup.Tor.SMPLTH<<8|Drive_Mast_Setup.Tor.TORQUE
#define OFF_Mastw          0<<15|regOFFadd<<12|Drive_Mast_Setup.Off.PWMMODE<<8|Drive_Mast_Setup.Off.TOFF
#define BLANK_Mastw        0<<15|regBLANKadd<<12|Drive_Mast_Setup.Bla.ABT<<8|Drive_Mast_Setup.Bla.TBLANK
#define DECAY_Mastw        0<<15|regDECAYadd<<12|Drive_Mast_Setup.Dec.DECMOD<<8|Drive_Mast_Setup.Dec.TDECAY
#define STALL_Mastw        0<<15|regSTALLadd<<12|Drive_Mast_Setup.Sll.VDIV<<10|Drive_Mast_Setup.Sll.SDCNT<<8|Drive_Mast_Setup.Sll.SDTHR
#define DRIVE_Mastw        0<<15|regDRIVEadd<<12|Drive_Mast_Setup.Dri.IDRIVEP<<10|Drive_Mast_Setup.Dri.IDRIVEN<<8|Drive_Mast_Setup.Dri.TDRIVEP<<6|Drive_Mast_Setup.Dri.TDRIVEN<<4|Drive_Mast_Setup.Dri.OCPDEG<<2|Drive_Mast_Setup.Dri.OCPTH
#define STATUS_Mastw       0<<15|regSTATUSadd<<12|Drive_Mast_Setup.Sta.STDLAT<<7|Drive_Mast_Setup.Sta.STD<<6|Drive_Mast_Setup.Sta.UVLO<<5|Drive_Mast_Setup.Sta.BPDF<<4|Drive_Mast_Setup.Sta.APDF<<3|Drive_Mast_Setup.Sta.BOCP<<2|Drive_Mast_Setup.Sta.AOCP<<1|Drive_Mast_Setup.Sta.OTS

#define CONTROL_PITCHw     0<<15|regCTRLadd<<12|Drive_Pitch_Setup.Ctl.DTIME<<10|Drive_Pitch_Setup.Ctl.ISGAIN<<8|Drive_Pitch_Setup.Ctl.EXSTALL<<7|Drive_Pitch_Setup.Ctl.MODE<<3|Drive_Pitch_Setup.Ctl.RSETEP<<2|Drive_Pitch_Setup.Ctl.RDIR<<1|Drive_Pitch_Setup.Ctl.ENABLE
#define TORQUE_PITCHw      0<<15|regTORQUEadd<<12|Drive_Pitch_Setup.Tor.SMPLTH<<8|Drive_Pitch_Setup.Tor.TORQUE
#define OFF_PITCHw         0<<15|regOFFadd<<12|Drive_Pitch_Setup.Off.PWMMODE<<8|Drive_Pitch_Setup.Off.TOFF
#define BLANK_PITCHw       0<<15|regBLANKadd<<12|Drive_Pitch_Setup.Bla.ABT<<8|Drive_Pitch_Setup.Bla.TBLANK
#define DECAY_PITCHw       0<<15|regDECAYadd<<12|Drive_Pitch_Setup.Dec.DECMOD<<8|Drive_Pitch_Setup.Dec.TDECAY
#define STALL_PITCHw       0<<15|regSTALLadd<<12|Drive_Pitch_Setup.Sll.VDIV<<10|Drive_Pitch_Setup.Sll.SDCNT<<8|Drive_Pitch_Setup.Sll.SDTHR
#define DRIVE_PITCHw       0<<15|regDRIVEadd<<12|Drive_Pitch_Setup.Dri.IDRIVEP<<10|Drive_Pitch_Setup.Dri.IDRIVEN<<8|Drive_Pitch_Setup.Dri.TDRIVEP<<6|Drive_Pitch_Setup.Dri.TDRIVEN<<4|Drive_Pitch_Setup.Dri.OCPDEG<<2|Drive_Pitch_Setup.Dri.OCPTH
#define STATUS_PITCHw      0<<15|regSTATUSadd<<12|Drive_Pitch_Setup.Sta.STDLAT<<7|Drive_Pitch_Setup.Sta.STD<<6|Drive_Pitch_Setup.Sta.UVLO<<5|Drive_Pitch_Setup.Sta.BPDF<<4|Drive_Pitch_Setup.Sta.APDF<<3|Drive_Pitch_Setup.Sta.BOCP<<2|Drive_Pitch_Setup.Sta.AOCP<<1|Drive_Pitch_Setup.Sta.OTS

#define     DRVA        1
#define     DRVB        2
#define     DRVAB       3

//==============================================================================
// Variable declarations
//==============================================================================


#endif	/* HEADER_TEMPLATE_H */

