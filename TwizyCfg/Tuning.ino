/**
 * ==========================================================================
 * Twizy/SEVCON configuration shell
 * ==========================================================================
 * 
 * Twizy/SEVCON tuning functions
 * 
 * Based on the OVMS Twizy firmware:
 * https://github.com/openvehicles/Open-Vehicle-Monitoring-System
 * 
 * Author: Michael Balzer <dexter@dexters-web.de>
 * 
 * License:
 *  This is free software under GNU Lesser General Public License (LGPL)
 *  https://www.gnu.org/licenses/lgpl.html
 *  
 */

#include <EEPROM.h>

#include "utils.h"
#include "CANopen.h"
#include "Tuning.h"


// SEVCON configuration status:

cfg_status twizy_cfg;


// SEVCON macro configuration profile:

cfg_profile twizy_cfg_profile;


unsigned int twizy_max_rpm;         // CFG: max speed (RPM: 0..11000)
unsigned long twizy_max_trq;        // CFG: max torque (mNm: 0..70125)
unsigned int twizy_max_pwr_lo;      // CFG: max power low speed (W: 0..17000)
unsigned int twizy_max_pwr_hi;      // CFG: max power high speed (W: 0..17000)

UINT8 twizy_autorecup_checkpoint;   // change detection for autorecup function
UINT twizy_autorecup_level;         // autorecup: current recup level (per mille)

UINT8 twizy_autodrive_checkpoint;   // change detection for autopower function
UINT twizy_autodrive_level;         // autopower: current drive level (per mille)


/***********************************************************************
 * COMMAND CLASS: SEVCON CONTROLLER CONFIGURATION
 *
 *  MSG: ...todo...
 *  SMS: CFG [cmd]
 *
 */

struct twizy_cfg_params {
  UINT8   DefaultKphMax;
  UINT    DefaultRpmMax;
  UINT    DefaultRpmRev;
  UINT    DeltaBrkStart;
  UINT    DeltaBrkEnd;
  UINT    DeltaBrkDown;

  UINT8   DefaultKphWarn;
  UINT    DefaultRpmWarn;
  UINT    DeltaWarnOff;

  UINT    DefaultTrq;
  UINT    DefaultTrqRated;
  UINT32  DefaultTrqLim;
  UINT    DeltaMapTrq;
  
  UINT32  DefaultCurrLim;
  UINT    DefaultCurrStatorMax;
  UINT    BoostCurr;
  UINT    DefaultFMAP[4]; // only upper 2 points needed
  UINT    ExtendedFMAP[4]; // only upper 2 points needed

  UINT    DefaultPwrLo;
  UINT    DefaultPwrLoLim;
  UINT    DefaultPwrHi;
  UINT    DefaultPwrHiLim;
  UINT    DefaultMaxMotorPwr;

  UINT8   DefaultRecup;
  UINT8   DefaultRecupPrc;

  UINT    DefaultRampStart;
  UINT8   DefaultRampStartPrm;
  UINT    DefaultRampAccel;
  UINT8   DefaultRampAccelPrc;

  UINT    DefaultPMAP[18];
  
  UINT    DefaultMapSpd[4];
};

struct twizy_cfg_params twizy_cfg_params[2] =
{
  {
    //
    // CFG[0] = TWIZY80:
    //
    //    55 Nm (0x4611.0x01) = default max power map (PMAP) torque
    //    55 Nm (0x6076.0x00) = default peak torque
    //    57 Nm (0x2916.0x01) = rated torque (??? should be equal to 0x6076...)
    //    70.125 Nm (0x4610.0x11) = max motor torque according to flux map
    //
    //    7250 rpm (0x2920.0x05) = default max fwd speed = ~80 kph
    //    8050 rpm = default overspeed warning trigger (STOP lamp ON) = ~89 kph
    //    8500 rpm = default overspeed brakedown trigger = ~94 kph
    //    10000 rpm = max neutral speed (0x3813.2d) = ~110 kph
    //    11000 rpm = severe overspeed fault (0x4624.00) = ~121 kph

    80,     // DefaultKphMax
    7250,   // DefaultRpmMax
    900,    // DefaultRpmMaxRev
    400,    // DeltaBrkStart
    800,    // DeltaBrkEnd
    1250,   // DeltaBrkDown

    89,     // DefaultKphWarn
    8050,   // DefaultRpmWarn
    550,    // DeltaWarnOff

    55000,  // DefaultTrq
    57000,  // DefaultTrqRated
    70125,  // DefaultTrqLim
    0,      // DeltaMapTrq
    
    450000, // DefaultCurrLim
    450,    // DefaultCurrStatorMax
    540,    // BoostCurr
    { 964, 9728, 1122, 9984 }, // DefaultFMAP
    { 1122, 10089, 2240, 11901 }, // ExtendedFMAP

    12182,  // DefaultPwrLo
    17000,  // DefaultPwrLoLim
    13000,  // DefaultPwrHi
    17000,  // DefaultPwrHiLim
    4608,   // DefaultMaxMotorPwr [1/256 kW]

    182,    // DefaultRecup
    18,     // DefaultRecupPrc

    400,    // DefaultRampStart
    40,     // DefaultRampStartPrm
    2500,   // DefaultRampAccel
    25,     // DefaultRampAccelPrc

            // DefaultPMAP:
    { 880,0, 880,2115, 659,2700, 608,3000, 516,3500,
      421,4500, 360,5500, 307,6500, 273,7250 },

            // DefaultMapSpd:
    { 3000, 3500, 4500, 6000 }
  },
  {
    //
    // CFG[1] = TWIZY45:
    //
    //    32.5 Nm (0x4611.0x01) = default max power map (PMAP) torque
    //    33 Nm (0x6076.0x00) = default peak torque
    //    33 Nm (0x2916.0x01) = rated torque (??? should be equal to 0x6076...)
    //    36 Nm (0x4610.0x11) = max motor torque according to flux map
    //
    //    5814 rpm (0x2920.0x05) = default max fwd speed = ~45 kph
    //    7200 rpm = default overspeed warning trigger (STOP lamp ON) = ~56 kph
    //    8500 rpm = default overspeed brakedown trigger = ~66 kph
    //    10000 rpm = max neutral speed (0x3813.2d) = ~77 kph
    //    11000 rpm = severe overspeed fault (0x4624.00) = ~85 kph

    45,     // DefaultKphMax
    5814,   // DefaultRpmMax
    1307,   // DefaultRpmMaxRev
    686,    // DeltaBrkStart
    1386,   // DeltaBrkEnd
    2686,   // DeltaBrkDown

    56,     // DefaultKphWarn
    7200,   // DefaultRpmWarn
    900,    // DeltaWarnOff

    32500,  // DefaultTrq
    33000,  // DefaultTrqRated
    36000,  // DefaultTrqLim
    500,    // DeltaMapTrq

    270000, // DefaultCurrLim
    290,    // DefaultCurrStatorMax
    330,    // BoostCurr
    { 480, 8192, 576, 8960 }, // DefaultFMAP
    { 656, 9600, 1328, 11901 }, // ExtendedFMAP
    
    7050,   // DefaultPwrLo
    10000,  // DefaultPwrLoLim
    7650,   // DefaultPwrHi
    10000,  // DefaultPwrHiLim
    2688,   // DefaultMaxMotorPwr [1/256 kW]

    209,    // DefaultRecup
    21,     // DefaultRecupPrc

    300,    // DefaultRampStart
    30,     // DefaultRampStartPrm
    2083,   // DefaultRampAccel
    21,     // DefaultRampAccelPrc

            // DefaultPMAP:
    { 520,0, 520,2050, 437,2500, 363,3000, 314,3500,
      279,4000, 247,4500, 226,5000, 195,6000 },

            // DefaultMapSpd:
    { 4357, 5083, 6535, 8714 }
  }
};

// ROM parameter access macro:
#define CFG twizy_cfg_params[twizy_cfg.type]


// scale utility:
//  returns <deflt> value scaled <from> <to> limited by <min> and <max>
UINT32 scale(UINT32 deflt, UINT32 from, UINT32 to, UINT32 min, UINT32 max)
{
  UINT32 val;

  if (to == from)
    return deflt;

  val = (deflt * to) / from;

  if (val < min)
    return min;
  else if (val > max)
    return max;
  else
    return val;
}


// vehicle_twizy_cfg_makepowermap():
//  Internal utility for cfg_speed() and cfg_power()
//  builds torque/speed curve (SDO 0x4611) for current max values...
//  - twizy_max_rpm
//  - twizy_max_trq
//  - twizy_max_pwr_lo
//  - twizy_max_pwr_hi
// See "Twizy Powermap Calculator" spreadsheet for details.

UINT8 pmap_fib[7] = { 1, 2, 3, 5, 8, 13, 21 };

UINT vehicle_twizy_cfg_makepowermap(void)
  /* twizy_max_rpm, twizy_max_trq, twizy_max_pwr_lo, twizy_max_pwr_hi */
{
  UINT err;
  UINT8 i;
  UINT rpm_0, rpm;
  UINT trq;
  UINT pwr;
  int rpm_d, pwr_d;

  if (twizy_max_rpm == CFG.DefaultRpmMax && twizy_max_trq == CFG.DefaultTrq
    && twizy_max_pwr_lo == CFG.DefaultPwrLo && twizy_max_pwr_hi == CFG.DefaultPwrHi) {

    // restore default torque map:
    for (i=0; i<18; i++) {
      if (err = writesdo(0x4611,0x01+i,CFG.DefaultPMAP[i]))
        return err;
    }
    
    // restore default flux map (only last 2 points):
    for (i=0; i<4; i++) {
      if (err = writesdo(0x4610,0x0f+i,CFG.DefaultFMAP[i]))
        return err;
    }
  }

  else {

    // calculate constant torque part:

    rpm_0 = (((UINT32)twizy_max_pwr_lo * 9549 + (twizy_max_trq>>1)) / twizy_max_trq);
    trq = (twizy_max_trq * 16 + 500) / 1000;

    if (err = writesdo(0x4611,0x01,trq))
      return err;
    if (err = writesdo(0x4611,0x02,0))
      return err;
    if (err = writesdo(0x4611,0x03,trq))
      return err;
    if (err = writesdo(0x4611,0x04,rpm_0))
      return err;

    // adjust flux map (only last 2 points):
    
    if (trq > CFG.DefaultFMAP[2]) {
      for (i=0; i<4; i++) {
        if (err = writesdo(0x4610,0x0f+i,CFG.ExtendedFMAP[i]))
          return err;
      }
    }
    else {
      for (i=0; i<4; i++) {
        if (err = writesdo(0x4610,0x0f+i,CFG.DefaultFMAP[i]))
          return err;
      }
    }
    
    // calculate constant power part:

    if (twizy_max_rpm > rpm_0)
      rpm_d = (twizy_max_rpm - rpm_0 + (pmap_fib[6]>>1)) / pmap_fib[6];
    else
      rpm_d = 0;

    pwr_d = ((int)twizy_max_pwr_hi - (int)twizy_max_pwr_lo + (pmap_fib[5]>>1)) / pmap_fib[5];

    for (i=0; i<7; i++) {

      if (i<6)
        rpm = rpm_0 + (pmap_fib[i] * rpm_d);
      else
        rpm = twizy_max_rpm;

      if (i<5)
        pwr = twizy_max_pwr_lo + (pmap_fib[i] * pwr_d);
      else
        pwr = twizy_max_pwr_hi;

      trq = ((((UINT32)pwr * 9549 + (rpm>>1)) / rpm) * 16 + 500) / 1000;

      if (err = writesdo(0x4611,0x05+(i<<1),trq))
        return err;
      if (err = writesdo(0x4611,0x06+(i<<1),rpm))
        return err;

    }

  }

  // commit map changes:
  if (err = writesdo(0x4641,0x01,1))
    return err;
  delay5(10);

  return 0;
}


// vehicle_twizy_cfg_readmaxpwr:
// read twizy_max_pwr_lo & twizy_max_pwr_hi
// from SEVCON registers (only if necessary / undefined)
UINT vehicle_twizy_cfg_readmaxpwr(void)
{
  UINT err;
  UINT rpm;

  // the controller does not store max power, derive it from rpm/trq:

  if (twizy_max_pwr_lo == 0) {
    if (err = readsdo(0x4611,0x04))
      return err;
    rpm = twizy_sdo.data;
    if (err = readsdo(0x4611,0x03))
      return err;
    if (twizy_sdo.data == CFG.DefaultPMAP[2] && rpm == CFG.DefaultPMAP[3])
      twizy_max_pwr_lo = CFG.DefaultPwrLo;
    else
      twizy_max_pwr_lo = (UINT)((((twizy_sdo.data*1000)>>4) * rpm + (9549>>1)) / 9549);
  }

  if (twizy_max_pwr_hi == 0) {
    if (err = readsdo(0x4611,0x12))
      return err;
    rpm = twizy_sdo.data;
    if (err = readsdo(0x4611,0x11))
      return err;
    if (twizy_sdo.data == CFG.DefaultPMAP[16] && rpm == CFG.DefaultPMAP[17])
      twizy_max_pwr_hi = CFG.DefaultPwrHi;
    else
      twizy_max_pwr_hi = (UINT)((((twizy_sdo.data*1000)>>4) * rpm + (9549>>1)) / 9549);
  }

  return 0;
}


UINT vehicle_twizy_cfg_speed(int max_kph, int warn_kph)
// max_kph: 6..?, -1=reset to default (80)
// warn_kph: 6..?, -1=reset to default (89)
{
  UINT err;
  UINT rpm;

  CHECKPOINT (41)

  // parameter validation:

  if (max_kph == -1)
    max_kph = CFG.DefaultKphMax;
  else if (max_kph < 6)
    return ERR_Range + 1;

  if (warn_kph == -1)
    warn_kph = CFG.DefaultKphWarn;
  else if (warn_kph < 6)
    return ERR_Range + 2;


  // get max torque for map scaling:

  if (twizy_max_trq == 0) {
    if (err = readsdo(0x6076,0x00))
      return err;
    twizy_max_trq = twizy_sdo.data - CFG.DeltaMapTrq;
  }

  // get max power for map scaling:

  if (err = vehicle_twizy_cfg_readmaxpwr())
    return err;
  
  // set overspeed warning range (STOP lamp):
  rpm = scale(CFG.DefaultRpmWarn,CFG.DefaultKphWarn,warn_kph,400,65535);
  if (err = writesdo(0x3813,0x34,rpm)) // lamp ON
    return err;
  if (err = writesdo(0x3813,0x3c,rpm-CFG.DeltaWarnOff)) // lamp OFF
    return err;

  // calc fwd rpm:
  rpm = scale(CFG.DefaultRpmMax,CFG.DefaultKphMax,max_kph,400,65535);

  // set fwd rpm:
  err = writesdo(0x2920,0x05,rpm);
  if (err)
    return err;

  // set rev rpm:
  err = writesdo(0x2920,0x06,LIMIT_MAX(rpm,CFG.DefaultRpmRev));
  if (err)
    return err;

  // adjust overspeed braking points (using fixed offsets):
  if (err = writesdo(0x3813,0x33,rpm+CFG.DeltaBrkStart)) // neutral braking start
    return err;
  if (err = writesdo(0x3813,0x35,rpm+CFG.DeltaBrkEnd)) // neutral braking end
    return err;
  if (err = writesdo(0x3813,0x3b,rpm+CFG.DeltaBrkDown)) // drive brakedown trigger
    return err;

  // adjust overspeed limits:
  if (err = writesdo(0x3813,0x2d,rpm+CFG.DeltaBrkDown+1500)) // neutral max speed
    return err;
  if (err = writesdo(0x4624,0x00,rpm+CFG.DeltaBrkDown+2500)) // severe overspeed fault
    return err;
  
  twizy_max_rpm = rpm;

  return 0;
}


UINT vehicle_twizy_cfg_power(int trq_prc, int pwr_lo_prc, int pwr_hi_prc, int curr_prc)
// See "Twizy Powermap Calculator" spreadsheet.
// trq_prc: 10..130, -1=reset to default (100%)
// i.e. TWIZY80:
//    100% = 55.000 Nm
//    128% = 70.125 Nm (130 allowed for easy handling)
// pwr_lo_prc: 10..139, -1=reset to default (100%)
//    100% = 12182 W (mechanical)
//    139% = 16933 W (mechanical)
// pwr_hi_prc: 10..130, -1=reset to default (100%)
//    100% = 13000 W (mechanical)
//    130% = 16900 W (mechanical)
// curr_prc: 10..123, -1=reset to default current limits
//    100% = 450 A (Twizy 45: 270 A)
//    120% = 540 A (Twizy 45: 324 A)
//    123% = 540 A (Twizy 45: 330 A)
//    setting this to any value disables high limits of trq & pwr
//    (=> flux map extended to enable higher torque)
//    safety max limits = SEVCON model specific boost current level
{
  UINT err;
  BOOL limited = FALSE;

  // parameter validation:

  if (curr_prc == -1) {
    curr_prc = 100;
    limited = TRUE;
  }
  else if (curr_prc < 10 || curr_prc > 123)
    return ERR_Range + 4;

  if (trq_prc == -1)
    trq_prc = 100;
  else if (trq_prc < 10 || (limited && trq_prc > 130))
    return ERR_Range + 1;

  if (pwr_lo_prc == -1)
    pwr_lo_prc = 100;
  else if (pwr_lo_prc < 10 || (limited && pwr_lo_prc > 139))
    return ERR_Range + 2;

  if (pwr_hi_prc == -1)
    pwr_hi_prc = 100;
  else if (pwr_hi_prc < 10 || (limited && pwr_hi_prc > 130))
    return ERR_Range + 3;

  // get max fwd rpm for map scaling:
  if (twizy_max_rpm == 0) {
    if (err = readsdo(0x2920,0x05))
      return err;
    twizy_max_rpm = twizy_sdo.data;
  }

  // set current limits:
  if (err = writesdo(0x4641,0x02,scale(
          CFG.DefaultCurrStatorMax,100,curr_prc,0,CFG.BoostCurr)))
    return err;
  if (err = writesdo(0x6075,0x00,scale(
          CFG.DefaultCurrLim,100,curr_prc,0,CFG.BoostCurr*1000L)))
    return err;

  // calc peak use torque:
  twizy_max_trq = scale(CFG.DefaultTrq,100,trq_prc,10000,
    (limited) ? CFG.DefaultTrqLim : 200000);

  // set peak use torque:
  if (err = writesdo(0x6076,0x00,twizy_max_trq + CFG.DeltaMapTrq))
    return err;

  // set rated torque:
  if (err = writesdo(0x2916,0x01,(trq_prc==100)
          ? CFG.DefaultTrqRated
          : (twizy_max_trq + CFG.DeltaMapTrq)))
    return err;

  // calc peak use power:
  twizy_max_pwr_lo = scale(CFG.DefaultPwrLo,100,pwr_lo_prc,500,
    (limited) ? CFG.DefaultPwrLoLim : 200000);
  twizy_max_pwr_hi = scale(CFG.DefaultPwrHi,100,pwr_hi_prc,500,
    (limited) ? CFG.DefaultPwrHiLim : 200000);
  
  // set motor max power:
  if (err = writesdo(0x3813,0x23,(pwr_lo_prc==100 && pwr_hi_prc==100)
          ? CFG.DefaultMaxMotorPwr
          : MAX(twizy_max_pwr_lo,twizy_max_pwr_hi)*0.353))
    return err;

  return 0;
}


UINT vehicle_twizy_cfg_tsmap(
  char map,
  INT8 t1_prc, INT8 t2_prc, INT8 t3_prc, INT8 t4_prc,
  int t1_spd, int t2_spd, int t3_spd, int t4_spd)
// map: 'D'=Drive 'N'=Neutral 'B'=Footbrake
//
// t1_prc: 0..100, -1=reset to default (D=100, N/B=100)
// t2_prc: 0..100, -1=reset to default (D=100, N/B=80)
// t3_prc: 0..100, -1=reset to default (D=100, N/B=50)
// t4_prc: 0..100, -1=reset to default (D=100, N/B=20)
//
// t1_spd: 0..?, -1=reset to default (D/N/B=33)
// t2_spd: 0..?, -1=reset to default (D/N/B=39)
// t3_spd: 0..?, -1=reset to default (D/N/B=50)
// t4_spd: 0..?, -1=reset to default (D/N/B=66)
{
  UINT err;
  UINT8 base;
  UINT val, bndlo, bndhi;
  UINT8 todo;

  // parameter validation:

  if (map != 'D' && map != 'N' && map != 'B')
    return ERR_Range + 1;

  // torque points:

  if ((t1_prc != -1) && (t1_prc < 0 || t1_prc > 100))
    return ERR_Range + 2;
  if ((t2_prc != -1) && (t2_prc < 0 || t2_prc > 100))
    return ERR_Range + 3;
  if ((t3_prc != -1) && (t3_prc < 0 || t3_prc > 100))
    return ERR_Range + 4;
  if ((t4_prc != -1) && (t4_prc < 0 || t4_prc > 100))
    return ERR_Range + 5;

  // speed points:

  if ((t1_spd != -1) && (t1_spd < 0))
    return ERR_Range + 6;
  if ((t2_spd != -1) && (t2_spd < 0))
    return ERR_Range + 7;
  if ((t3_spd != -1) && (t3_spd < 0))
    return ERR_Range + 8;
  if ((t4_spd != -1) && (t4_spd < 0))
    return ERR_Range + 9;

  // get map base subindex in SDO 0x3813:

  if (map=='B')
    base = 0x07;
  else if (map=='N')
    base = 0x1b;
  else // 'D'
    base = 0x24;

  // set:
  // we need to adjust point by point to avoid the "Param dyn range" alert,
  // ensuring a new speed has no conflict with the previous surrounding points

  todo = 0x0f;

  while (todo) {

    // point 1:
    if (todo & 0x01) {

      // get speed boundaries:
      bndlo = 0;
      if (err = readsdo(0x3813,base+3))
        return err;
      bndhi = twizy_sdo.data;

      // calc new speed:
      if (t1_spd >= 0)
        val = scale(CFG.DefaultMapSpd[0],33,t1_spd,0,65535);
      else
        val = (map=='D') ? 3000 : CFG.DefaultMapSpd[0];

      if (val >= bndlo && val <= bndhi) {
        // ok, change:
        if (err = writesdo(0x3813,base+1,val))
          return err;

        if (t1_prc >= 0)
          val = scale(32767,100,t1_prc,0,32767);
        else
          val = 32767;
        if (err = writesdo(0x3813,base+0,val))
          return err;

        todo &= ~0x01;
      }
    }

    // point 2:
    if (todo & 0x02) {

      // get speed boundaries:
      if (err = readsdo(0x3813,base+1))
        return err;
      bndlo = twizy_sdo.data;
      if (err = readsdo(0x3813,base+5))
        return err;
      bndhi = twizy_sdo.data;

      // calc new speed:
      if (t2_spd >= 0)
        val = scale(CFG.DefaultMapSpd[1],39,t2_spd,0,65535);
      else
        val = (map=='D') ? 3500 : CFG.DefaultMapSpd[1];

      if (val >= bndlo && val <= bndhi) {
        // ok, change:
        if (err = writesdo(0x3813,base+3,val))
          return err;

        if (t2_prc >= 0)
          val = scale(32767,100,t2_prc,0,32767);
        else
          val = (map=='D') ? 32767 : 26214;
        if (err = writesdo(0x3813,base+2,val))
          return err;

        todo &= ~0x02;
      }
    }

    // point 3:
    if (todo & 0x04) {

      // get speed boundaries:
      if (err = readsdo(0x3813,base+3))
        return err;
      bndlo = twizy_sdo.data;
      if (err = readsdo(0x3813,base+7))
        return err;
      bndhi = twizy_sdo.data;

      // calc new speed:
      if (t3_spd >= 0)
        val = scale(CFG.DefaultMapSpd[2],50,t3_spd,0,65535);
      else
        val = (map=='D') ? 4500 : CFG.DefaultMapSpd[2];

      if (val >= bndlo && val <= bndhi) {
        // ok, change:
        if (err = writesdo(0x3813,base+5,val))
          return err;

        if (t3_prc >= 0)
          val = scale(32767,100,t3_prc,0,32767);
        else
          val = (map=='D') ? 32767 : 16383;
        if (err = writesdo(0x3813,base+4,val))
          return err;

        todo &= ~0x04;
      }
    }

    // point 4:
    if (todo & 0x08) {

      // get speed boundaries:
      if (err = readsdo(0x3813,base+5))
        return err;
      bndlo = twizy_sdo.data;
      bndhi = 65535;

      // calc new speed:
      if (t4_spd >= 0)
        val = scale(CFG.DefaultMapSpd[3],66,t4_spd,0,65535);
      else
        val = (map=='D') ? 6000 : CFG.DefaultMapSpd[3];

      if (val >= bndlo && val <= bndhi) {
        // ok, change:
        if (err = writesdo(0x3813,base+7,val))
          return err;

        if (t4_prc >= 0)
          val = scale(32767,100,t4_prc,0,32767);
        else
          val = (map=='D') ? 32767 : 6553;
        if (err = writesdo(0x3813,base+6,val))
          return err;

        todo &= ~0x08;
      }
    }

  } // while (todo)


  return 0;
}


UINT vehicle_twizy_cfg_drive(int max_prc, int autodrive_ref, int autodrive_minprc)
// max_prc: 10..100, -1=reset to default (100)
// autodrive_ref: 0..250, -1=default (off) (not a direct SEVCON control)
//      sets power 100% ref point to <autodrive_ref> * 100 W
// autodrive_minprc: 0..100, -1=default (off) (not a direct SEVCON control)
//      sets lower limit on auto power adjustment
{
  UINT err;

  // parameter validation:

  if (max_prc == -1)
    max_prc = 100;
  else if (max_prc < 10 || max_prc > 100)
    return ERR_Range + 1;

#ifdef OVMS_TWIZY_BATTMON

  if (autodrive_ref == -1)
    ;
  else if (autodrive_ref < 0 || autodrive_ref > 250)
    return ERR_Range + 2;

  if (autodrive_minprc == -1)
    autodrive_minprc = 0;
  else if (autodrive_minprc < 0 || autodrive_minprc > 100)
    return ERR_Range + 3;

  if ((autodrive_ref > 0) && (!sys_can.DisableAutoPower))
  {
    // calculate max drive level:
    twizy_autodrive_level = LIMIT_MAX(((long) twizy_batt[0].max_drive_pwr * 5L * 1000L)
            / autodrive_ref, 1000);
    twizy_autodrive_level = LIMIT_MIN(twizy_autodrive_level, autodrive_minprc * 10);

    // set autopower checkpoint:
    twizy_autodrive_checkpoint = (twizy_batt[0].max_drive_pwr + autodrive_ref + autodrive_minprc);
  }
  else
  {
    twizy_autodrive_level = 1000;
  }

#else

  twizy_autodrive_level = 1000;
  
#endif //OVMS_TWIZY_BATTMON

  // set:
  if (err = writesdo(0x2920,0x01,LIMIT_MAX(max_prc*10, twizy_autodrive_level)))
    return err;

  return 0;
}


UINT vehicle_twizy_cfg_recup(int neutral_prc, int brake_prc, int autorecup_ref, int autorecup_minprc)
// neutral_prc: 0..100, -1=reset to default (18)
// brake_prc: 0..100, -1=reset to default (18)
// autorecup_ref: 0..250, -1=default (off) (not a direct SEVCON control)
//      ATT: parameter function changed in V3.6.0!
//      now sets power 100% ref point to <autorecup_ref> * 100 W
// autorecup_minprc: 0..100, -1=default (off) (not a direct SEVCON control)
//      sets lower limit on auto power adjustment
{
  UINT err;
  UINT level;

  // parameter validation:

  if (neutral_prc == -1)
    neutral_prc = CFG.DefaultRecupPrc;
  else if (neutral_prc < 0 || neutral_prc > 100)
    return ERR_Range + 1;

  if (brake_prc == -1)
    brake_prc = CFG.DefaultRecupPrc;
  else if (brake_prc < 0 || brake_prc > 100)
    return ERR_Range + 2;

#ifdef OVMS_TWIZY_BATTMON

  if (autorecup_ref == -1)
    ;
  else if (autorecup_ref < 0 || autorecup_ref > 250)
    return ERR_Range + 3;

  if (autorecup_minprc == -1)
    autorecup_minprc = 0;
  else if (autorecup_minprc < 0 || autorecup_minprc > 100)
    return ERR_Range + 4;

  if ((autorecup_ref > 0) && (!sys_can.DisableAutoPower))
  {
    // calculate max recuperation level:
    twizy_autorecup_level = LIMIT_MAX(((long) twizy_batt[0].max_recup_pwr * 5L * 1000L)
            / autorecup_ref, 1000);
    twizy_autorecup_level = LIMIT_MIN(twizy_autorecup_level, autorecup_minprc * 10);
    
    // set autopower checkpoint:
    twizy_autorecup_checkpoint = (twizy_batt[0].max_recup_pwr + autorecup_ref + autorecup_minprc);
  }
  else
  {
    twizy_autorecup_level = 1000;
  }

#else

  twizy_autorecup_level = 1000;
  
#endif //OVMS_TWIZY_BATTMON

  // set neutral recup level:
  level = scale(CFG.DefaultRecup,CFG.DefaultRecupPrc,neutral_prc,0,1000);
  if (twizy_autorecup_level != 1000)
      level = (((long) level) * twizy_autorecup_level) / 1000;
  if (err = writesdo(0x2920,0x03,level))
    return err;
  
  // set brake recup level:
  level = scale(CFG.DefaultRecup,CFG.DefaultRecupPrc,brake_prc,0,1000);
  if (twizy_autorecup_level != 1000)
      level = (((long) level) * twizy_autorecup_level) / 1000;
  if (err = writesdo(0x2920,0x04,level))
    return err;

  return 0;
}


#ifdef OVMS_TWIZY_BATTMON

// Auto recup & drive power update function:
// check for BMS max pwr change, update SEVCON settings accordingly
// this is called by vehicle_twizy_state_ticker1() = approx. once per second
void vehicle_twizy_cfg_autopower(void)
{
  int ref, minprc;

  // check for SEVCON write access:
  if ((!sys_can.EnableWrite) || (sys_can.DisableAutoPower)
          || ((twizy_status & CAN_STATUS_KEYON) == 0))
    return;

  // adjust recup levels?
  ref = cfgparam(autorecup_ref);
  minprc = cfgparam(autorecup_minprc);
  if ((ref > 0)
    && ((twizy_batt[0].max_recup_pwr + ref + minprc) != twizy_autorecup_checkpoint))
  {
    vehicle_twizy_cfg_recup(cfgparam(neutral), cfgparam(brake), ref, minprc);
  }
  
  // adjust drive level?
  ref = cfgparam(autodrive_ref);
  minprc = cfgparam(autodrive_minprc);
  if ((ref > 0)
    && ((twizy_batt[0].max_drive_pwr + ref + minprc) != twizy_autodrive_checkpoint)
    && (twizy_kickdown_hold == 0))
  {
    vehicle_twizy_cfg_drive(cfgparam(drive), ref, minprc);
  }
}

#endif //OVMS_TWIZY_BATTMON


UINT vehicle_twizy_cfg_ramps(int start_prm, int accel_prc, int decel_prc, int neutral_prc, int brake_prc)
// start_prm: 1..250, -1=reset to default (40) (Att! changed in V3.4 from prc to prm!)
// accel_prc: 1..100, -1=reset to default (25)
// decel_prc: 0..100, -1=reset to default (20)
// neutral_prc: 0..100, -1=reset to default (40)
// brake_prc: 0..100, -1=reset to default (40)
{
  UINT err;

  // parameter validation:

  if (start_prm == -1)
    start_prm = CFG.DefaultRampStartPrm;
  else if (start_prm < 1 || start_prm > 250)
    return ERR_Range + 1;

  if (accel_prc == -1)
    accel_prc = CFG.DefaultRampAccelPrc;
  else if (accel_prc < 1 || accel_prc > 100)
    return ERR_Range + 2;

  if (decel_prc == -1)
    decel_prc = 20;
  else if (decel_prc < 0 || decel_prc > 100)
    return ERR_Range + 3;

  if (neutral_prc == -1)
    neutral_prc = 40;
  else if (neutral_prc < 0 || neutral_prc > 100)
    return ERR_Range + 4;

  if (brake_prc == -1)
    brake_prc = 40;
  else if (brake_prc < 0 || brake_prc > 100)
    return ERR_Range + 5;

  // set:

  if (err = writesdo(0x291c,0x02,scale(CFG.DefaultRampStart,CFG.DefaultRampStartPrm,start_prm,10,10000)))
    return err;
  if (err = writesdo(0x2920,0x07,scale(CFG.DefaultRampAccel,CFG.DefaultRampAccelPrc,accel_prc,10,10000)))
    return err;
  if (err = writesdo(0x2920,0x0b,scale(2000,20,decel_prc,10,10000)))
    return err;
  if (err = writesdo(0x2920,0x0d,scale(4000,40,neutral_prc,10,10000)))
    return err;
  if (err = writesdo(0x2920,0x0e,scale(4000,40,brake_prc,10,10000)))
    return err;

  return 0;
}


UINT vehicle_twizy_cfg_rampl(int accel_prc, int decel_prc)
// accel_prc: 1..100, -1=reset to default (30)
// decel_prc: 0..100, -1=reset to default (30)
{
  UINT err;

  // parameter validation:

  if (accel_prc == -1)
    accel_prc = 30;
  else if (accel_prc < 1 || accel_prc > 100)
    return ERR_Range + 1;

  if (decel_prc == -1)
    decel_prc = 30;
  else if (decel_prc < 0 || decel_prc > 100)
    return ERR_Range + 2;

  // set:

  if (err = writesdo(0x2920,0x0f,scale(6000,30,accel_prc,0,20000)))
    return err;
  if (err = writesdo(0x2920,0x10,scale(6000,30,decel_prc,0,20000)))
    return err;

  return 0;
}


UINT vehicle_twizy_cfg_smoothing(int prc)
// prc: 0..100, -1=reset to default (70)
{
  UINT err;

  // parameter validation:

  if (prc == -1)
    prc = 70;
  else if (prc < 0 || prc > 100)
    return ERR_Range + 1;

  // set:
  if (err = writesdo(0x290a,0x01,1+(prc/10)))
    return err;
  if (err = writesdo(0x290a,0x03,scale(800,70,prc,0,1000)))
    return err;

  return 0;
}


#ifdef OVMS_TWIZY_CFG_BRAKELIGHT

UINT vehicle_twizy_cfg_brakelight(int on_lev, int off_lev)
// *** NOT FUNCTIONAL WITHOUT HARDWARE MODIFICATION ***
// *** SEVCON cannot control Twizy brake lights ***
// on_lev: 0..100, -1=reset to default (100=off)
// off_lev: 0..100, -1=reset to default (100=off)
// on_lev must be >= off_lev
// ctrl bit in 0x2910.1 will be set/cleared accordingly
{
  UINT err;

  // parameter validation:

  if (on_lev == -1)
    on_lev = 100;
  else if (on_lev < 0 || on_lev > 100)
    return ERR_Range + 1;

  if (off_lev == -1)
    off_lev = 100;
  else if (off_lev < 0 || off_lev > 100)
    return ERR_Range + 2;

  if (on_lev < off_lev)
    return ERR_Range + 3;

  // set range:
  if (err = writesdo(0x3813,0x05,scale(1024,100,off_lev,64,1024)))
    return err;
  if (err = writesdo(0x3813,0x06,scale(1024,100,on_lev,64,1024)))
    return err;

  // set ctrl bit:
  if (err = readsdo(0x2910,0x01))
    return err;
  if (on_lev != 100 || off_lev != 100)
    twizy_sdo.data |= 0x2000;
  else
    twizy_sdo.data &= ~0x2000;
  if (err = writesdo(0x2910,0x01,twizy_sdo.data))
    return err;

  return 0;
}

#endif // OVMS_TWIZY_CFG_BRAKELIGHT


// vehicle_twizy_cfg_calc_checksum: get checksum for twizy_cfg_profile
//
// Note: for extendability of struct twizy_cfg_profile, 0-Bytes will
//  not affect the checksum, so new fields can simply be added at the end
//  without losing version compatibility.
//  (0-bytes translate to value -1 = default)
BYTE vehicle_twizy_cfg_calc_checksum(BYTE *profile)
{
  UINT checksum;
  UINT8 i;

  checksum = 0x0101; // version tag

  for (i=1; i<sizeof(twizy_cfg_profile); i++)
    checksum += profile[i];

  if ((checksum & 0x0ff) == 0)
    checksum >>= 8;

  return (checksum & 0x0ff);
}


// vehicle_twizy_cfg_readprofile: read profile from params to twizy_cgf_profile
//    with checksum validation
//    it invalid checksum initialize to default config & return FALSE
BOOL vehicle_twizy_cfg_readprofile(UINT8 key)
{
  if (key >= 1 && key <= 3) {
    // read custom cfg from params:
    //par_getbin(PARAM_PROFILE_S + ((key-1)<<1), &twizy_cfg_profile, sizeof twizy_cfg_profile);
    EEPROM.get(key * 64, twizy_cfg_profile);

    // check consistency:
    if (twizy_cfg_profile.checksum == vehicle_twizy_cfg_calc_checksum((BYTE *)&twizy_cfg_profile))
      return TRUE;
    else {
      // init to defaults:
      memset(&twizy_cfg_profile, 0, sizeof(twizy_cfg_profile));
      return FALSE;
    }
  }
  else {
    // no custom cfg: load defaults
    memset(&twizy_cfg_profile, 0, sizeof(twizy_cfg_profile));
    return TRUE;
  }
}


// vehicle_twizy_cfg_writeprofile: write from twizy_cgf_profile to params
//    with checksum calculation
BOOL vehicle_twizy_cfg_writeprofile(UINT8 key)
{
  if (key >= 1 && key <= 3) {
    twizy_cfg_profile.checksum = vehicle_twizy_cfg_calc_checksum((BYTE *)&twizy_cfg_profile);
    //par_setbin(PARAM_PROFILE_S + ((key-1)<<1), &twizy_cfg_profile, sizeof twizy_cfg_profile);
    EEPROM.put(key * 64, twizy_cfg_profile);
    return TRUE;
  }
  else {
    return FALSE;
  }
}


// vehicle_twizy_cfg_applyprofile: configure current profile
//    return value: 0 = no error, else error code
//    sets: twizy_cfg.profile_cfgmode, twizy_cfg.profile_user
UINT vehicle_twizy_cfg_applyprofile(UINT8 key)
{
  UINT err;
  int pval;

  // clear success flag:
  twizy_cfg.applied = 0;
  
  // login:
  
  if (err = login(1))
    return err;
  
  // update op (user) mode params:

  if (err = vehicle_twizy_cfg_drive(cfgparam(drive),cfgparam(autodrive_ref),cfgparam(autodrive_minprc)))
    return err;

  if (err = vehicle_twizy_cfg_recup(cfgparam(neutral),cfgparam(brake),cfgparam(autorecup_ref),cfgparam(autorecup_minprc)))
    return err;

  if (err = vehicle_twizy_cfg_ramps(cfgparam(ramp_start),cfgparam(ramp_accel),cfgparam(ramp_decel),cfgparam(ramp_neutral),cfgparam(ramp_brake)))
    return err;

  if (err = vehicle_twizy_cfg_rampl(cfgparam(ramplimit_accel),cfgparam(ramplimit_decel)))
    return err;

  if (err = vehicle_twizy_cfg_smoothing(cfgparam(smooth)))
    return err;

  // update user profile status:
  twizy_cfg.profile_user = key;

  
  // update pre-op (admin) mode params if configmode possible at the moment:

  err = configmode(1);
  
  if (!err) {

    err = vehicle_twizy_cfg_speed(cfgparam(speed),cfgparam(warn));
    if (!err)
      err = vehicle_twizy_cfg_power(cfgparam(torque),cfgparam(power_low),cfgparam(power_high),cfgparam(current));
    if (!err)
      err = vehicle_twizy_cfg_makepowermap();

    if (!err)
      err = vehicle_twizy_cfg_tsmap('D',
              cfgparam(tsmap[0].prc1),cfgparam(tsmap[0].prc2),cfgparam(tsmap[0].prc3),cfgparam(tsmap[0].prc4),
              cfgparam(tsmap[0].spd1),cfgparam(tsmap[0].spd2),cfgparam(tsmap[0].spd3),cfgparam(tsmap[0].spd4));
    if (!err)
      err = vehicle_twizy_cfg_tsmap('N',
              cfgparam(tsmap[1].prc1),cfgparam(tsmap[1].prc2),cfgparam(tsmap[1].prc3),cfgparam(tsmap[1].prc4),
              cfgparam(tsmap[1].spd1),cfgparam(tsmap[1].spd2),cfgparam(tsmap[1].spd3),cfgparam(tsmap[1].spd4));
    if (!err)
      err = vehicle_twizy_cfg_tsmap('B',
              cfgparam(tsmap[2].prc1),cfgparam(tsmap[2].prc2),cfgparam(tsmap[2].prc3),cfgparam(tsmap[2].prc4),
              cfgparam(tsmap[2].spd1),cfgparam(tsmap[2].spd2),cfgparam(tsmap[2].spd3),cfgparam(tsmap[2].spd4));

#ifdef OVMS_TWIZY_CFG_BRAKELIGHT
    if (!err)
      err = vehicle_twizy_cfg_brakelight(cfgparam(brakelight_on),cfgparam(brakelight_off));
#endif // OVMS_TWIZY_CFG_BRAKELIGHT

    // update cfgmode profile status:
    if (err == 0)
      twizy_cfg.profile_cfgmode = key;

    // switch back to op-mode (5 tries):
    key = 5;
    while (configmode(0) != 0) {
      if (--key == 0)
        break;
      delay5(20); // 100 ms
    }

  } else {
    
    // pre-op mode currently not possible;
    // just set speed limit:
    pval = cfgparam(speed);
    if (pval == -1)
      pval = CFG.DefaultKphMax;

    err = writesdo(0x2920,0x05,scale(CFG.DefaultRpmMax,CFG.DefaultKphMax,pval,0,65535));
    if (!err)
      err = writesdo(0x2920,0x06,scale(CFG.DefaultRpmMax,CFG.DefaultKphMax,pval,0,CFG.DefaultRpmRev));
    
  }
  
  // set success flag:
  twizy_cfg.applied = 1;
  
  return err;
}


// vehicle_twizy_cfg_switchprofile: load and configure a profile
//    return value: 0 = no error, else error code
//    sets: twizy_cfg.profile_cfgmode, twizy_cfg.profile_user
UINT vehicle_twizy_cfg_switchprofile(UINT8 key)
{
  // check key:

  if (key > 3)
    return ERR_Range + 1;

  // load new profile:
  vehicle_twizy_cfg_readprofile(key);
  twizy_cfg.unsaved = 0;

  // apply profile:
  return vehicle_twizy_cfg_applyprofile(key);
}


#ifdef OVMS_TWIZY_DEBUG
// utility: output power map info to string:
char *vehicle_twizy_fmt_powermap(char *s)
{
  // Pt2 = Constant Torque End (max_pwr_lo / max_trq)
  s = stp_i(s, " PwrLo: ", twizy_max_pwr_lo);
  readsdo(0x4611,0x03);
  s = stp_l2f(s, "W / ", (twizy_sdo.data * 1000) / 16, 3);
  readsdo(0x4611,0x04);
  s = stp_l2f(s, "Nm @ ", (twizy_sdo.data * CFG.DefaultKphMax * 10) / CFG.DefaultRpmMax, 1);
  s = stp_rom(s, "kph");

  // Pt8 = max_pwr_hi
  s = stp_i(s, " PwrHi: ", twizy_max_pwr_hi);
  readsdo(0x4611,0x0f);
  s = stp_l2f(s, "W / ", (twizy_sdo.data * 1000) / 16, 3);
  readsdo(0x4611,0x10);
  s = stp_l2f(s, "Nm @ ", (twizy_sdo.data * CFG.DefaultKphMax * 10) / CFG.DefaultRpmMax, 1);
  s = stp_rom(s, "kph");

  return s;
}
#endif // OVMS_TWIZY_DEBUG


// utility: output profile switch result info to string
//    and set new profile nr in PARAM_PROFILE
char *vehicle_twizy_fmt_switchprofileresult(char *s, INT8 profilenr, UINT err)
{
  if (err && (err != ERR_CfgModeFailed)) {
    // failure:
    s = vehicle_twizy_fmt_err(s, err);
  }
  else {
    
    if (profilenr == -1) {
      s = stp_rom(s, "WS"); // working set
      }
    else {
      s = stp_i(s, "#", profilenr);
      //par_set(PARAM_PROFILE, s-1);
      EEPROM.put(PARAM_PROFILE, profilenr);
      }
    
    if (err == ERR_CfgModeFailed) {
      // partial success:
      s = stp_rom(s, " PARTIAL: retry at stop!");
    }
    else {
      // full success:
      s = stp_i(s, " OK: SPEED ", cfgparam(speed));
      s = stp_i(s, " ", cfgparam(warn));
      s = stp_i(s, " POWER ", cfgparam(torque));
      s = stp_i(s, " ", cfgparam(power_low));
      s = stp_i(s, " ", cfgparam(power_high));
      s = stp_i(s, " ", cfgparam(current));
    }

    s = stp_i(s, " DRIVE ", cfgparam(drive));
    s = stp_i(s, " ", cfgparam(autodrive_ref));
    s = stp_i(s, " ", cfgparam(autodrive_minprc));
    s = stp_i(s, " RECUP ", cfgparam(neutral));
    s = stp_i(s, " ", cfgparam(brake));
    s = stp_i(s, " ", cfgparam(autorecup_ref));
    s = stp_i(s, " ", cfgparam(autorecup_minprc));
  }

  return s;
}


void vehicle_twizy_init() {
  INT8 i;

  twizy_max_rpm = 0;
  twizy_max_trq = 0;
  twizy_max_pwr_lo = 0;
  twizy_max_pwr_hi = 0;

  twizy_cfg.type = 0;
  
  // reload last selected user profile on init:
  
  EEPROM.get(PARAM_PROFILE, i);
  twizy_cfg.profile_user = constrain(i, 0, 3);
  twizy_cfg.profile_cfgmode = twizy_cfg.profile_user;
  vehicle_twizy_cfg_readprofile(twizy_cfg.profile_user);
  
  twizy_cfg.unsaved = 0;
  twizy_cfg.keystate = 0;

  // Init autopower system:
  twizy_autorecup_level = twizy_autodrive_level = 1000;
  twizy_autorecup_checkpoint = twizy_autodrive_checkpoint = 0;
  
}

