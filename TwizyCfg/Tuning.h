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

#ifndef _Tuning_h
#define _Tuning_h

#define OVMS_TWIZY_CFG_BRAKELIGHT 1

union cfg_status {
  
  UINT8 drivemode;
  
  struct {
    unsigned type:1;                  // CFG: 0=Twizy80, 1=Twizy45
    unsigned profile_user:2;          // CFG: user selected profile: 0=Default, 1..3=Custom
    unsigned profile_cfgmode:2;       // CFG: profile, cfgmode params were last loaded from
    unsigned unsaved:1;               // CFG: RAM profile changed & not yet saved to EEPROM
    unsigned keystate:1;              // CFG: key state change detection
    unsigned applied:1;               // CFG: applyprofile success flag
  };
  
};

extern cfg_status twizy_cfg;


// SEVCON macro configuration profile:
// 1 kept in RAM (working set)
// 3 stored in binary EEPROM param slots PARAM_PROFILE1 /2 /3

struct cfg_profile {
  UINT8       checksum;
  UINT8       speed, warn;
  UINT8       torque, power_low, power_high;
  UINT8       drive, neutral, brake;
  struct tsmap {
    UINT8       spd1, spd2, spd3, spd4;
    UINT8       prc1, prc2, prc3, prc4;
  }           tsmap[3]; // 0=D 1=N 2=B
  UINT8       ramp_start, ramp_accel, ramp_decel, ramp_neutral, ramp_brake;
  UINT8       smooth;
  UINT8       brakelight_on, brakelight_off;
  // V3.2.1 additions:
  UINT8       ramplimit_accel, ramplimit_decel;
  // V3.4.0 additions:
  UINT8       autorecup_minprc;
  // V3.6.0 additions:
  UINT8       autorecup_ref;
  UINT8       autodrive_minprc;
  UINT8       autodrive_ref;
  // V3.7.0 additions:
  UINT8       current;
};

extern cfg_profile twizy_cfg_profile;

// EEPROM memory usage info:
// sizeof twizy_cfg_profile = 24 + 8x3 = 48 byte
// Maximum size = 2 x PARAM_MAX_LENGTH = 64 byte

// Macros for converting profile values:
// shift values by 1 (-1.. => 0..) to map param range to UINT8
#define cfgparam(NAME)  (((int)(twizy_cfg_profile.NAME))-1)
#define cfgvalue(VAL)   ((UINT8)((VAL)+1))


extern unsigned int twizy_max_rpm;         // CFG: max speed (RPM: 0..11000)
extern unsigned long twizy_max_trq;        // CFG: max torque (mNm: 0..70125)
extern unsigned int twizy_max_pwr_lo;      // CFG: max power low speed (W: 0..17000)
extern unsigned int twizy_max_pwr_hi;      // CFG: max power high speed (W: 0..17000)

extern UINT8 twizy_autorecup_checkpoint;   // change detection for autorecup function
extern UINT twizy_autorecup_level;         // autorecup: current recup level (per mille)

extern UINT8 twizy_autodrive_checkpoint;   // change detection for autopower function
extern UINT twizy_autodrive_level;         // autopower: current drive level (per mille)


// EEPROM addresses:
#define PARAM_PROFILE               0x0000 // current cfg profile nr (INT8 0..3)
#define PARAM_PROFILE_S             0x0040 // custom profiles base

#endif

