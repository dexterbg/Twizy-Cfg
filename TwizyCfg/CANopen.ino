/**
 * ==========================================================================
 * Twizy/SEVCON configuration shell
 * ==========================================================================
 * 
 * Twizy/SEVCON CANopen SDO client
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

#include "CANopen.h"

// SDO TX/RX buffer:
sdo_buffer twizy_sdo;

// CAN RX buffer:
unsigned long rxId;
byte rxLen;
byte rxBuf[8];


// CAN TX utility:

bool sendMsg(INT32U id, INT8U len, INT8U *buf) {
  
  for (int tries=3; tries>0; tries--) {
    if (CAN.sendMsgBuf(id, 0, len, buf) != CAN_GETTXBFTIMEOUT) {
      // CAN_OK = frame has been sent
      // CAN_SENDMSGTIMEOUT = we made it into a send buffer
      //  → no need to repeat the send:
      return true;
    }
  }
  
  return false;
}


// Check received CAN frames for CANopen reply from node 1 (ID 0x581):

#define SAVEMSG(dst) for(int i = 0; i<rxLen; i++) { dst[i] = rxBuf[i]; }

bool checkReply() {

  while (CAN.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
    if (rxId == 0x581) {
      SAVEMSG(twizy_sdo.byte);
      return true;
    }
  }

  return false;
}


// asynchronous SDO request:
void vehicle_twizy_sendsdoreq(void)
{
  sendMsg(0x601, 8, twizy_sdo.byte);

  // clear status to detect reply:
  twizy_sdo.control = 0xff;
}


// synchronous SDO request (50 ms timeout, 3 tries):
UINT vehicle_twizy_sendsdoreq_sync(void)
{
  UINT8 control, timeout, tries;
  UINT32 data;

  control = twizy_sdo.control;
  data = twizy_sdo.data;

  tries = 3;
  do {

    // send request:
    twizy_sdo.control = control;
    twizy_sdo.data = data;
    vehicle_twizy_sendsdoreq();

    // wait for reply:
    timeout = 50;
    do {
      if (!checkReply()) {
        delay(1);
      }
    } while (twizy_sdo.control == 0xff && --timeout);

    if (timeout != 0)
      break;

    // timeout, abort request:
    twizy_sdo.control = SDO_Abort;
    twizy_sdo.data = SDO_Abort_Timeout;
    vehicle_twizy_sendsdoreq();

    if (tries > 1)
      delay(10);

  } while (--tries);

  if (timeout == 0)
    return ERR_Timeout;

  return 0; // ok, response in twizy_sdo.*
}


// read from SDO:
UINT readsdo(UINT index, UINT8 subindex)
{
  UINT8 control;

  // request upload:
  twizy_sdo.control = SDO_InitUploadRequest;
  twizy_sdo.index = index;
  twizy_sdo.subindex = subindex;
  twizy_sdo.data = 0;
  if (vehicle_twizy_sendsdoreq_sync() != 0)
    return ERR_ReadSDO_Timeout;

  // check response:
  if ((twizy_sdo.control & SDO_CommandMask) != SDO_InitUploadResponse) {
    // check for CANopen general error:
    if (twizy_sdo.data == CAN_GeneralError && index != 0x5310) {
      // add SEVCON error code:
      control = twizy_sdo.control;
      readsdo(0x5310,0x00);
      twizy_sdo.control = control;
      twizy_sdo.index = index;
      twizy_sdo.subindex = subindex;
      twizy_sdo.data |= CAN_GeneralError;
    }
    return ERR_ReadSDO;
  }

  // if expedited xfer we're done now:
  if (twizy_sdo.control & SDO_Expedited)
    return 0;

  // segmented xfer necessary:
  twizy_sdo.control = SDO_Abort;
  twizy_sdo.data = SDO_Abort_OutOfMemory;
  vehicle_twizy_sendsdoreq();
  return ERR_ReadSDO_SegXfer; // caller needs to use readsdo_buf
}


// read from SDO into buffer (supporting segmented xfer):
UINT readsdo_buf(UINT index, UINT8 subindex, byte *dst, byte *maxlen)
{
  UINT8 n, toggle, dlen;

  // request upload:
  twizy_sdo.control = SDO_InitUploadRequest;
  twizy_sdo.index = index;
  twizy_sdo.subindex = subindex;
  twizy_sdo.data = 0;
  if (vehicle_twizy_sendsdoreq_sync() != 0)
    return ERR_ReadSDO_Timeout;

  // check response:
  if ((twizy_sdo.control & SDO_CommandMask) != SDO_InitUploadResponse) {
    // check for CANopen general error:
    if (twizy_sdo.data == CAN_GeneralError && index != 0x5310) {
      // add SEVCON error code:
      n = twizy_sdo.control;
      readsdo(0x5310,0x00);
      twizy_sdo.control = n;
      twizy_sdo.index = index;
      twizy_sdo.subindex = subindex;
      twizy_sdo.data |= CAN_GeneralError;
    }
    return ERR_ReadSDO;
  }

  // check for expedited xfer:
  if (twizy_sdo.control & SDO_Expedited) {

    // copy twizy_sdo.data to dst:
    dlen = 8 - ((twizy_sdo.control & SDO_ExpeditedUnusedMask) >> 2);
    for (n = 4; n < dlen && (*maxlen) > 0; n++, (*maxlen)--)
      *dst++ = twizy_sdo.byte[n];

    return 0;
  }

  // segmented xfer necessary:

  toggle = 0;

  do {

    // request segment:
    twizy_sdo.control = (SDO_UploadSegmentRequest|toggle);
    twizy_sdo.index = 0;
    twizy_sdo.subindex = 0;
    twizy_sdo.data = 0;
    if (vehicle_twizy_sendsdoreq_sync() != 0)
      return ERR_ReadSDO_Timeout;

    // check response:
    if ((twizy_sdo.control & (SDO_CommandMask|SDO_SegmentToggle)) != (SDO_UploadSegmentResponse|toggle)) {
      // mismatch:
      twizy_sdo.control = SDO_Abort;
      twizy_sdo.data = SDO_Abort_SegMismatch;
      vehicle_twizy_sendsdoreq();
      return ERR_ReadSDO_SegMismatch;
    }

    // ok, copy response data to dst:
    dlen = 8 - ((twizy_sdo.control & SDO_SegmentUnusedMask) >> 1);
    for (n = 1; n < dlen && (*maxlen) > 0; n++, (*maxlen)--)
      *dst++ = twizy_sdo.byte[n];

    // last segment fetched? => success!
    if (twizy_sdo.control & SDO_SegmentEnd)
      return 0;

    // maxlen reached? => abort xfer
    if ((*maxlen) == 0) {
      twizy_sdo.control = SDO_Abort;
      twizy_sdo.data = SDO_Abort_OutOfMemory;
      vehicle_twizy_sendsdoreq();
      return 0; // consider this as success, we read as much as we could
    }

    // toggle toggle bit:
    toggle = toggle ? 0 : SDO_SegmentToggle;

  } while(1);
  // not reached
}


// write to SDO without size indication:
UINT writesdo(UINT index, UINT8 subindex, UINT32 data)
{
  UINT8 control;

  // request download:
  twizy_sdo.control = SDO_InitDownloadRequest | SDO_Expedited; // no size needed, server is smart
  twizy_sdo.index = index;
  twizy_sdo.subindex = subindex;
  twizy_sdo.data = data;
  if (vehicle_twizy_sendsdoreq_sync() != 0)
    return ERR_WriteSDO_Timeout;

  // check response:
  if ((twizy_sdo.control & SDO_CommandMask) != SDO_InitDownloadResponse) {
    // check for CANopen general error:
    if (twizy_sdo.data == CAN_GeneralError) {
      // add SEVCON error code:
      control = twizy_sdo.control;
      readsdo(0x5310,0x00);
      twizy_sdo.control = control;
      twizy_sdo.index = index;
      twizy_sdo.subindex = subindex;
      twizy_sdo.data |= CAN_GeneralError;
    }
    return ERR_WriteSDO;
  }

  // success
  return 0;
}


// SEVCON login/logout (access level 4):
UINT login(bool on)
{
  UINT err;

  // check login level:
  if (err = readsdo(0x5000,1))
    return ERR_LoginFailed + err;

  if (on && twizy_sdo.data != 4) {
    // login:
    writesdo(0x5000,3,0);
    if (err = writesdo(0x5000,2,0x4bdf))
      return ERR_LoginFailed + err;

    // check new level:
    if (err = readsdo(0x5000,1))
      return ERR_LoginFailed + err;
    if (twizy_sdo.data != 4)
      return ERR_LoginFailed;
  }

  else if (!on && twizy_sdo.data != 0) {
    // logout:
    writesdo(0x5000,3,0);
    if (err = writesdo(0x5000,2,0))
      return ERR_LoginFailed + err;

    // check new level:
    if (err = readsdo(0x5000,1))
      return ERR_LoginFailed + err;
    if (twizy_sdo.data != 0)
      return ERR_LoginFailed;
  }

  return 0;
}


// SEVCON state change operational/pre-operational:
UINT configmode(bool on)
{
  UINT err;

  if (!on) {

    // request operational state:
    if (err = writesdo(0x2800,0,0))
      return ERR_CfgModeFailed + err;

    // give controller some time:
    delay(10);

    // check new status:
    if (err = readsdo(0x5110,0))
      return ERR_CfgModeFailed + err;

    if (twizy_sdo.data != 5)
      return ERR_CfgModeFailed;
  }
  
  else {

    // check controller status:
    if (err = readsdo(0x5110,0))
      return ERR_CfgModeFailed + err;

    if (twizy_sdo.data != 127) {

      // request preoperational state:
      if (err = writesdo(0x2800,0,1))
        return ERR_CfgModeFailed + err;

      // give controller some time:
      delay(10);

      // check new status:
      if (err = readsdo(0x5110,0))
        return ERR_CfgModeFailed + err;

      if (twizy_sdo.data != 127) {
        // reset preop state request:
        if (err = writesdo(0x2800,0,0))
          return ERR_CfgModeFailed + err;
        return ERR_CfgModeFailed;
      }
    }
  }
  
  return 0;
}


// utility: output SDO to string:

char *vehicle_twizy_fmt_sdo(char *s)
{
  s = stp_rom(s, " SDO ");
  if ((twizy_sdo.control & 0b11100000) == 0b10000000)
    s = stp_rom(s, "ABORT ");
  s = stp_x(s, "0x", twizy_sdo.index);
  s = stp_sx(s, ".", twizy_sdo.subindex);
  if (twizy_sdo.data > 0x0ffff)
    s = stp_lx(s, ": 0x", twizy_sdo.data);
  else
    s = stp_x(s, ": 0x", twizy_sdo.data);

  switch (twizy_sdo.data) {
    case 0x05040000:
      s = stp_rom(s, ": SEVCON OFFLINE ");
      break;
    case 0x08000004:
      s = stp_rom(s, ": NEEDS PRE-OP MODE ");
      break;
    case 0x08000008:
      s = stp_rom(s, ": ACCESS LEVEL TOO LOW ");
      break;
    case 0x0800000a:
    case 0x0800000b:
    case 0x0800000c:
      s = stp_rom(s, ": INVALID VALUE ");
      break;
  }

  return s;
}


// utility: translate error code to string:

char *vehicle_twizy_fmt_err(char *s, UINT err)
{
  UINT8 detail = err & 0x000f;

  // output error code:
  s = stp_x(s, " ERROR ", err);

  switch (err & 0xfff0) {
    case ERR_Range:
      // User error:
      s = stp_i(s, "INVALID PARAM ", detail);
      break;
    case ERR_CfgModeFailed:
      s = stp_rom(s, " NOT IN STOP");
    // fall through...
    default:
      switch (detail) {
        case ERR_NoCANWrite:
          s = stp_rom(s, " NO CANWRITE");
          break;
        case ERR_ComponentOffline:
          s = stp_rom(s, " SEVCON OFFLINE");
          break;
      }
      s = vehicle_twizy_fmt_sdo(s);
      break;
  }

  return s;
}



