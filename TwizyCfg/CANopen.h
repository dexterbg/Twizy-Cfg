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

#ifndef _CANopen_h
#define _CANopen_h


// SDO request/response:

// message structure:
//    0 = control byte
//    1+2 = index (little endian)
//    3 = subindex
//    4..7 = data (little endian)

union __attribute__ ((__packed__)) sdo_buffer {

  // raw / segment data:
  UINT8   byte[8];

  // request / expedited:
  struct __attribute__ ((__packed__)) {
    UINT8   control;
    UINT    index;
    UINT8   subindex;
    UINT32  data;
  };

};

extern sdo_buffer twizy_sdo;


// SDO communication codes:
#define SDO_CommandMask             0b11100000
#define SDO_ExpeditedUnusedMask     0b00001100
#define SDO_Expedited               0b00000010
#define SDO_Abort                   0b10000000

#define SDO_Abort_SegMismatch       0x05030000
#define SDO_Abort_Timeout           0x05040000
#define SDO_Abort_OutOfMemory       0x05040005


#define SDO_InitUploadRequest       0b01000000
#define SDO_InitUploadResponse      0b01000000

#define SDO_UploadSegmentRequest    0b01100000
#define SDO_UploadSegmentResponse   0b00000000

#define SDO_InitDownloadRequest     0b00100000
#define SDO_InitDownloadResponse    0b01100000

#define SDO_SegmentToggle           0b00010000
#define SDO_SegmentUnusedMask       0b00001110
#define SDO_SegmentEnd              0b00000001


#define CAN_GeneralError            0x08000000

// I/O level CAN error codes / classes:

#define ERR_NoCANWrite              0x0001
#define ERR_Timeout                 0x000a
#define ERR_ComponentOffline        0x000b

#define ERR_ReadSDO                 0x0002
#define ERR_ReadSDO_Timeout         0x0003
#define ERR_ReadSDO_SegXfer         0x0004
#define ERR_ReadSDO_SegMismatch     0x0005

#define ERR_WriteSDO                0x0008
#define ERR_WriteSDO_Timeout        0x0009


// App level CAN error codes / classes:

#define ERR_LoginFailed             0x0010
#define ERR_CfgModeFailed           0x0020
#define ERR_Range                   0x0030
#define ERR_UnknownHardware         0x0040


#endif // _CANopen_h

