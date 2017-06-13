/**
 * ==========================================================================
 * Twizy/SEVCON configuration shell
 * ==========================================================================
 * 
 * Utilities
 * 
 * Based on the OVMS:
 * https://github.com/openvehicles/Open-Vehicle-Monitoring-System
 * 
 * License:
 *  This is free software.
 *  This is a modified copy of the OVMS general utils, so the OVMS license applies:
 *  https://github.com/openvehicles/Open-Vehicle-Monitoring-System/blob/master/LICENSE
 *  
 */

#ifndef _utils_h
#define _utils_h


// Type shortcuts:

typedef unsigned char UINT8;
typedef unsigned int UINT;
typedef unsigned long UINT32;

typedef signed char INT8;


// Useful macros:

#define SQR(n) ((n)*(n))
#define ABS(n) (((n) < 0) ? -(n) : (n))
#define MIN(n,m) ((n) < (m) ? (n) : (m))
#define MAX(n,m) ((n) > (m) ? (n) : (m))
#define LIMIT_MIN(n,lim) ((n) < (lim) ? (lim) : (n))
#define LIMIT_MAX(n,lim) ((n) > (lim) ? (lim) : (n))


#endif // _utils_h

