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

#include "utils.h"


// hex string decode:

unsigned long axtoul(char *s)
{
  unsigned long val = 0;
  unsigned char c;
  unsigned int dig = 0;

  while (s && *s) {
    c = *s | 0x20;

    if (c == 'x') {
      val = 0; // prefix 0x => reset val
    }
    else if (c >= '0' && c <= '9') {
      dig = c - '0';
      val = (val << 4) | dig;
    }
    else if (c >= 'a' && c <= 'f') {
      dig = c - 'a' + 10;
      val = (val << 4) | dig;
    }
    else {
      break; // no hex char, stop
    }

    s++;
  }

  return val;
}


// string prefix check:

bool starts_with(char *s, const char *pfx)
{
  while ((*s == *pfx) && (*pfx != 0))
  {
    pfx++;
    s++;
  }
  return (*pfx == 0);
}


// string-print string:

char *stp_rom(char *dst, const char *val)
{
  while (*dst = *val++) dst++;
  return dst;
}

// string-print ram string:

char *stp_ram(char *dst, const char *val)
{
  while (*dst = *val++) dst++;
  return dst;
}

// string-print ram string with optional prefix:

char *stp_s(char *dst, const char *prefix, char *val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  return stp_ram(dst, val);
}

// string-print string with optional prefix:

char *stp_rs(char *dst, const char *prefix, const char *val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  return stp_rom(dst, val);
}

// string-print integer with optional string prefix:

char *stp_i(char *dst, const char *prefix, int val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  itoa(val, dst, 10);
  while (*dst) dst++;
  return dst;
}

// string-print long with optional string prefix:

char *stp_l(char *dst, const char *prefix, long val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  ltoa(val, dst, 10);
  while (*dst) dst++;
  return dst;
}

// string-print unsigned long with optional string prefix:

char *stp_ul(char *dst, const char *prefix, unsigned long val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  ultoa(val, dst, 10);
  while (*dst) dst++;
  return dst;
}

// ltox
//  (note: fills fixed len chars with '0' padding = sprintf %04x)

void ltox(unsigned long i, char *s, unsigned int len)
{
  unsigned char n;

  s += len;
  *s = '\0';

  for (n = len; n != 0; --n)
  {
    *--s = "0123456789ABCDEF"[i & 0x0F];
    i >>= 4;
  }
}

// string-print unsigned integer hexadecimal with optional string prefix:
//  (note: fills fixed 4 chars with '0' padding = sprintf %04x)

char *stp_x(char *dst, const char *prefix, unsigned int val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  ltox(val, dst, 4);
  while (*dst) dst++;
  return dst;
}

// string-print unsigned long hexadecimal with optional string prefix:
//  (note: fills fixed 8 chars with '0' padding = sprintf %08lx)

char *stp_lx(char *dst, const char *prefix, unsigned long val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  ltox(val, dst, 8);
  while (*dst) dst++;
  return dst;
}

// string-print unsigned integer hexadecimal with optional string prefix:
//  (note: fills fixed 2 chars with '0' padding = sprintf %02x)

char *stp_sx(char *dst, const char *prefix, unsigned char val)
{
  if (prefix)
    dst = stp_rom(dst, prefix);
  ltox(val, dst, 2);
  while (*dst) dst++;
  return dst;
}

// string-print unsigned long to fixed size with padding

char *stp_ulp(char *dst, const char *prefix, unsigned long val, int len, char pad)
{
  char buf[11];
  byte bl;

  if (prefix)
    dst = stp_rom(dst, prefix);

  ultoa(val, buf, 10);

  for (bl = strlen(buf); bl < len; bl++)
    *dst++ = pad;

  dst = stp_ram(dst, buf);

  return dst;
}


// string-print fixed precision long as float

char *stp_l2f(char *dst, const char *prefix, long val, int prec)
{
  long factor, lval;
  char p;

  for (factor = 1, p = prec; p > 0; p--)
    factor *= 10;
  lval = val / factor;

  if (prefix)
    dst = stp_rom(dst, prefix);
  if (lval == 0 && val < 0) // handle "negative zero"
    *dst++ = '-';
  dst = stp_l(dst, NULL, lval);
  *dst++ = '.';
  dst = stp_ulp(dst, NULL, ABS(val) % factor, prec, '0');

  return dst;
}


// string-print unsigned long as fixed point number with optional string prefix

#define chDecimal '.'
#define chSeparator ','

char *stp_l2f_h(char *dst, const char *prefix, unsigned long val, int cdecimal)
{
  char *start, *end;
  int cch;

  if (prefix)
    dst = stp_rom(dst, prefix);

  start = dst;
  cch = 0;
  // decompose the number, writing out the digits backwards
  while (val != 0 || cch <= cdecimal)
  {
    // write out the thousands separator when needed
    if (((cch - cdecimal) % 3) == 0 && cch > cdecimal)
      *dst++ = chSeparator;

    // peel off the next digit
    *dst++ = '0' + (val % 10);
    val /= 10;

    // write out the decimal point when needed
    if (++cch == cdecimal)
      *dst++ = chDecimal;
  }
  end = dst - 1;
  // reverse the string in place
  while (start < end)
  {
    char chT = *start;
    *start++ = *end;
    *end-- = chT;
  }
  // null terminate
  *dst = 0;
  return dst;
}


// Text command argument tokenizing:

char *net_sms_argend = NULL;

char* net_sms_initargs(char* arguments)
{
  char *p;

  if (arguments == NULL) return NULL;

  net_sms_argend = arguments + strlen(arguments);
  if (net_sms_argend == arguments) return NULL;

  // Zero-terminate the first argument
  for (p = arguments; (*p != ' ') && (*p != 0); p++) {}
  if (*p == ' ') *p = 0;

  return arguments;
}

char* net_sms_nextarg(char *lastarg)
{
  char *p;

  if (lastarg == NULL) return NULL;

  // find end of previous argument:
  for (p = lastarg; (*p != 0); p++) {}
  if (p == net_sms_argend) return NULL;

  // find start of next argument:
  while (*++p == ' ') {}
  if (p == net_sms_argend) return NULL;
  lastarg = p;
  
  // zero-terminate:
  for (; (*p != ' ') && (*p != 0); p++) {}
  if (*p == ' ') *p = 0;

  return lastarg;
}



