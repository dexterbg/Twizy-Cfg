/**
 * ==========================================================================
 * Twizy/SEVCON configuration shell
 * ==========================================================================
 * 
 * Based on the OVMS Twizy firmware:
 * https://github.com/openvehicles/Open-Vehicle-Monitoring-System
 * 
 * Author: Michael Balzer <dexter@dexters-web.de>
 * 
 * Libraries used:
 *  - MCP_CAN: https://github.com/coryjfowler/MCP_CAN_lib
 * 
 * License:
 *  This is free software under GNU Lesser General Public License (LGPL)
 *  https://www.gnu.org/licenses/lgpl.html
 *  
 */
#define TWIZY_CFG_VERSION "V1.0 (2017-06-13)"

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include "utils.h"
#include "CANopen.h"
#include "TwizyCfg_config.h"


// CAN interface:
MCP_CAN CAN(TWIZY_CAN_CS_PIN);

// Output buffers:
char net_scratchpad[200];
char net_msg_scratchpad[200];


// --------------------------------------------------------------------
// COMMAND DISPATCHER:
//

bool exec(char *cmd)
{
  UINT err;
  char *s;
  char *t;
  bool go_op_onexit = true;
  char *arguments;
  
  int arg[5] = {-1,-1,-1,-1,-1};
  INT8 arg2[4] = {-1,-1,-1,-1};
  long data;
  char maps[4] = {'D','N','B',0};
  UINT8 i;

  arguments = net_sms_initargs(cmd);

  // convert cmd to upper-case:
  for (s=cmd; ((*s!=0)&&(*s!=' ')); s++)
    if ((*s > 0x60) && (*s < 0x7b)) *s=*s-0x20;

  
  if (*cmd == '?' || starts_with(cmd, "HELP")) {
    Serial.print(F("\n"
      "Twizy-Cfg " TWIZY_CFG_VERSION "\n"
      "\n"
      "Commands:\n"
      " ?, help                  -- output this info\n"
      " r <id> <sub>             -- read SDO register (numerical)\n"
      " rs <id> <sub>            -- read SDO register (string)\n"
      " w <id> <sub> <val>       -- write SDO register (numerical) & show old value\n"
      " wo <id> <sub> <val>      -- write-only SDO register (numerical)\n"
      " p                        -- preop mode\n"
      " o                        -- op mode\n"
      "\n"
      "Note: <id> and <sub> are hexadecimal, <val> are decimal\n"
      "Examples:\n"
      " rs 1008 0                -- read SEVCON firmware name\n"
      " w 2920 3 325             -- set neutral recup level to 32.5%\n"
      "\n"
      ));
    
    return false;
  }

  else {
    
    // common reply intro:
    s = stp_ram(net_scratchpad, cmd);
    s = stp_rom(s, ": ");
    
    //
    // COMMAND DISPATCHER:
    //  Part 2: online commands (SEVCON access necessary)
    //  - PRE
    //  - OP
    //  - READ[S]
    //  - WRITE[O]
    //
    
    // login:
    if (err = login(1)) {
      s = vehicle_twizy_fmt_err(s, err);
    }
    

    else if (starts_with(cmd, "P")) {
      // P: enter config mode
      if (err = configmode(1))
        s = vehicle_twizy_fmt_err(s, err);
      else
        s = stp_rom(s, "OK");
      go_op_onexit = false;
    }
    

    else if (starts_with(cmd, "O")) {
      // O: leave config mode
      if (err = configmode(0))
        s = vehicle_twizy_fmt_err(s, err);
      else
        s = stp_rom(s, "OK");
      go_op_onexit = false;
    }
    

    else if (starts_with(cmd, "R")) {
      // R index_hex subindex_hex
      // RS index_hex subindex_hex
      if (arguments = net_sms_nextarg(arguments))
        arg[0] = (int)axtoul(arguments);
      if (arguments = net_sms_nextarg(arguments))
        arg[1] = (int)axtoul(arguments);

      if (!arguments) {
        s = stp_rom(s, "ERROR: Too few args");
      }
      else {
        if (cmd[1] != 'S') {
          // READ:
          if (err = readsdo(arg[0], arg[1])) {
            s = vehicle_twizy_fmt_err(s, err);
          }
          else {
            s = vehicle_twizy_fmt_sdo(s);
            s = stp_ul(s, " = ", twizy_sdo.data);
          }
        }
        else {
          // READS: SMS intro 'CFG READS: 0x1234.56=' = 21 chars, 139 remaining
          if (err = readsdo_buf(arg[0], arg[1], (byte*)net_msg_scratchpad, (i=139, &i))) {
            s = vehicle_twizy_fmt_err(s, err);
          }
          else {
            net_msg_scratchpad[139-i] = 0;
            s = stp_x(s, "0x", arg[0]);
            s = stp_sx(s, ".", arg[1]);
            s = stp_s(s, "=", net_msg_scratchpad);
          }
        }
      }

      go_op_onexit = false;
    }
    

    else if (starts_with(cmd, "W")) {
      // W index_hex subindex_hex data_dec
      // WO index_hex subindex_hex data_dec
      if (arguments = net_sms_nextarg(arguments))
        arg[0] = (int)axtoul(arguments);
      if (arguments = net_sms_nextarg(arguments))
        arg[1] = (int)axtoul(arguments);
      if (arguments = net_sms_nextarg(arguments))
        data = atol(arguments);

      if (!arguments) {
        s = stp_rom(s, "ERROR: Too few args");
      }
      else {

        if (cmd[1] == 'O') {
          // WRITEONLY:

          // write new value:
          if (err = writesdo(arg[0], arg[1], data)) {
            s = vehicle_twizy_fmt_err(s, err);
          }
          else {
            s = stp_rom(s, "OK: ");
            s = vehicle_twizy_fmt_sdo(s);
          }
        }

        else {
          // READ-WRITE:

          // read old value:
          if (err = readsdo(arg[0], arg[1])) {
            s = vehicle_twizy_fmt_err(s, err);
          }
          else {
            // read ok:
            s = stp_rom(s, "OLD:");
            s = vehicle_twizy_fmt_sdo(s);
            s = stp_ul(s, " = ", twizy_sdo.data);

            // write new value:
            if (err = writesdo(arg[0], arg[1], data))
              s = vehicle_twizy_fmt_err(s, err);
            else
              s = stp_ul(s, " => NEW: ", data);
          }
        }
      }

      go_op_onexit = false;
    }

  
    else {
      // unknown command
      s = stp_rom(s, "Unknown command");
    }
    
    // go operational?
    if (go_op_onexit)
      configmode(0);
    
  }


  // 
  // FINISH: send command response
  //
  
  Serial.println(net_scratchpad);

  return true;
}


// --------------------------------------------------------------------
// MAIN
//

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup() {
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  //
  // Init Twizy CAN interface
  //
  
  while (CAN.begin(MCP_STDEXT, CAN_500KBPS, TWIZY_CAN_MCP_FREQ) != CAN_OK) {
    Serial.println(F("setup: waiting for CAN connection..."));
    delay(1000);
  }
  
  // Set filters:
  
  CAN.init_Mask(0, 0, 0x07FF0000);
  CAN.init_Filt(0, 0, 0x05810000); // CANopen response node 1
  CAN.init_Filt(1, 0, 0x00000000);
  
  CAN.init_Mask(1, 0, 0x07FF0000);
  CAN.init_Filt(2, 0, 0x00000000);
  CAN.init_Filt(3, 0, 0x00000000);
  CAN.init_Filt(4, 0, 0x00000000);
  CAN.init_Filt(5, 0, 0x00000000);
  
  CAN.setMode(MCP_NORMAL);

  
  // Output info & prompt:
  exec((char *) "?");
  Serial.print("\n> ");
}


void loop() {
  
  if (stringComplete) {
    
    // execute command:
    Serial.println(inputString);
    exec((char *) inputString.c_str());
    Serial.print("\n> ");
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  
}


/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    }
    else if (inChar >= 32) {
      inputString += inChar;
    }
  }
  
}


