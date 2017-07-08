/**
 * ==========================================================================
 * Twizy/SEVCON configuration shell
 * ==========================================================================
 */
#ifndef _TwizyCfg_config_h
#define _TwizyCfg_config_h

// Debug output level:
//  1 = show every SDO write
#define TWIZY_DEBUG 0

// Set your CAN MCP clock frequency here:
#define TWIZY_CAN_MCP_FREQ        MCP_16MHZ

// Set your CAN CS pin number here:
#define TWIZY_CAN_CS_PIN          53

// Set your CAN IRQ pin here (0 = no IRQ):
#define TWIZY_CAN_IRQ_PIN         0

#endif // _TwizyCfg_config_h

