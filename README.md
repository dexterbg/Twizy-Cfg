# Twizy-Cfg SEVCON configuration shell

This is a minimalistic SEVCON Gen4 configuration shell for Arduino.

It's a port of my SEVCON core functionality from the [OVMS project](https://github.com/openvehicles/Open-Vehicle-Monitoring-System), with a simple command interface on the Arduino serial port.

**V1** supported only reading & writing SDO registers and entering/leaving pre-operational mode. This is now supported both by the V1 shortcut commands as well as the original OVMS command syntax.

**V2** now also supports the OVMS/Twizy tuning macro commands (i.e. `power`, `speed`, `recup` etc.) as well as profile management including saving to / loading from the Arduino EEPROM. Assuming this will be used by hardware hackers, the `brakelight` command has been included as well. Still missing from the OVMS command set is the `clear` command, this is planned to get included along with log access and output. Also missing are all dynamic adjustment functions, i.e. auto drive/recuperation power level following and kickdown.

Read the OVMS user manual and command overview for details on all commands. You may also like to read the SEVCON Gen4 manual for some basic description of registers. The full register set is documented in the SEVCON master dictionary, which is ©SEVCON. It's contained in the SEVCON DVT package.

Most registers of interest for normal tuning can be found in the [Twizy SDO list](extras/Twizy-SDO-List.ods).

**Note**: SEVCON write access has been limited by Renault on Twizys (SEVCONs) delivered after June 2016. The write protection applies to all power and driving profile tuning registers, and also to the battery limits. A locked SEVCON will respond to register writes with an error. You can also check the software version (`rs 100a 00`): if it's `0712.0003` or higher, the Twizy is locked.


## Hardware requirements

1. An Arduino (a Nano will do)
2. An MCP 2515 SPI CAN module, i.e. a "CAN-BUS Shield"
3. An OBD2 plug/cable for the MCP module


## Installation

To download, click the DOWNLOADS button in the top right corner, download the ZIP file. Extract the ZIP, open `TwizyCfg/TwizyCfg.ino` from Arduino IDE.

You will also need this library:
  - [MCP_CAN_lib by Cory Fowler](https://github.com/coryjfowler/MCP_CAN_lib)

Enter your CAN module configuration in the `TwizyCfg_config.h` tab.


## Usage

Connect to the OBD2 port, switch on the Twizy, start the sketch & open the serial monitor. Set the serial monitor line mode to "new line".

The Arduino will display a help screen, then wait for your commands:


### Low level commands

| Function | Command |
| --- | --- |
| Show help | `?` / `help` |
| Enter pre-op mode | `p` |
| Enter op mode | `o` |
| Read numerical register | `r <id> <sub>` |
| Read string register | `rs <id> <sub>` |
| Write register | `w <id> <sub> <val>` |
| Write-only register | `wo <id> <sub> <val>` |

  - Standard OVMS command syntax (i.e. `pre`, `read` etc.) is also accepted
  - `<id>` & `<sub>` define the SDO register to access, need to be given as hexadecimal numbers
  - `<val>` is the value to write, needs to be given as an unsigned decimal number (negative = two's complement, see below)


### Macro commands

| Function | Command |
| --- | --- |
| Set profile from base64 | `set <prf> <b64>` |
| Reset profile | `reset <prf>` |
| Get profile base64 | `get <prf>` |
| Show main profile values | `info` |
| Save config to profile | `save <prf>` |
| Load config from profile | `load <prf>` |
| Set drive level | `drive <prc>` |
| Set recuperation levels neutral & brake | `recup <ntr> <brk>` |
| Set ramp levels | `ramps <st> <ac> <dc> <nt> <br>` |
| Set ramp limits | `rampl <ac> <dc>` |
| Set smoothing | `smooth <prc>` |
| Set max & warn speed | `speed <max> <warn>` |
| Set torque, power & current levels | `power <trq> <pw1> <pw2> <cur>` |
| Set torque speed maps | `tsmap <DNB> <p1@s1> <p2@s2> <p3@s3> <p4@s4>` |
| Set brakelight accel levels | `brakelight <on> <off>` |

  - See [OVMS user manual](https://github.com/openvehicles/Open-Vehicle-Monitoring-System/raw/master/docs/Renault-Twizy/OVMS-UserGuide-RenaultTwizy.pdf)
  - See [OVMS command overview](https://github.com/openvehicles/Open-Vehicle-Monitoring-System/raw/master/docs/Renault-Twizy/Twizy-Command-Overview.pdf)
  - See [Twizy profile editor](https://dexters-web.de/cfgedit)
  - See [Twizy profile converter](https://dexters-web.de/cfgconv)


### Examples

  - Get firmware version: `rs 100a 00` (if it's `0712.0003` or higher, the Twizy is locked)
  - Set vmax to 100 kph: `speed 100`
  - Set torque to 130% and power to 120%: `power 130 120`
  - Set neutral recuperation to 20% and brake recuperation to 30%: `recup 20 30`
  - Set and apply a base64 profile: `set 0 3m9wg295ABozAAAAAAAAAAAuOkVbZVNFNRUrRVtlNSMbJGUAAAABAABlZQAAAAAA` (Twizy needs to be on, not in `GO`)
  - Save current profile to EEPROM slot 1: `save 1`
  - Reset SEVCON to default configuration: `reset`


### Notes

The shell automatically logs into the SEVCON with access level 4 (highest user level, 5 is reserved for SEVCON engineering), so all user SDOs can be written to. See SEVCON master dictionary for access levels. **Note**: all logins are logged in the SEVCON event logs. See SEVCON manual on how to access the logs.

When writing to a register using `w`, for convenience the old value will be read and shown. For write-only registers use `wo` to skip the read operation.

The shell does not know about the data type of the SDO accessed (that's why you need to use `rs` for string SDOs). It handles all numerical registers as being unsigned long. To convert to/from negative values, build the two's complement value by adding the offset:

| Type | Negative offset | -1 = … |
| --- | --- | --- |
| Integer8 | 256 | 255 |
| Integer16 | 65536 | 65535 |
| Integer32 | 4294967296 | 4294967295 |

Most signed tuning registers are of type Integer16. See [Twizy DCF](extras/Twizy-DCF-0712-0002.ods) for register data and access types and default values.

Error messages are mostly self-explanatory, i.e. "SEVCON OFFLINE" means you haven't switched on the Twizy.


## Author

Twizy-Cfg and the OVMS Twizy adaption has been created and is maintained by Michael Balzer (<dexter@dexters-web.de> / https://dexters-web.de/).

Some of the functions contained in the "utils" module have been written by other [OVMS contributors](https://github.com/openvehicles/Open-Vehicle-Monitoring-System/graphs/contributors).


## Donations

**Donations** to support my efforts and further development are very welcome.  
Please send donations via **Paypal** to: `dexter@dexters-web.de`  
**Thanks! :-)**


## License

This is free software; you can redistribute it and/or modify it under the terms of the [GNU Lesser General Public License](https://www.gnu.org/licenses/lgpl.html) as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this software; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110- 1301  USA


**Have fun!**
