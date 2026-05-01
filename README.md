# BYD Battery Status Checker (byd_bat)

A lightweight C++ application for reading data from BYD Battery systems via Modbus RTU over TCP.

## Overview

This application reads battery status, configuration, and monitoring data from BYD Battery Management Systems (BMS) using the Modbus RTU protocol over TCP/IP. It supports both HV (High Voltage) and LV (Low Voltage) battery systems including HVS, HVM, HVL, LVS, LVL, and LVFlex models.

**Key Features:**
- Single execution mode - reads all data once and exits
- Comprehensive system information display
- Optional detailed per-module analysis with `-v` flag
- Individual cell voltage and temperature monitoring
- Fast and lightweight

## Requirements

- libmodbus (3.1.x or later)
- CMake 3.16+
- C++17 compatible compiler (GCC 7+, Clang 5+)
- Network access to BYD Battery system

## Building

```bash
mkdir build
cd build
cmake ..
make
```

The executable `byd_bat` will be created in the build directory.

## Usage

```bash
# Basic usage (summary only)
./byd_bat <ip_address> [port]

# With verbose mode (includes per-module details)
./byd_bat -v <ip_address> [port]

# Display help
./byd_bat --help
```

### Examples

```bash
# Connect to battery at default port (8080)
./byd_bat 192.168.1.233

# Connect to battery at custom port
./byd_bat 192.168.1.233 8080

# Verbose mode - includes detailed per-module information
./byd_bat -v 192.168.1.233

# Full syntax with all options
./byd_bat --verbose 192.168.1.233 8080
```

### Command Line Options

| Option | Description |
|--------|-------------|
| `-v, --verbose` | Display detailed per-module information including individual cell voltages and temperatures |
| `-h, --help` | Display help message |

### Output Sections

**Without `-v` (Default):**
- System Information (serial, firmware versions, configuration)
- Current Status (SOC, SOH, voltage, current, power)
- Cell Information (min/max cell voltages)
- Temperature Information (min/max temperatures)
- Cycle Information (charge/discharge cycles, lifetime energy)
- Error and Warning Status
- Additional Information (versions, date/time)

**With `-v` (Verbose Mode):**
- All of the above, plus:
- Detailed per-module information for each BMS module
- Individual cell voltages (16 cells per module)
- Individual temperature sensors (up to 8 per module)
- Per-module balancing status
- Per-module warnings and errors
- Module lifetime energy statistics

## Modbus Register Map

### System Information Registers (0x0000 - 0x0065)

| Address | Size | Name | Description | Format |
|---------|------|------|-------------|--------|
| 0x0000-0x0008 | 9 registers (18 bytes) | BMU Serial Number | Battery Management Unit serial number | ASCII string |
| 0x000C | 1 register | BMU A Version | BMU firmware version (Area A) | High byte: major, Low byte: minor |
| 0x000D | 1 register | BMU B Version | BMU firmware version (Area B) | High byte: major, Low byte: minor |
| 0x000E | 1 register | BMS Version | BMS firmware version | High byte: major, Low byte: minor |
| 0x000F | 1 register | Working Area | Current working firmware area | High byte: BMU (1=A, 2=B), Low byte: BMS (1=A, 2=B) |
| 0x0010 | 1 register | System Configuration | Module count, BMS count, Inverter type | Bits 0-3: Module count, Bits 4-7: BMS count, Bits 8-11: Inverter type |
| 0x0011 | 1 register | Application & Battery Type | System application and battery model | High byte: Application (0=Off Grid, 1=On Grid, 2=Backup), Low byte: Battery type |
| 0x0012 | 1 register | Phase Configuration | Single or three-phase system | High byte: Phase (0=Single, 1=Three) |
| 0x004B | 1 register | Modbus Address | Configured Modbus slave address | uint16 |
| 0x004C | 1 register | BMU MCU Type | BMU microcontroller type identifier | uint16 |
| 0x004D | 1 register | BMS MCU Type | BMS microcontroller type identifier | uint16 |
| 0x0063 | 1 register | Date (Year/Month) | Current date | High byte: Year-2000, Low byte: Month (1-12) |
| 0x0064 | 1 register | Date (Day/Hour) | Current date | High byte: Day (1-31), Low byte: Hour (0-23) |
| 0x0065 | 1 register | Time (Minute/Second) | Current time | High byte: Minute (0-59), Low byte: Second (0-59) |

### Real-time Status Registers (0x0500 - 0x0518)

| Address | Size | Name | Description | Format | Unit |
|---------|------|------|-------------|--------|------|
| 0x0500 | 1 register | State of Charge (SOC) | Battery charge percentage | uint16 | % |
| 0x0501 | 1 register | Maximum Cell Voltage | Highest voltage among all cells | uint16 × 0.01 | V |
| 0x0502 | 1 register | Minimum Cell Voltage | Lowest voltage among all cells | uint16 × 0.01 | V |
| 0x0503 | 1 register | State of Health (SOH) | Battery health percentage | uint16 | % |
| 0x0504 | 1 register | Current | Battery current (charging/discharging) | int16 × 0.1 | A |
| 0x0505 | 1 register | Battery Voltage | Total battery pack voltage | uint16 × 0.01 | V |
| 0x0506 | 1 register | Maximum Cell Temperature | Highest temperature among all cells | uint16 | °C |
| 0x0507 | 1 register | Minimum Cell Temperature | Lowest temperature among all cells | uint16 | °C |
| 0x0508 | 1 register | BMU Temperature | Battery Management Unit temperature | uint16 | °C |
| 0x050A | 1 register | BMU Version | BMU firmware version | High byte: major, Low byte: minor |
| 0x050D | 1 register | Error Bitmask | System error flags (see Error Codes) | uint16 bitmask |
| 0x050E | 1 register | P/T Version | Parameter table version | High byte: major, Low byte: minor |
| 0x0510 | 1 register | Output Voltage | Battery output/terminal voltage | uint16 × 0.01 | V |
| 0x0511-0x0512 | 2 registers | System Lifetime Charge Energy | Total energy charged (UINT32 little-endian) | × 0.001 | kWh |
| 0x0513-0x0514 | 2 registers | System Lifetime Discharge Energy | Total energy discharged (UINT32 little-endian) | × 0.001 | kWh |

### BMS Detailed Information (Command-Response Protocol)

**Required for verbose mode (`-v` option)**

To read detailed BMS information, use a command-response protocol:

1. **Write Command**: Write to register `0x0550` with value `[BMS_INDEX, 0x8100]`
   - BMS_INDEX: 0x0001 for BMS#1, 0x0002 for BMS#2, etc.
   - Second value: Always 0x8100 (command code)

2. **Poll Status**: Read register `0x0551` until it returns `0x8801` (ready status)

3. **Read Data**: Read 4 blocks of data from `0x0558` (65 registers each time)
   - First register in each block is length (always 128 bytes)
   - Subsequent 64 registers contain actual data

#### BMS Data Block Format (260 registers total after reading 4 chunks)

| Offset | Name | Description | Format | Unit |
|--------|------|-------------|--------|------|
| 0 | Payload Size | Data block size | uint16 | bytes |
| 1 | Cell Voltage Max | Maximum cell voltage | int16 | mV |
| 2 | Cell Voltage Min | Minimum cell voltage | int16 | mV |
| 3 | Cell V-Max/V-Min Index | Cell numbers | High byte: Max cell#, Low byte: Min cell# |
| 4 | Cell Temperature Max | Maximum cell temperature | int16 | °C |
| 5 | Cell Temperature Min | Minimum cell temperature | int16 | °C |
| 6 | Cell T-Max/T-Min Index | Cell numbers | High byte: Max cell#, Low byte: Min cell# |
| 7 | Balancing Flags | Active balancing bitmask | uint16 | bitmask |
| 15-16 | Charge Energy | Lifetime charge energy (UINT32 LE) | × 0.001 | kWh |
| 17-18 | Discharge Energy | Lifetime discharge energy (UINT32 LE) | × 0.001 | kWh |
| 21 | Battery Voltage | Pack voltage | int16 × 0.1 | V |
| 24 | Output Voltage | Terminal voltage | int16 × 0.1 | V |
| 25 | SOC | State of charge | int16 × 0.1 | % |
| 26 | SOH | State of health | int16 | % |
| 27 | Current | Battery current | int16 × 0.1 | A |
| 28 | Warning 1 | Warning flags set 1 | uint16 | bitmask |
| 29 | Warning 2 | Warning flags set 2 | uint16 | bitmask |
| 34-45 | Module Serial | Module serial number | 12 regs | ASCII |
| 48 | Errors | System fault flags | uint16 | bitmask |
| 49-64 | Cell Voltages | Individual cell voltages (16 cells) | int16 | mV |
| 180-183 | Cell Temperatures | Temperature sensors (8 sensors, 2 per reg) | int8 | °C |

## Error, Warning, and Event Codes

### Error Bitmask (Register 0x050D, BMS Data Offset 48)

| Bit | Error Description |
|-----|-------------------|
| 0 | Cells Voltage Sensor Failure |
| 1 | Temperature Sensor Failure |
| 2 | BIC Communication Failure |
| 3 | Pack Voltage Sensor Failure |
| 4 | Current Sensor Failure |
| 5 | Charging MOS Failure |
| 6 | Discharging MOS Failure |
| 7 | Precharging MOS Failure |
| 8 | Main Relay Failure |
| 9 | Precharging Failed |
| 10 | Heating Device Failure |
| 11 | Radiator Failure |
| 12 | BIC Balance Failure |
| 13 | Cells Failure |
| 14 | PCB Temperature Sensor Failure |
| 15 | Functional Safety Failure |

### Warning Bitmask 1 & 2 (BMS Data Offsets 28, 29)

| Bit | Warning Description |
|-----|---------------------|
| 0 | Battery Over Voltage |
| 1 | Battery Under Voltage |
| 2 | Cells OverVoltage |
| 3 | Cells UnderVoltage |
| 4 | Cells Imbalance |
| 5 | Charging High Temperature (Cells) |
| 6 | Charging Low Temperature (Cells) |
| 7 | Discharging High Temperature (Cells) |
| 8 | Discharging Low Temperature (Cells) |
| 9 | Charging OverCurrent (Cells) |
| 10 | Discharging OverCurrent (Cells) |
| 11 | Charging OverCurrent (Hardware) |
| 12 | Short Circuit |
| 13 | Inversely Connection |
| 14 | Interlock Switch Abnormal |
| 15 | Air Switch Abnormal |

## Supported Inverters

The system can be configured with the following inverter types (Register 0x0010, bits 8-11):

0. Fronius HV
1. Goodwe HV/Viessmann HV
2. Goodwe LV/Viessmann LV
3. KOSTAL HV
4. Selectronic LV
5. SMA SBS3.7/5.0/6.0 HV
6. SMA LV
7. Victron LV
8. SUNTECH LV
9. Sungrow HV
10. KACO_HV
11. Studer LV
12. SolarEdge LV
13. Ingeteam HV
14. Sungrow LV
15. Schneider LV
16. SMA SBS2.5 HV
17. Solis LV
18. Solis HV
19. SMA STP 5.0-10.0 SE HV
20. Deye LV
21. Phocos LV
22. GE HV
23. Deye HV
24. Raion LV
25. KACO_NH
26. Solplanet
27. Western HV
28. SOSEN
29. Hoymiles LV
30. Hoymiles HV
31. SAJ HV

## Supported Battery Types

Based on the BMU serial number prefix:

- **P03 or E0P3**: P3 (HV modules in series)
  - Type 0: HVL
  - Type 1: HVM
  - Type 2: HVS

- **P02 or P011**: P2 (LV modules in parallel)
  - Type 0: LVL
  - Type 1: LVFlex (Lite)
  - Type 2: LVS/LVS Lite

## Protocol Details

- **Connection**: Modbus RTU over TCP/IP
- **Default Port**: 8080
- **Slave ID**: 1
- **Function Code**: 0x03 (Read Holding Registers), 0x10 (Write Multiple Registers)
- **Byte Order**: Big-endian (network byte order)
- **Framing**: RTU (with CRC-16)

## Technical Notes

### Why RTU over TCP?

BYD Battery systems use Modbus RTU protocol encapsulated over TCP/IP, which is different from standard Modbus TCP. This application:

1. Creates a Modbus RTU context
2. Establishes a TCP connection to the battery
3. Binds the TCP socket to the RTU context
4. Communicates using RTU framing (with CRC) over the TCP connection

This is compatible with the BYD protocol that uses `ModbusRtuFramer` (as seen in the Python pymodbus implementation).

### Performance

- Summary mode: ~2-3 seconds (reads ~115 registers)
- Verbose mode: ~10-15 seconds per module (includes polling and multiple chunk reads)
- Total time for verbose mode = base time + (module count × 10-15 seconds)

## Troubleshooting

### Connection Issues

If you cannot connect to the battery:

1. Verify IP address is correct
2. Ensure battery system is powered on
3. Check network connectivity (`ping <ip_address>`)
4. Verify port 8080 is accessible (check firewall)
5. Ensure no other application is querying the battery simultaneously

### Incomplete Data

If module queries fail or return zeros:

1. The BMS may be busy - try again
2. Some modules may not respond if not installed/connected
3. Check for errors in the system status

### Build Issues

If compilation fails:

```bash
# Ensure libmodbus is installed
pkg-config --modversion libmodbus

# On Debian/Ubuntu
sudo apt-get install libmodbus-dev

# On Fedora/RHEL
sudo dnf install libmodbus-devel

# On Gentoo
sudo emerge dev-libs/libmodbus
```

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

**Author:** Fabien Proriol <condo4@kazoe.org>
**Copyright:** © 2026 Fabien Proriol

## Credits

Based on reverse-engineering of the BYD Battery Modbus protocol. Protocol information derived from:
- BYD Battery official documentation
- Community reverse engineering efforts
- Analysis of existing implementations (Python, JavaScript)

## Version History

- **1.0** - Initial release
  - Basic system status reading
  - Detailed per-module information with `-v` flag
  - Single execution mode
