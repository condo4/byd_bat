/*
 * BYD Battery Status Checker - Modbus Reader Implementation
 * Copyright (C) 2026 Fabien Proriol <condo4@kazoe.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "modbusreader.h"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <algorithm>
#include <sstream>

// Utility function to trim strings
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\0");
    if (first == std::string::npos)
        return "";
    size_t last = str.find_last_not_of(" \t\n\r\0");
    return str.substr(first, (last - first + 1));
}

// Initialize static constants
const std::vector<std::string> ModbusReader::INVERTER_LIST = {
    "Fronius HV", "Goodwe HV/Viessmann HV", "Goodwe LV/Viessmann LV", "KOSTAL HV",
    "Selectronic LV", "SMA SBS3.7/5.0/6.0 HV", "SMA LV", "Victron LV", "SUNTECH LV",
    "Sungrow HV", "KACO_HV", "Studer LV", "SolarEdge LV", "Ingeteam HV", "Sungrow LV",
    "Schneider LV", "SMA SBS2.5 HV", "Solis LV", "Solis HV", "SMA STP 5.0-10.0 SE HV",
    "Deye LV", "Phocos LV", "GE HV", "Deye HV", "Raion LV", "KACO_NH", "Solplanet",
    "Western HV", "SOSEN", "Hoymiles LV", "Hoymiles HV", "SAJ HV"
};

const std::vector<std::string> ModbusReader::APPLICATION_LIST = {
    "Off Grid", "On Grid", "Backup"
};

const std::vector<std::string> ModbusReader::PHASE_LIST = {
    "Single", "Three"
};

const std::vector<std::string> ModbusReader::ERRORS = {
    "Cells Voltage Sensor Failure", "Temperature Sensor Failure", "BIC Communication Failure",
    "Pack Voltage Sensor Failure", "Current Sensor Failure", "Charging Mos Failure",
    "DisCharging Mos Failure", "PreCharging Mos Failure", "Main Relay Failure",
    "PreCharging Failed", "Heating Device Failure", "Radiator Failure",
    "BIC Balance Failure", "Cells Failure", "PCB Temperature Sensor Failure",
    "Functional Safety Failure"
};

const std::vector<std::string> ModbusReader::WARNINGS = {
    "Battery Over Voltage", "Battery Under Voltage", "Cells OverVoltage", "Cells UnderVoltage",
    "Cells Imbalance", "Charging High Temperature(Cells)", "Charging Low Temperature(Cells)",
    "DisCharging High Temperature(Cells)", "DisCharging Low Temperature(Cells)",
    "Charging OverCurrent(Cells)", "DisCharging OverCurrent(Cells)",
    "Charging OverCurrent(Hardware)", "Short Circuit", "Inversly Connection",
    "Interlock switch Abnormal", "AirSwitch Abnormal"
};

const std::vector<std::string> ModbusReader::WARNINGS3 = {
    "Battery Over Voltage", "Battery Under Voltage", "Cell Over Voltage", "Cell Under Voltage",
    "Voltage Sensor Failure", "Temperature Sensor Failure", "High Temperature Discharging (Cells)",
    "Low Temperature Discharging (Cells)", "High Temperature Charging (Cells)",
    "Low Temperature Charging (Cells)", "Over Current Discharging", "Over Current Charging",
    "Main circuit Failure", "Short Circuit Alarm", "Cells ImBalance", "Current Sensor Failure"
};

ModbusReader::ModbusReader(const std::string &host, int port)
    : m_modbusCtx(nullptr)
    , m_socket(-1)
    , m_host(host)
    , m_port(port)
{
}

ModbusReader::~ModbusReader()
{
    if (m_modbusCtx) {
        modbus_free(m_modbusCtx);
    }
    if (m_socket != -1) {
        close(m_socket);
    }
}

bool ModbusReader::connectToDevice()
{
    // For RTU over TCP: create RTU context then connect via TCP socket
    m_modbusCtx = modbus_new_rtu("/dev/null", 115200, 'N', 8, 1);

    if (!m_modbusCtx) {
        fprintf(stderr, "Failed to create Modbus RTU context\n");
        return false;
    }

    // Set slave ID to 1
    modbus_set_slave(m_modbusCtx, 1);

    // Create TCP socket manually
    struct sockaddr_in addr;
    m_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (m_socket == -1) {
        fprintf(stderr, "Failed to create socket\n");
        modbus_free(m_modbusCtx);
        m_modbusCtx = nullptr;
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(m_port);

    if (inet_pton(AF_INET, m_host.c_str(), &addr.sin_addr) <= 0) {
        fprintf(stderr, "Invalid address\n");
        close(m_socket);
        modbus_free(m_modbusCtx);
        m_modbusCtx = nullptr;
        return false;
    }

    if (::connect(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "Connection failed: %s\n", strerror(errno));
        close(m_socket);
        modbus_free(m_modbusCtx);
        m_modbusCtx = nullptr;
        return false;
    }

    // Set the socket to the modbus context
    modbus_set_socket(m_modbusCtx, m_socket);

    // Set timeout
    modbus_set_response_timeout(m_modbusCtx, 2, 0);
    modbus_set_byte_timeout(m_modbusCtx, 0, 500000);

    printf("Connected to Modbus device at %s:%d (RTU over TCP)\n", m_host.c_str(), m_port);
    return true;
}

std::vector<uint16_t> ModbusReader::readRegs(int start, int regCount)
{
    std::vector<uint16_t> result;

    if (!m_modbusCtx)
        return result;

    uint16_t *tab_reg = new uint16_t[regCount];

    int rc = modbus_read_registers(m_modbusCtx, start, regCount, tab_reg);

    if (rc == regCount) {
        for (int i = 0; i < regCount; ++i) {
            result.push_back(tab_reg[i]);
        }
    } else {
        fprintf(stderr, "Read error at address 0x%04X: %s\n", start, modbus_strerror(errno));
    }

    delete[] tab_reg;
    return result;
}

bool ModbusReader::writeRegs(int start, const std::vector<uint16_t> &values)
{
    if (!m_modbusCtx || values.empty())
        return false;

    uint16_t *tab_reg = new uint16_t[values.size()];
    for (size_t i = 0; i < values.size(); ++i) {
        tab_reg[i] = values[i];
    }

    int rc = modbus_write_registers(m_modbusCtx, start, values.size(), tab_reg);
    delete[] tab_reg;

    if (rc != (int)values.size()) {
        fprintf(stderr, "Write error at address 0x%04X: %s\n", start, modbus_strerror(errno));
        return false;
    }

    return true;
}

std::string ModbusReader::readRegBytes(int start, int byteCount)
{
    std::string buf;
    int regCount = (byteCount + 1) >> 1;
    std::vector<uint16_t> regs = readRegs(start, regCount);

    for (uint16_t r : regs) {
        buf.push_back((r >> 8) & 0xFF);
        buf.push_back(r & 0xFF);
    }

    return buf;
}

std::map<int, uint16_t> ModbusReader::loadRegs(int startReg, int regCount)
{
    std::map<int, uint16_t> regMap;
    std::vector<uint16_t> regValues = readRegs(startReg, regCount);

    for (size_t i = 0; i < regValues.size(); ++i) {
        regMap[startReg + i] = regValues[i];
    }

    return regMap;
}

std::string ModbusReader::bitmaskStr(uint16_t bitm, const std::vector<std::string> &bitmaskList)
{
    std::vector<std::string> warnings;

    for (int bit = 0; bit < 16; ++bit) {
        if (bitm & (1 << bit)) {
            if (bit < (int)bitmaskList.size()) {
                warnings.push_back(bitmaskList[bit]);
            }
        }
    }

    if (warnings.empty())
        return "Normal";

    std::string result;
    for (size_t i = 0; i < warnings.size(); ++i) {
        if (i > 0) result += ";";
        result += warnings[i];
    }
    return result;
}

int16_t ModbusReader::signed16bit(uint16_t val)
{
    if (val >= 0x8000) {
        return val - 0x10000;
    }
    return val;
}

std::string ModbusReader::workingArea(int area)
{
    return (area == 1) ? "A" : "B";
}

BmsModuleData ModbusReader::queryModule(int bmsId)
{
    BmsModuleData data = {};
    data.bmsId = bmsId;

    // Step 1: Write command to query this BMS module
    std::vector<uint16_t> cmd = {(uint16_t)bmsId, 0x8100};
    if (!writeRegs(0x0550, cmd)) {
        fprintf(stderr, "Failed to write query command for BMS %d\n", bmsId);
        return data;
    }

    // Step 2: Poll for ready response (0x8801)
    const int maxPolls = 20;  // 20 * 500ms = 10 seconds
    bool ready = false;

    for (int poll = 0; poll < maxPolls; ++poll) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::vector<uint16_t> status = readRegs(0x0551, 1);
        if (!status.empty() && status[0] == 0x8801) {
            ready = true;
            break;
        }
    }

    if (!ready) {
        fprintf(stderr, "Timeout waiting for BMS %d to be ready\n", bmsId);
        return data;
    }

    // Step 3: Read 4 chunks of 65 registers each
    std::vector<uint16_t> allData;
    const int chunkSize = 65;
    const int numChunks = 4;

    for (int chunk = 0; chunk < numChunks; ++chunk) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::vector<uint16_t> chunkData = readRegs(0x0558, chunkSize);

        if ((int)chunkData.size() == chunkSize) {
            allData.insert(allData.end(), chunkData.begin(), chunkData.end());
        } else {
            // Fill with zeros on failure
            for (int i = 0; i < chunkSize; ++i) {
                allData.push_back(0);
            }
        }
    }

    // Verify we have all data
    if ((int)allData.size() < numChunks * chunkSize) {
        fprintf(stderr, "Incomplete data from BMS %d\n", bmsId);
        return data;
    }

    // Step 4: Parse the data (based on modbus-client.js)
    data.maxCellVoltage = signed16bit(allData[1]);
    data.minCellVoltage = signed16bit(allData[2]);
    data.maxVoltageCell = (allData[3] >> 8) & 0xFF;
    data.maxVoltageModule = allData[3] & 0xFF;
    data.maxTemp = signed16bit(allData[4]);
    data.minTemp = signed16bit(allData[5]);
    data.maxTempCell = (allData[6] >> 8) & 0xFF;
    data.maxTempModule = allData[6] & 0xFF;

    // Balancing flags
    data.balancingFlags = allData[7];
    data.balancingActive = 0;
    for (int bit = 0; bit < 16; ++bit) {
        if (data.balancingFlags & (1 << bit)) {
            data.balancingActive++;
        }
    }

    // Energy (UINT32 little-endian)
    data.chargeEnergyKwh = (allData[16] * 65536.0 + allData[15]) * 0.001;
    data.dischargeEnergyKwh = (allData[18] * 65536.0 + allData[17]) * 0.001;

    data.batteryVoltage = signed16bit(allData[21]) * 0.1;
    data.outputVoltage = signed16bit(allData[24]) * 0.1;
    data.soc = signed16bit(allData[25]) * 0.1;
    data.soh = signed16bit(allData[26]);
    data.current = -(signed16bit(allData[27]) * 0.1);  // Negated

    data.warnings1 = allData[28];
    data.warnings2 = allData[29];
    data.errors = allData[48];

    // Module serial number (registers 34-45, 12 registers = 24 bytes)
    std::string serialBytes;
    for (int i = 34; i < 46 && i < (int)allData.size(); ++i) {
        uint16_t val = allData[i];
        uint8_t ch1 = (val >> 8) & 0xFF;
        uint8_t ch2 = val & 0xFF;
        if (ch1 >= 32 && ch1 < 127) serialBytes.push_back(ch1);
        if (ch2 >= 32 && ch2 < 127) serialBytes.push_back(ch2);
    }
    data.serial = trim(serialBytes);

    // 16 cell voltages (registers 49-64, in mV)
    for (int i = 0; i < 16; ++i) {
        int idx = 49 + i;
        if (idx < (int)allData.size()) {
            data.cellVoltages.push_back(signed16bit(allData[idx]));
        }
    }

    // 8 temperatures (registers 180-183, 2 temps per register)
    for (int i = 0; i < 4; ++i) {
        int idx = 180 + i;
        if (idx < (int)allData.size()) {
            uint16_t val = allData[idx];
            int8_t tHi = (val >> 8) & 0xFF;
            int8_t tLo = val & 0xFF;
            data.cellTemps.push_back(tHi);
            data.cellTemps.push_back(tLo);
        }
    }

    // Remove trailing zeros from temps
    while (!data.cellTemps.empty() && data.cellTemps.back() == 0) {
        data.cellTemps.pop_back();
    }

    return data;
}

void ModbusReader::displayModuleData(const BmsModuleData &data, int moduleNumber)
{
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ MODULE %d DETAILED INFORMATION                                              │\n", moduleNumber);
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    if (!data.serial.empty()) {
        printf("  %-35s : %s\n", "Module Serial Number", data.serial.c_str());
    }

    printf("  %-35s : %.3f V (Cell #%d)\n", "Maximum Cell Voltage",
           data.maxCellVoltage / 1000.0, data.maxVoltageCell);
    printf("  %-35s : %.3f V (Cell #%d)\n", "Minimum Cell Voltage",
           data.minCellVoltage / 1000.0, data.maxVoltageCell);
    printf("  %-35s : %.3f V\n", "Cell Voltage Delta",
           (data.maxCellVoltage - data.minCellVoltage) / 1000.0);

    printf("  %-35s : %d °C (Sensor #%d)\n", "Maximum Temperature",
           data.maxTemp, data.maxTempCell);
    printf("  %-35s : %d °C (Sensor #%d)\n", "Minimum Temperature",
           data.minTemp, data.maxTempModule);

    printf("  %-35s : %.1f %%\n", "State of Charge", data.soc);
    printf("  %-35s : %d %%\n", "State of Health", data.soh);
    printf("  %-35s : %.1f V\n", "Battery Voltage", data.batteryVoltage);
    printf("  %-35s : %.1f V\n", "Output Voltage", data.outputVoltage);
    printf("  %-35s : %.1f A\n", "Current", data.current);

    double power = data.current * data.outputVoltage;
    printf("  %-35s : %.1f W", "Power", power);
    if (power > 0) {
        printf(" (Discharging)\n");
    } else if (power < 0) {
        printf(" (Charging)\n");
    } else {
        printf(" (Idle)\n");
    }

    printf("  %-35s : %.3f kWh\n", "Lifetime Charge Energy", data.chargeEnergyKwh);
    printf("  %-35s : %.3f kWh\n", "Lifetime Discharge Energy", data.dischargeEnergyKwh);

    if (data.chargeEnergyKwh > 0) {
        double efficiency = (data.dischargeEnergyKwh / data.chargeEnergyKwh) * 100.0;
        printf("  %-35s : %.1f %%\n", "Round-trip Efficiency", efficiency);
    }

    // Estimated cycles (assuming 3.6 kWh usable per module)
    double estimatedCycles = data.dischargeEnergyKwh / 3.6;
    printf("  %-35s : %.0f\n", "Estimated Cycles", estimatedCycles);

    // Balancing status
    if (data.balancingActive > 0) {
        printf("  %-35s : %d cells active (0x%04X)\n", "Balancing Status",
               data.balancingActive, data.balancingFlags);
    } else {
        printf("  %-35s : Inactive\n", "Balancing Status");
    }

    // Warnings and errors
    if (data.warnings1 != 0 || data.warnings2 != 0) {
        printf("  %-35s : 0x%04X / 0x%04X\n", "Warnings", data.warnings1, data.warnings2);
        if (data.warnings1 != 0) {
            printf("  %-35s :\n", "Warning Set 1");
            for (int bit = 0; bit < 16; ++bit) {
                if (data.warnings1 & (1 << bit)) {
                    if (bit < (int)WARNINGS.size()) {
                        printf("    - %s\n", WARNINGS[bit].c_str());
                    }
                }
            }
        }
        if (data.warnings2 != 0) {
            printf("  %-35s :\n", "Warning Set 2");
            for (int bit = 0; bit < 16; ++bit) {
                if (data.warnings2 & (1 << bit)) {
                    if (bit < (int)WARNINGS.size()) {
                        printf("    - %s\n", WARNINGS[bit].c_str());
                    }
                }
            }
        }
    }

    if (data.errors != 0) {
        printf("  %-35s : 0x%04X\n", "Error Bitmask", data.errors);
        printf("  %-35s :\n", "Active Errors");
        for (int bit = 0; bit < 16; ++bit) {
            if (data.errors & (1 << bit)) {
                if (bit < (int)ERRORS.size()) {
                    printf("    - %s\n", ERRORS[bit].c_str());
                }
            }
        }
    }

    // Cell voltages
    if (!data.cellVoltages.empty()) {
        printf("\n  %-35s :\n", "Individual Cell Voltages");
        for (size_t i = 0; i < data.cellVoltages.size(); ++i) {
            if (i % 4 == 0) printf("    ");
            printf("C%-2zu: %.3fV  ", i + 1, data.cellVoltages[i] / 1000.0);
            if ((i + 1) % 4 == 0) printf("\n");
        }
        if (data.cellVoltages.size() % 4 != 0) printf("\n");
    }

    // Cell temperatures
    if (!data.cellTemps.empty()) {
        printf("\n  %-35s :\n", "Temperature Sensors");
        for (size_t i = 0; i < data.cellTemps.size(); ++i) {
            if (i % 8 == 0) printf("    ");
            printf("T%-2zu: %3d°C  ", i + 1, data.cellTemps[i]);
            if ((i + 1) % 8 == 0) printf("\n");
        }
        if (data.cellTemps.size() % 8 != 0) printf("\n");
    }
}

void ModbusReader::readAllData(bool verbose)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                    BYD BATTERY SYSTEM - COMPLETE STATUS\n");
    printf("================================================================================\n\n");

    // Load all registers
    std::map<int, uint16_t> modbusRegs = loadRegs(0x0000, 0x66);
    std::map<int, uint16_t> statusRegs = loadRegs(0x0500, 25);
    modbusRegs.insert(statusRegs.begin(), statusRegs.end());

    // Read BMU serial number
    std::string bmuSerialBytes = readRegBytes(0, 18);
    std::string bmuSerial = trim(bmuSerialBytes);

    std::string batType;
    std::vector<std::string> batteryType;
    int batteryTypeIndex = modbusRegs[0x0011] & 0xFF;

    if (bmuSerial.find("P03") == 0 || bmuSerial.find("E0P3") == 0) {
        batType = "P3 (HV - Modules in Series)";
        batteryType = {"HVL", "HVM", "HVS"};
    } else if (bmuSerial.find("P02") == 0 || bmuSerial.find("P011") == 0) {
        batType = "P2 (LV - Modules in Parallel)";
        batteryType = {"LVL", "LVFlex(Lite)", "LVS/LVS Lite"};
    }

    int moduleCount = modbusRegs[0x0010] & 0x0F;
    int bmsCount = (modbusRegs[0x0010] >> 4) & 0x0F;
    int inverterType = (modbusRegs[0x0010] >> 8) & 0x0F;
    int applicationType = (modbusRegs[0x0011] >> 8) & 0xFF;
    int phaseType = (modbusRegs[0x0012] >> 8) & 0xFF;

    // ========== SYSTEM INFORMATION ==========
    printf("┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ SYSTEM INFORMATION                                                         │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : %s\n", "BMU Serial Number", bmuSerial.c_str());
    printf("  %-35s : %s\n", "Battery Type", batType.c_str());

    if (batteryTypeIndex < (int)batteryType.size()) {
        printf("  %-35s : %s\n", "Battery Model", batteryType[batteryTypeIndex].c_str());
    }

    printf("  %-35s : V%d.%d\n", "BMU Firmware Version (Area A)", modbusRegs[0x000C] >> 8, modbusRegs[0x000C] & 0xFF);
    printf("  %-35s : V%d.%d\n", "BMU Firmware Version (Area B)", modbusRegs[0x000D] >> 8, modbusRegs[0x000D] & 0xFF);
    printf("  %-35s : V%d.%d\n", "BMS Firmware Version", modbusRegs[0x000E] >> 8, modbusRegs[0x000E] & 0xFF);
    printf("  %-35s : BMU=%s / BMS=%s\n", "Working Area",
           workingArea(modbusRegs[0x000f] >> 8).c_str(),
           workingArea(modbusRegs[0x000f] & 0xFF).c_str());

    printf("  %-35s : %d\n", "Module Quantity", moduleCount);
    printf("  %-35s : %d\n", "BMS Quantity", bmsCount);

    if (inverterType < (int)INVERTER_LIST.size()) {
        printf("  %-35s : %s\n", "Inverter Type", INVERTER_LIST[inverterType].c_str());
    }

    if (applicationType < (int)APPLICATION_LIST.size()) {
        printf("  %-35s : %s\n", "Application", APPLICATION_LIST[applicationType].c_str());
    }

    if (phaseType < (int)PHASE_LIST.size()) {
        printf("  %-35s : %s\n", "Phase Configuration", PHASE_LIST[phaseType].c_str());
    }

    printf("  %-35s : 0x%04X\n", "Modbus Address", modbusRegs[0x004B]);
    printf("  %-35s : 0x%04X\n", "BMU MCU Type", modbusRegs[0x004C]);
    printf("  %-35s : 0x%04X\n", "BMS MCU Type", modbusRegs[0x004D]);

    // ========== CURRENT STATUS ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ CURRENT STATUS                                                             │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : %d %%\n", "State of Charge (SOC)", modbusRegs[0x0500]);
    printf("  %-35s : %d %%\n", "State of Health (SOH)", modbusRegs[0x0503]);
    printf("  %-35s : %.2f V\n", "Battery Voltage", modbusRegs[0x0505] * 0.01);
    printf("  %-35s : %.2f V\n", "Output Voltage", modbusRegs[0x0510] * 0.01);
    printf("  %-35s : %.1f A\n", "Current", signed16bit(modbusRegs[0x0504]) * 0.1);

    double power = signed16bit(modbusRegs[0x0504]) * 0.1 * modbusRegs[0x0510] * 0.01;
    printf("  %-35s : %.2f W", "Power", power);
    if (power > 0) {
        printf(" (Discharging)\n");
    } else if (power < 0) {
        printf(" (Charging)\n");
    } else {
        printf(" (Idle)\n");
    }

    // ========== CELL INFORMATION ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ CELL INFORMATION                                                           │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : %.2f V\n", "Maximum Cell Voltage", modbusRegs[0x0501] * 0.01);
    printf("  %-35s : %.2f V\n", "Minimum Cell Voltage", modbusRegs[0x0502] * 0.01);
    printf("  %-35s : %.2f V\n", "Cell Voltage Delta", (modbusRegs[0x0501] - modbusRegs[0x0502]) * 0.01);

    // ========== TEMPERATURE INFORMATION ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ TEMPERATURE INFORMATION                                                    │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : %d °C\n", "Maximum Cell Temperature", modbusRegs[0x0506]);
    printf("  %-35s : %d °C\n", "Minimum Cell Temperature", modbusRegs[0x0507]);
    printf("  %-35s : %d °C\n", "BMU Temperature", modbusRegs[0x0508]);
    printf("  %-35s : %d °C\n", "Temperature Delta", modbusRegs[0x0506] - modbusRegs[0x0507]);

    // ========== CYCLE INFORMATION ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ CYCLE INFORMATION                                                          │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : %d\n", "Charge Cycles", modbusRegs[0x0511]);
    printf("  %-35s : %d\n", "Discharge Cycles", modbusRegs[0x0513]);
    printf("  %-35s : %d\n", "Total Cycles", modbusRegs[0x0511] + modbusRegs[0x0513]);

    // System energy (UINT32, little-endian)
    if (modbusRegs.count(0x0511) && modbusRegs.count(0x0512)) {
        double sysChargeEnergyKwh = (modbusRegs[0x0512] * 65536.0 + modbusRegs[0x0511]) * 0.001;
        printf("  %-35s : %.3f kWh\n", "System Lifetime Charge Energy", sysChargeEnergyKwh);
    }

    if (modbusRegs.count(0x0513) && modbusRegs.count(0x0514)) {
        double sysDischargeEnergyKwh = (modbusRegs[0x0514] * 65536.0 + modbusRegs[0x0513]) * 0.001;
        printf("  %-35s : %.3f kWh\n", "System Lifetime Discharge Energy", sysDischargeEnergyKwh);

        if (modbusRegs.count(0x0511) && modbusRegs.count(0x0512)) {
            double sysChargeEnergyKwh = (modbusRegs[0x0512] * 65536.0 + modbusRegs[0x0511]) * 0.001;
            if (sysChargeEnergyKwh > 0) {
                double sysEfficiency = (sysDischargeEnergyKwh / sysChargeEnergyKwh) * 100.0;
                printf("  %-35s : %.1f %%\n", "System Round-trip Efficiency", sysEfficiency);
            }
        }
    }

    // ========== ERROR AND WARNING STATUS ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ ERROR AND WARNING STATUS                                                   │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    uint16_t errorBitmask = modbusRegs[0x050d];
    printf("  %-35s : 0x%04X\n", "Error Bitmask", errorBitmask);

    if (errorBitmask == 0) {
        printf("  %-35s : No Errors\n", "Error Status");
    } else {
        printf("  %-35s :\n", "Active Errors");
        for (int bit = 0; bit < 16; ++bit) {
            if (errorBitmask & (1 << bit)) {
                if (bit < (int)ERRORS.size()) {
                    printf("    - %s\n", ERRORS[bit].c_str());
                }
            }
        }
    }

    // ========== ADDITIONAL INFORMATION ==========
    printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
    printf("│ ADDITIONAL INFORMATION                                                     │\n");
    printf("└────────────────────────────────────────────────────────────────────────────┘\n\n");

    printf("  %-35s : V%d.%d\n", "BMU Version (Status)", modbusRegs[0x050a] >> 8, modbusRegs[0x050a] & 0xFF);
    printf("  %-35s : V%d.%d\n", "Parameter Table Version", modbusRegs[0x050e] >> 8, modbusRegs[0x050e] & 0xFF);

    // System date/time if available
    if (modbusRegs.count(0x0063)) {
        int year = (modbusRegs[0x0063] >> 8) + 2000;
        int month = modbusRegs[0x0063] & 0xFF;
        int day = modbusRegs[0x0064] >> 8;
        int hour = modbusRegs[0x0064] & 0xFF;
        int minute = modbusRegs[0x0065] >> 8;
        int second = modbusRegs[0x0065] & 0xFF;

        printf("  %-35s : %04d-%02d-%02d %02d:%02d:%02d\n",
               "System Date/Time", year, month, day, hour, minute, second);
    }

    // ========== MODULE DETAILED INFORMATION ==========
    if (verbose && moduleCount > 0) {
        printf("\n┌────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│ QUERYING DETAILED MODULE INFORMATION                                       │\n");
        printf("└────────────────────────────────────────────────────────────────────────────┘\n");
        printf("\nDetected %d module(s). Reading detailed information...\n", moduleCount);

        for (int i = 1; i <= moduleCount; ++i) {
            printf("\nQuerying BMS Module %d/%d...\n", i, moduleCount);
            BmsModuleData moduleData = queryModule(i);

            if (moduleData.maxCellVoltage != 0 || moduleData.minCellVoltage != 0) {
                displayModuleData(moduleData, i);
            } else {
                printf("  Failed to read data from Module %d\n", i);
            }
        }
    } else if (!verbose && moduleCount > 0) {
        printf("\nNote: Use -v option to display detailed per-module information\n");
    }

    printf("\n================================================================================\n");
    printf("                              END OF REPORT\n");
    printf("================================================================================\n\n");
}
