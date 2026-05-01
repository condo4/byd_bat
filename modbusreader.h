/*
 * BYD Battery Status Checker - Modbus Reader Header
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

#ifndef MODBUSREADER_H
#define MODBUSREADER_H

#include <string>
#include <vector>
#include <map>
#include <modbus/modbus.h>

struct BmsModuleData {
    int bmsId;
    int16_t maxCellVoltage;      // mV
    int16_t minCellVoltage;      // mV
    uint8_t maxVoltageCell;
    uint8_t maxVoltageModule;
    int16_t maxTemp;             // °C
    int16_t minTemp;             // °C
    uint8_t maxTempCell;
    uint8_t maxTempModule;
    uint16_t balancingFlags;
    int balancingActive;
    double chargeEnergyKwh;
    double dischargeEnergyKwh;
    double batteryVoltage;
    double outputVoltage;
    double soc;
    int16_t soh;
    double current;
    uint16_t warnings1;
    uint16_t warnings2;
    uint16_t errors;
    std::string serial;
    std::vector<int16_t> cellVoltages;    // 16 cells in mV
    std::vector<int8_t> cellTemps;        // up to 8 temps in °C
};

class ModbusReader
{
public:
    explicit ModbusReader(const std::string &host, int port);
    ~ModbusReader();

    bool connectToDevice();
    void readAllData(bool verbose = false);

private:
    // Helper functions
    std::vector<uint16_t> readRegs(int start, int regCount = 1);
    bool writeRegs(int start, const std::vector<uint16_t> &values);
    std::string readRegBytes(int start, int byteCount = 1);
    std::map<int, uint16_t> loadRegs(int startReg, int regCount = 1);
    std::string bitmaskStr(uint16_t bitm, const std::vector<std::string> &bitmaskList);
    int16_t signed16bit(uint16_t val);
    std::string workingArea(int area);

    // BMS module query
    BmsModuleData queryModule(int bmsId);
    void displayModuleData(const BmsModuleData &data, int moduleNumber);

    modbus_t *m_modbusCtx;
    int m_socket;
    std::string m_host;
    int m_port;

    // Constants
    static const std::vector<std::string> INVERTER_LIST;
    static const std::vector<std::string> APPLICATION_LIST;
    static const std::vector<std::string> PHASE_LIST;
    static const std::vector<std::string> ERRORS;
    static const std::vector<std::string> WARNINGS;
    static const std::vector<std::string> WARNINGS3;
};

#endif // MODBUSREADER_H
