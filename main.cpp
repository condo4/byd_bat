/*
 * BYD Battery Status Checker - Main Program
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

#include <iostream>
#include <cstdlib>
#include <cstring>
#include "modbusreader.h"

void printHelp(const char *programName)
{
    printf("BYD Battery Status Checker\n");
    printf("==========================\n\n");
    printf("Usage: %s [options] <ip_address> [port]\n\n", programName);
    printf("Arguments:\n");
    printf("  ip_address    IP address of the BYD Battery Management Unit (required)\n");
    printf("  port          Modbus TCP port (default: 8080)\n\n");
    printf("Options:\n");
    printf("  -v, --verbose Display detailed per-module information (individual cell\n");
    printf("                voltages and temperatures for each module)\n");
    printf("  -h, --help    Display this help message\n\n");
    printf("Examples:\n");
    printf("  %s 192.168.1.233\n", programName);
    printf("  %s 192.168.1.233 8080\n", programName);
    printf("  %s -v 192.168.1.233\n", programName);
    printf("  %s --verbose 192.168.1.233 8080\n\n", programName);
    printf("Description:\n");
    printf("  This tool connects to a BYD Battery system via Modbus RTU over TCP\n");
    printf("  and reads comprehensive battery status including:\n");
    printf("  - System information and configuration\n");
    printf("  - Current status (SOC, SOH, voltage, current, power)\n");
    printf("  - Cell voltages and temperatures\n");
    printf("  - Error and warning status\n\n");
    printf("  Use -v to also display:\n");
    printf("  - Detailed per-module information\n");
    printf("  - Individual cell voltages and temperatures for each module\n\n");
}

int main(int argc, char *argv[])
{
    bool verbose = false;
    int argIdx = 1;

    // Check for help
    if (argc < 2) {
        printHelp(argc > 0 ? argv[0] : "byd_bat");
        return 1;
    }

    // Parse options
    while (argIdx < argc && argv[argIdx][0] == '-') {
        if (strcmp(argv[argIdx], "-h") == 0 || strcmp(argv[argIdx], "--help") == 0) {
            printHelp(argv[0]);
            return 0;
        } else if (strcmp(argv[argIdx], "-v") == 0 || strcmp(argv[argIdx], "--verbose") == 0) {
            verbose = true;
            argIdx++;
        } else {
            fprintf(stderr, "Error: Unknown option '%s'\n\n", argv[argIdx]);
            printHelp(argv[0]);
            return 1;
        }
    }

    // Check if IP address is provided
    if (argIdx >= argc) {
        fprintf(stderr, "Error: IP address required\n\n");
        printHelp(argv[0]);
        return 1;
    }

    // Parse IP address and optional port
    std::string ipAddress = argv[argIdx++];
    int port = 8080;  // Default port

    if (argIdx < argc) {
        char *endptr;
        port = strtol(argv[argIdx], &endptr, 10);
        if (*endptr != '\0' || port <= 0 || port > 65535) {
            fprintf(stderr, "Error: Invalid port number '%s'\n\n", argv[argIdx]);
            printHelp(argv[0]);
            return 1;
        }
    }

    // Display connection info
    printf("Connecting to BYD Battery at %s:%d%s...\n\n",
           ipAddress.c_str(), port, verbose ? " (verbose mode)" : "");

    // Create ModbusReader instance
    ModbusReader reader(ipAddress, port);

    // Connect to the Modbus device
    if (!reader.connectToDevice()) {
        fprintf(stderr, "\nError: Failed to connect to Modbus device at %s:%d\n",
                ipAddress.c_str(), port);
        fprintf(stderr, "Please check:\n");
        fprintf(stderr, "  - IP address is correct\n");
        fprintf(stderr, "  - Battery system is powered on\n");
        fprintf(stderr, "  - Network connectivity\n");
        fprintf(stderr, "  - Firewall settings\n\n");
        return 1;
    }

    // Read all data once and display
    reader.readAllData(verbose);

    return 0;
}
