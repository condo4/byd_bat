cmake_minimum_required(VERSION 3.16)
project(byd_bat VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find libmodbus
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Add executable
add_executable(byd_bat
    main.cpp
    modbusreader.h
    modbusreader.cpp
)

# Link libmodbus library
target_link_libraries(byd_bat
    ${MODBUS_LIBRARIES}
)

target_include_directories(byd_bat PRIVATE ${MODBUS_INCLUDE_DIRS})

# Installation rules
install(TARGETS byd_bat
    RUNTIME DESTINATION bin
)
