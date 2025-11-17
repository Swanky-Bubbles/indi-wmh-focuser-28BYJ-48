cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")

# Find INDI
find_package(INDI REQUIRED)

# Use pkg-config for lgpio
find_package(PkgConfig REQUIRED)
pkg_check_modules(LGPIO REQUIRED lgpio)

# Build driver
add_library(indi_waveshare_28byj48 SHARED
    indi_waveshare_28byj48.cpp
    indi_waveshare_28byj48.h
)

target_include_directories(indi_waveshare_28byj48 PRIVATE
    ${INDI_INCLUDE_DIRS}
    ${LGPIO_INCLUDE_DIRS}
)

target_link_libraries(indi_waveshare_28byj48
    ${INDI_LIBRARIES}
    ${LGPIO_LIBRARIES}
    pthread
)

install(TARGETS indi_waveshare_28byj48 DESTINATION lib/indi)
install(FILES indi_waveshare_28byj48.xml DESTINATION /usr/share/indi)
