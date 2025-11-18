cmake_minimum_required(VERSION 3.15)
project(indi_drv8825_focuser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add custom CMake modules folder
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find INDI
find_package(PkgConfig REQUIRED)
pkg_check_modules(INDI REQUIRED indi)

# Find libgpiod
pkg_check_modules(GPIOD REQUIRED libgpiod)

include_directories(
    ${INDI_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
)

link_directories(
    ${INDI_LIBRARY_DIRS}
    ${GPIOD_LIBRARY_DIRS}
)

add_definitions(${INDI_CFLAGS_OTHER} ${GPIOD_CFLAGS_OTHER})

# Build the driver
add_library(indi_drv8825_focuser SHARED
    indi_drv8825_focuser.cpp
    indi_drv8825_focuser.h
)

target_link_libraries(indi_drv8825_focuser
    ${INDI_LIBRARIES}
    ${GPIOD_LIBRARIES}
)

# Install the driver to the standard INDI drivers directory
install(TARGETS indi_drv8825_focuser
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/indi)
