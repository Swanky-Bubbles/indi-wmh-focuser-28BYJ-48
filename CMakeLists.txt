cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")

find_package(INDI REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

# --- Build shared library ---
add_library(indi_waveshare_28byj48 SHARED indi_waveshare_28byj48.cpp)
target_include_directories(indi_waveshare_28byj48 PRIVATE ${INDI_INCLUDE_DIRS})
target_link_libraries(indi_waveshare_28byj48 PRIVATE ${INDI_LIBRARIES} lgpio pthread)

# --- Install rules ---
# Install shared library
install(TARGETS indi_waveshare_28byj48
        LIBRARY DESTINATION /usr/lib/indi)

# Install XML description
install(FILES indi-waveshare-28byj48.xml
        DESTINATION /usr/share/indi)
