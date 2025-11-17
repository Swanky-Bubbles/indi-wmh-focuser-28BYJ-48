cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(INDI REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(LGPIO REQUIRED)

# Source files
set(SOURCES
    indi_waveshare_28byj48.cpp
    indi_waveshare_28byj48.h
)

# Build shared library
add_library(indi_waveshare_28byj48 SHARED ${SOURCES})

# Include directories
target_include_directories(indi_waveshare_28byj48 PRIVATE
    ${INDI_INCLUDE_DIRS}
    ${LGPIO_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(indi_waveshare_28byj48
    ${INDI_LIBRARIES}
    ${LGPIO_LIBRARIES}
    pthread
)

# Install driver
install(TARGETS indi_waveshare_28byj48 DESTINATION lib/indi)
install(FILES indi_waveshare_28byj48.xml DESTINATION /usr/share/indi)
