cmake_minimum_required(VERSION 3.16)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find INDI
find_package(PkgConfig REQUIRED)
find_package(INDI REQUIRED)
include_directories(${INDI_INCLUDE_DIRS})
link_directories(${INDI_LIBRARY_DIRS})

# Find lgpio
find_package(LGPIO REQUIRED)
include_directories(${LGPIO_INCLUDE_DIRS})
link_directories(${LGPIO_LIBRARY_DIRS})

# Sources
set(SOURCES
    indi_waveshare_28byj48.cpp
)

# Build shared library
add_library(indi_waveshare_28byj48 SHARED ${SOURCES})

target_link_libraries(indi_waveshare_28byj48 ${INDI_LIBRARIES} ${LGPIO_LIBRARIES})

# Install driver
install(TARGETS indi_waveshare_28byj48
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/indi)

# XML for KStars
set(XML_FILE indi_waveshare_28byj48.xml)

install(FILES ${XML_FILE} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/indi)
