cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(PkgConfig REQUIRED)
find_package(INDI REQUIRED)

include_directories("/usr/include/libindi")
link_directories(${INDI_LIBRARY_DIRS})

add_library(indi_waveshare_28byj48 SHARED
    indi_waveshare_28byj48.cpp
)

target_link_libraries(indi_waveshare_28byj48 ${INDI_LIBRARIES})

# Install driver library
install(TARGETS indi_waveshare_28byj48 DESTINATION lib/indi)

# Install XML for KStars / INDI
install(FILES indi_waveshare_28byj48.xml DESTINATION /usr/share/indi)
