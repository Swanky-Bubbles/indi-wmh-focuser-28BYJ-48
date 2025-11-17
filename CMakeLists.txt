cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28byj48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(PkgConfig REQUIRED)
pkg_check_modules(INDI REQUIRED indi)

include_directories(${INDI_INCLUDE_DIRS})
link_directories(${INDI_LIBRARY_DIRS})

add_library(indi_waveshare_28byj48 SHARED
    indi_waveshare_28byj48.cpp
)

target_link_libraries(indi_waveshare_28byj48 ${INDI_LIBRARIES})

# install plugin and XML
install(TARGETS indi_waveshare_28byj48 DESTINATION lib/indi)
install(FILES indi_waveshare_28byj48.xml DESTINATION /usr/share/indi)
