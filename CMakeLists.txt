cmake_minimum_required(VERSION 3.10)
project(indi_waveshare_28BYJ48)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Allow finding INDI
set(CMAKE_PREFIX_PATH "/usr;/usr/local")
find_package(INDI REQUIRED)

find_package(LGPIO REQUIRED)

if (NOT LGPIO_FOUND)
    message(FATAL_ERROR "lgpio not found. Please install liblgpio-dev and ensure pkg-config can find it.")
endif()

include_directories(
    ${LGPIO_INCLUDE_DIRS}
    ${INDI_INCLUDE_DIRS}
)

link_directories(
    ${LGPIO_LIBRARY_DIRS}
    ${INDI_LIBRARY_DIRS}
)

add_definitions(${LGPIO_CFLAGS_OTHER})

# The device source
add_library(indi_waveshare_28byj48 SHARED
    indi_waveshare_28byj48.cpp
)

target_link_libraries(indi_waveshare_28byj48
    ${LGPIO_LIBRARIES}
    ${INDI_LIBRARIES}
)

# Install for INDI/KStars
install(TARGETS indi_waveshare_28byj48
    LIBRARY DESTINATION lib/indi
)

install(FILES indi_waveshare_28byj48.xml
    DESTINATION /usr/share/indi
)
