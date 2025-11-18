cmake_minimum_required(VERSION 3.5)
project(indi_hatb_focuser CXX)

# Use C++17 for libindi 2.1.x (std::any, etc.)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If you have a local FindINDI.cmake, use it as well
set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/"
    ${CMAKE_MODULE_PATH}
)

# Find INDI
find_package(INDI REQUIRED)

# Focuser driver executable
add_executable(indi_hatb_focuser
    indi_hatb_focuser.cpp
    hatb_motor.h
)

target_include_directories(indi_hatb_focuser
    PRIVATE
        ${INDI_INCLUDE_DIR}
)

target_link_libraries(indi_hatb_focuser
    ${INDI_LIBRARIES}
    lgpio
)

# Fallback for INDI_DATA_DIR if the package didn't define it
if(NOT INDI_DATA_DIR)
    set(INDI_DATA_DIR "/usr/share/indi")
endif()

# Install binary
install(TARGETS indi_hatb_focuser
        RUNTIME DESTINATION bin)

# Install XML, with your exact structure / name
install(FILES indi_hatb_focuser.xml
        DESTINATION ${INDI_DATA_DIR})
