cmake_minimum_required(VERSION 3.5)
project(indi_hatb_focuser CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# In case you have custom FindINDI.cmake
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(INDI REQUIRED)

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

install(TARGETS indi_hatb_focuser RUNTIME DESTINATION bin)

# XML driver definition
install(FILES indi_hatb_focuser.xml DESTINATION ${INDI_DATA_DIR})
