cmake_minimum_required(VERSION 3.10)
project(indi_drv8825_focuser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# Find pkg-config first
find_package(PkgConfig REQUIRED)

# Find INDI
pkg_check_modules(INDI REQUIRED libindi)

# Find libgpiod
pkg_check_modules(GPIOD REQUIRED libgpiod)

# Include directories
include_directories(
    ${INDI_INCLUDE_DIRS}
    ${GPIOD_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    indi_drv8825_focuser.cpp
)

# Build the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${INDI_LIBRARIES}
    ${GPIOD_LIBRARIES}
    pthread
)
