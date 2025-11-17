# FindLGPIO.cmake
find_library(LGPIO_LIBRARIES NAMES lgpio)
find_path(LGPIO_INCLUDE_DIRS NAMES lgpio.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LGPIO DEFAULT_MSG LGPIO_LIBRARIES LGPIO_INCLUDE_DIRS)

mark_as_advanced(LGPIO_LIBRARIES LGPIO_INCLUDE_DIRS)