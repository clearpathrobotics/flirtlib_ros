# - Find FLIRT include dirs and libraries
# Use this module by invoking find_package with the form:
#  find_package(FLIRT
#    [version] [EXACT]      # Minimum or EXACT version e.g. 1.0.0
#    [REQUIRED]             # Fail with error if FLIRT is not found
#    [COMPONENTS <libs>...] # FLIRT libraries by their canonical name
#  )                        # e.g. "feature" for "libflirt_feature"
# This module finds headers and requested component libraries.
# Results are reported in variables:
#  FLIRT_FOUND            - True if headers and requested libraries were found
#  FLIRT_INCLUDE_DIRS     - FLIRT include directories
#  FLIRT_LIBRARIES        - FLIRT component libraries to be linked

# Copyright 2015 Enrique Fernandez <efernandez@clearpathrobotics.com>
# Clearpath Robotics, Inc.

# Search for FLIRT:
find_path(FLIRT_INCLUDE_DIRS NAMES feature/Descriptor.h
  PATHS
    /usr/include/flirtlib
    /usr/local/include/flirtlib
    /opt/local/include/flirtlib)

find_library(FLIRT_FEATURE NAMES feature
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

find_library(FLIRT_GEOMETRY NAMES geometry
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

find_library(FLIRT_GUI NAMES gui
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

find_library(FLIRT_SENSORS NAMES sensors
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

find_library(FLIRT_SENSORSTREAM NAMES sensorstream
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

# NO_DEFAULT_PATH is needed to avoid taking
# the libutils.so from /opt/ros/$(rosversion -d)/lib
find_library(FLIRT_UTILS NAMES utils
  NO_DEFAULT_PATH
  PATHS
    /usr/lib/flirtlib
    /usr/local/lib/flirtlib
    /opt/local/lib/flirtlib)

set(FLIRT_LIBRARIES
  ${FLIRT_FEATURE}
  ${FLIRT_GEOMETRY}
  ${FLIRT_GUI}
  ${FLIRT_SENSORS}
  ${FLIRT_SENSORSTREAM}
  ${FLIRT_UTILS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FLIRT DEFAULT_MSG
  FLIRT_INCLUDE_DIRS FLIRT_LIBRARIES)
