# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_untitled2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED untitled2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(untitled2_FOUND FALSE)
  elseif(NOT untitled2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(untitled2_FOUND FALSE)
  endif()
  return()
endif()
set(_untitled2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT untitled2_FIND_QUIETLY)
  message(STATUS "Found untitled2: 0.0.0 (${untitled2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'untitled2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${untitled2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(untitled2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${untitled2_DIR}/${_extra}")
endforeach()
