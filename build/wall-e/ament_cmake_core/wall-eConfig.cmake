# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wall-e_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wall-e_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wall-e_FOUND FALSE)
  elseif(NOT wall-e_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wall-e_FOUND FALSE)
  endif()
  return()
endif()
set(_wall-e_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wall-e_FIND_QUIETLY)
  message(STATUS "Found wall-e: 0.0.0 (${wall-e_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wall-e' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wall-e_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wall-e_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wall-e_DIR}/${_extra}")
endforeach()
