# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_object_detection_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED object_detection_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(object_detection_FOUND FALSE)
  elseif(NOT object_detection_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(object_detection_FOUND FALSE)
  endif()
  return()
endif()
set(_object_detection_CONFIG_INCLUDED TRUE)

# output package information
if(NOT object_detection_FIND_QUIETLY)
  message(STATUS "Found object_detection: 1.0.0 (${object_detection_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'object_detection' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${object_detection_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(object_detection_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${object_detection_DIR}/${_extra}")
endforeach()
