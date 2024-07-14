# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sub_neartest_distance_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sub_neartest_distance_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sub_neartest_distance_FOUND FALSE)
  elseif(NOT sub_neartest_distance_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sub_neartest_distance_FOUND FALSE)
  endif()
  return()
endif()
set(_sub_neartest_distance_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sub_neartest_distance_FIND_QUIETLY)
  message(STATUS "Found sub_neartest_distance: 1.0.0 (${sub_neartest_distance_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sub_neartest_distance' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sub_neartest_distance_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sub_neartest_distance_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sub_neartest_distance_DIR}/${_extra}")
endforeach()
