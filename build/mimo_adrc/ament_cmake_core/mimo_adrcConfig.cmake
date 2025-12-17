# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mimo_adrc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mimo_adrc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mimo_adrc_FOUND FALSE)
  elseif(NOT mimo_adrc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mimo_adrc_FOUND FALSE)
  endif()
  return()
endif()
set(_mimo_adrc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mimo_adrc_FIND_QUIETLY)
  message(STATUS "Found mimo_adrc: 0.0.0 (${mimo_adrc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mimo_adrc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mimo_adrc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mimo_adrc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${mimo_adrc_DIR}/${_extra}")
endforeach()
