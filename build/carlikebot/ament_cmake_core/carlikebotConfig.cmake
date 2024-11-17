# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_carlikebot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED carlikebot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(carlikebot_FOUND FALSE)
  elseif(NOT carlikebot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(carlikebot_FOUND FALSE)
  endif()
  return()
endif()
set(_carlikebot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT carlikebot_FIND_QUIETLY)
  message(STATUS "Found carlikebot: 0.0.0 (${carlikebot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'carlikebot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${carlikebot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(carlikebot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${carlikebot_DIR}/${_extra}")
endforeach()
