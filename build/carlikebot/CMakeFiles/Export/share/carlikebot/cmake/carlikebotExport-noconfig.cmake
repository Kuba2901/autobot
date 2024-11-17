#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "carlikebot::carlikebot" for configuration ""
set_property(TARGET carlikebot::carlikebot APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(carlikebot::carlikebot PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcarlikebot.so"
  IMPORTED_SONAME_NOCONFIG "libcarlikebot.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS carlikebot::carlikebot )
list(APPEND _IMPORT_CHECK_FILES_FOR_carlikebot::carlikebot "${_IMPORT_PREFIX}/lib/libcarlikebot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
