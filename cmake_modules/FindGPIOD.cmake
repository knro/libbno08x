# - Try to find GPIOD
# Once done this will define
#
#  GPIOD_FOUND - system has libgpiod
#  GPIOD_INCLUDE_DIR - the libgpiod include directory
#  GPIOD_LIBRARIES - Link these to use libgpiod
#  GPIOD::CXX - Imported target for the C++ bindings
#
# Components:
#  CXX - C++ bindings (gpiodcxx)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Use pkg-config to find libgpiod
find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(LIBGPIOD QUIET libgpiodcxx)
endif()

if (LIBGPIOD_FOUND)
  set(GPIOD_FOUND TRUE)
  set(GPIOD_LIBRARIES ${LIBGPIOD_LIBRARIES})
  set(GPIOD_INCLUDE_DIR ${LIBGPIOD_INCLUDE_DIRS})
  set(LIBGPIOD_VERSION ${LIBGPIOD_VERSION}) # Set LIBGPIOD_VERSION for CMakeLists.txt
  message(STATUS "Found libgpiod (pkg-config): ${GPIOD_LIBRARIES} (Version: ${LIBGPIOD_VERSION})")
else()
  # Fallback to find_path and find_library if pkg-config fails
  if (GPIOD_INCLUDE_DIR AND GPIOD_LIBRARIES)
    # in cache already
    set(GPIOD_FOUND TRUE)
    message(STATUS "Found libgpiod: ${GPIOD_LIBRARIES}")
  else()
    find_path(GPIOD_INCLUDE_DIR gpiod.hpp
      ${_obIncDir}
      ${GNUWIN32_DIR}/include
      /usr/local/include
    )

    find_library(GPIOD_LIBRARIES NAMES gpiodcxx
      PATHS
      ${_obLinkDir}
      ${GNUWIN32_DIR}/lib
      /usr/local/lib
    )

    if(GPIOD_INCLUDE_DIR AND GPIOD_LIBRARIES)
      set(GPIOD_FOUND TRUE)
    else()
      set(GPIOD_FOUND FALSE)
    endif()

    if(GPIOD_FOUND)
      if(NOT GPIOD_FIND_QUIETLY)
        message(STATUS "Found GPIOD: ${GPIOD_LIBRARIES}")
      endif()
    else()
      if(GPIOD_FIND_REQUIRED)
        message(FATAL_ERROR "libgpiod not found. Please install libgpiod-dev")
      endif()
    endif()
  endif()
endif()

# Check if CXX component is requested
if("CXX" IN_LIST GPIOD_FIND_COMPONENTS)
  # Create imported target for C++ bindings
  if(NOT TARGET GPIOD::CXX)
    add_library(GPIOD::CXX UNKNOWN IMPORTED)
    set_target_properties(GPIOD::CXX PROPERTIES
      IMPORTED_LOCATION "${GPIOD_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${GPIOD_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(GPIOD_INCLUDE_DIR GPIOD_LIBRARIES LIBGPIOD_VERSION)
