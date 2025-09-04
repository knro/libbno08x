cmake_minimum_required(VERSION 3.10)
project(bno08x LANGUAGES C CXX VERSION 0.0.1)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiodcxx)

if (LIBGPIOD_VERSION VERSION_GREATER_EQUAL "2.0")
  add_definitions(-DHAVE_LIBGPIOD_V2)
endif()

# Add the library source files
include_directories(src)
add_library(bno08x SHARED src/BNO08x.cpp src/sh2.c src/shtp.c src/sh2_SensorValue.c src/sh2_util.c)
target_link_libraries(bno08x PUBLIC ${LIBGPIOD_LIBRARIES})
set_target_properties(bno08x PROPERTIES
    PUBLIC_HEADER "src/BNO08x.h;src/sh2.h;src/sh2_err.h;src/sh2_hal.h;src/sh2_SensorValue.h;src/sh2_util.h;src/shtp.h"
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

# Example executable
add_executable(rotation_vector_example examples/rotation_vector.cpp)
target_link_libraries(rotation_vector_example bno08x)

install(TARGETS bno08x
    EXPORT BNO08xTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/bno08x
)

include(CMakePackageConfigHelpers)
configure_package_config_file(BNO08xConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/BNO08xConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/BNO08x"
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/BNO08xConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT BNO08xTargets
    FILE BNO08xTargets.cmake
    NAMESPACE BNO08x::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/BNO08x
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/BNO08xConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/BNO08xConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/BNO08x"
)

configure_file(bno08x.pc.in bno08x.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/bno08x.pc DESTINATION ${CMAKE_INSTALL_DATADIR}/pkgconfig)
