cmake_minimum_required(VERSION 3.1.0)

project(qmc5883 VERSION 1.0.0 LANGUAGES C)

add_definitions(-DHAVE_CLOCK_GETTIME=1)
add_definitions(-DVERSION="${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

add_executable(qmc5883 src/main.c driver/src/qmc5883.c driver/src/i2c.c)
target_link_libraries(qmc5883 m)
include_directories(./driver/inc)
set_target_properties(qmc5883 PROPERTIES
    OUTPUT_NAME qmc5883)

include(GNUInstallDirs)
install(TARGETS qmc5883 DESTINATION ${CMAKE_INSTALL_BINDIR})
message(STATUS "summary of build options:
    Install prefix:  ${CMAKE_INSTALL_PREFIX}
    Target system:   ${CMAKE_SYSTEM_NAME}
    Compiler:
      C compiler:    ${CMAKE_C_COMPILER} (${CMAKE_C_COMPILER_ID})
      CFLAGS:        ${CMAKE_C_FLAGS_${_build_type}} ${CMAKE_C_FLAGS}
")
