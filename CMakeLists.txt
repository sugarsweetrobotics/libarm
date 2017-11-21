cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(arm
  VERSION 1.0.0)
set(PROJECT_DESCRIPTION “arm”)
set(PROJECT_VENDOR "ssr")
set(PROJECT_VENDOR_CONTACT "ysuga@ssr.tokyo")
set(PROJECT_AUTHOR "ssr")
set(PROJECT_AUTHOR_SHORT "ssr")


# Set up installation directories
set(BIN_INSTALL_DIR "bin")
set(LIB_INSTALL_DIR "lib")
# set(MAN_INSTALL_DIR "share/man/man3")
set(INC_INSTALL_DIR
    "include/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(SHARE_INSTALL_DIR
    "share/${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

# Set up installation directories
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  if (CMAKE_CL_64)
    set(SYSDIR $ENV{SYSTEMROOT}\\system32)
  else (CMAKE_CL_64)
    set(SYSDIR $ENV{SYSTEMROOT}\\syswow64)
  endif(CMAKE_CL_64)
else(${CMAKE_SYSTEM_NAME} MATCHES Windows)
endif(${CMAKE_SYSTEM_NAME} MATCHES Windows)

if(WIN32)
  set (SYS_INSTALL_DIR ${SYSDIR})
endif(WIN32)

# Subdirectories
add_subdirectory(include)
add_subdirectory(src)

# add_subdirectory(doc)


# Package creation
# By default, do not warn when built on machines using only VS Express:
#IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
#SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
#ENDIF()
#include(InstallRequiredSystemLibraries)

#set(cpack_options "${PROJECT_BINARY_DIR}/cpack_options.cmake")

#configure_file("${CMAKE_SOURCE_DIR}/cmake/cpack_options.cmake.in"
#    ${cpack_options} @ONLY)
#set(CPACK_PROJECT_CONFIG_FILE ${cpack_options})
#include(${CPACK_PROJECT_CONFIG_FILE})
#include(CPack)

