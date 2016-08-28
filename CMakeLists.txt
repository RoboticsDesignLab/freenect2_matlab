CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

SET(PROJECT_VER_MAJOR 0)
SET(PROJECT_VER_MINOR 0)
SET(PROJECT_VER_PATCH 1)
SET(PROJECT_VER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")
SET(PROJECT_APIVER "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}")

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()

IF(WIN32)
  # no permission for the default install prefix %ProgramFiles%
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "installation path")
ENDIF()

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(freenect2_matlab)

SET(MY_DIR ${freenect2_matlab_SOURCE_DIR})

OPTION(BUILD_SHARED_LIBS "Build shared (ON) or static (OFF) libraries" ON)

IF(MSVC)
  # suppress several "possible loss of data" warnings, and
  # "zero-length array in struct" from libusb.h
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4267 /wd4244 /wd4200 /wd4305 /wd4146 /Od")
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS) # no warning for getenv()
ELSE()
  # Heed warnings from non-MSVC compilers
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
ENDIF()

SET(HAVE_CXX11 disabled)
IF(ENABLE_CXX11)
  INCLUDE(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  IF(COMPILER_SUPPORTS_CXX11)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    SET(HAVE_CXX11 yes)
  ELSEIF(COMPILER_SUPPORTS_CXX0X)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    SET(HAVE_CXX11 c++0x)
  ELSE()
    SET(HAVE_CXX11 no)
    MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  ENDIF()
ENDIF(ENABLE_CXX11)

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${MY_DIR}/cmake_modules)

#set the default path for built executables to the "bin" directory
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

#set the default path for built libraries to the "lib" directory
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# dependencies
FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(Matlab REQUIRED COMPONENTS MX_LIBRARY)

# Add includes
INCLUDE_DIRECTORIES(
  "${MY_DIR}/include"
  ${freenect2_INCLUDE_DIRS}
  ${Matlab_INCLUDE_DIRS}
)

SET(SOURCES
  include/freenect2.h

  src/freenect2.cpp
)

SET(LIBRARIES
  ${freenect2_DEBUG_LIBRARIES}
  ${Matlab_LIBRARIES}
)

ADD_DEFINITIONS(-DMATLAB_MEX_FILE)
#ADD_LIBRARY(freenect2_matlab ${SOURCES})
matlab_add_mex(NAME freenect2_matlab ${SOURCES} OUTPUT_NAME freenect2_matlab)
#SET_TARGET_PROPERTIES(freenect2_matlab PROPERTIES
#  CXX_VISIBILITY_PRESET hidden
#  VISIBILITY_INLINES_HIDDEN 1
#  VERSION ${PROJECT_VER}
#  SOVERSION ${PROJECT_APIVER}
#  SUFFIX ".mexw64"
#  LINK_FLAGS "/export:mexFunction"
#)

IF(MSVC AND NOT BUILD_SHARED_LIBS)
  # MSVC creates freenect2.lib for both dynamic and static by default
  set_target_properties(freenect2_matlab PROPERTIES SUFFIX "static.lib")
ENDIF()
STRING(REPLACE ";" "\n " LIBRARIES_STRING "${LIBRARIES}")
MESSAGE(STATUS "Linking with these libraries: \n ${LIBRARIES_STRING}")
TARGET_LINK_LIBRARIES(freenect2_matlab ${LIBRARIES})

INSTALL(TARGETS freenect2_matlab DESTINATION lib RUNTIME DESTINATION bin)
INSTALL(DIRECTORY "${MY_DIR}/include/" DESTINATION include PATTERN "*.in" EXCLUDE)