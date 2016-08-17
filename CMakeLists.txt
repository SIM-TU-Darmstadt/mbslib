CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMAKE_BUILD_TYPE: Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel" FORCE)
ENDIF()

PROJECT (mbslib)

SET ( MBSLIB_VERSION_MAJOR 0 )
SET ( MBSLIB_VERSION_MINOR 1 )
SET ( MBSLIB_VERSION_PATCH 0 )
SET ( MBSLIB_VERSION ${MBSLIB_VERSION_MAJOR}.${MBSLIB_VERSION_MINOR}.${MBSLIB_VERSION_PATCH} )
SET ( MBSLIB_SO_VERSION ${MBSLIB_VERSION_MAJOR}.${MBSLIB_VERSION_MINOR}.${MBSLIB_VERSION_PATCH} )
SET ( PROJECT_VERSION ${MBSLIB_VERSION} )

INCLUDE(GNUInstallDirs)
include(ExternalProject)

if(NOT DEFINED MBSLIB_RUNTIME_INSTALL_DIRECTORY)
    set(MBSLIB_RUNTIME_INSTALL_DIRECTORY ${CMAKE_INSTALL_BINDIR} CACHE INTERNAL "" FORCE)
endif()

if(NOT DEFINED MBSLIB_ARCHIVE_INSTALL_DIRECTORY)
    set(MBSLIB_ARCHIVE_INSTALL_DIRECTORY ${CMAKE_INSTALL_LIBDIR} CACHE INTERNAL "" FORCE)
endif()

if(NOT DEFINED MBSLIB_LIBRARY_INSTALL_DIRECTORY)
    set(MBSLIB_LIBRARY_INSTALL_DIRECTORY ${CMAKE_INSTALL_LIBDIR} CACHE INTERNAL "" FORCE)
endif()

add_definitions(-fPIC)

include(cmake/Common.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(USE_EIGEN2 "If ON it uses Eigen2 instead of Eigen3" OFF)
option(USE_URDF "activate for experimental URDF support" OFF)
option(USE_ADOLC "Use adolc" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(USE_MBSLIB_OUTPUT_DIR "Build mbslib to build/output" ON)
option(BUILD_EXTENSIONS "Build extensions" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_APPLICATION "Build applications" ON)
option(BUILD_TESTING "Build tests" ON)

configure_file(src/mbslib/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/mbslib/config.hpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

SET(MBSLIB_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
SET(MBS_TEST_DIR "${MBSLIB_BASE_DIR}/dependencies/mbstestlib/tests")
SET(DEPENDENCY_DIR "${MBSLIB_BASE_DIR}/dependencies")

SET(DEPS_INSTALL_PATH ${DEPENDENCY_DIR}/install)
IF(NOT EXISTS ${DEPS_INSTALL_PATH})
  execute_process(COMMAND mkdir -p ${DEPS_INSTALL_PATH})
ENDIF()

SET(RESOURCE_DIR "${MBSLIB_BASE_DIR}/resources")
if(USE_MBSLIB_OUTPUT_DIR)
    SET(OUTPUT_DIR "${MBSLIB_BASE_DIR}/build/output" CACHE PATH "MBSLIB_OUTPUT_DIR")
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}/bin")
    SET(EXECUTABLE_OUTPUT_PATH  "${OUTPUT_DIR}/bin")
    SET(LIBRARY_OUTPUT_PATH "${OUTPUT_DIR}/lib")

    SET(RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}/bin")
    SET(ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}/lib")
    SET(LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}/lib")

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib-debug)
        set(LIB_INSTALL_DIR ${OUTPUT_DIR}/lib-debug)
        set(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIR}/bin-dbg)
    else()
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib)
        set(LIB_INSTALL_DIR ${OUTPUT_DIR}/lib)
        set(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIR}/bin)
    endif()
endif()

include(cmake/FindPkgConfig.cmake)
SET(EXTERNAL_EIGEN OFF)
if(USE_EIGEN2)
    SET(EIGENVERSION eigen2)
    pkg_check_modules(EIGEN2 eigen2)
    IF(EIGEN2_FOUND)
        SET(MBSLIB_EIGEN_PATH ${EIGEN2_INCLUDE_DIRS} CACHE PATH "mbslib eigen path" FORCE)
    ELSE(EIGEN2_FOUND)
        SET(EXTERNAL_EIGEN ON)
        ExternalProject_Add(eigen2
                PREFIX ${DEPENDENCY_DIR}/eigen2
                HG_REPOSITORY https://bitbucket.org/eigen/eigen/
                HG_TAG 2.0.17
                CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${DEPS_INSTALL_PATH}
                INSTALL_DIR ${DEPS_INSTALL_PATH})

        SET(MBSLIB_EIGEN_PATH "${DEPS_INSTALL_PATH}/include/eigen2" CACHE PATH "mbslib eigen path" FORCE)
    ENDIF(EIGEN2_FOUND)
else(USE_EIGEN2)
    SET(EIGENVERSION eigen3)
    pkg_check_modules(EIGEN3 eigen3)
    IF(EIGEN3_FOUND)
        SET(MBSLIB_EIGEN_PATH ${EIGEN3_INCLUDE_DIRS} CACHE PATH "mbslib eigen path" FORCE)
    ELSE(EIGEN3_FOUND)
        SET(EXTERNAL_EIGEN ON)
        ExternalProject_Add(eigen3
                PREFIX ${DEPENDENCY_DIR}/eigen3
                HG_REPOSITORY https://bitbucket.org/eigen/eigen/
                HG_TAG 3.2.6
                CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${DEPS_INSTALL_PATH}
                INSTALL_DIR ${DEPS_INSTALL_PATH}
                STEP_TARGETS eigen)

        SET(MBSLIB_EIGEN_PATH "${DEPS_INSTALL_PATH}/include/eigen3" CACHE PATH "mbslib eigen path" FORCE)
    ENDIF(EIGEN3_FOUND)
endif(USE_EIGEN2)

message("Creating mbslib")
message("Directories: ")
message("  MBSLIB_BASE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message("  MBS_TEST_DIR = ${MBS_TEST_DIR}")
message("  MBSLIB_EIGEN_DIR = ${MBSLIB_EIGEN_PATH}")
message("  DEPENDENCY_DIR = ${DEPENDENCY_DIR}")
message("  OUTPUT_DIR = ${OUTPUT_DIR}")
message("  CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")
message("  CMAKE_RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message("  EXECUTABLE_OUTPUT_PATH = ${EXECUTABLE_OUTPUT_PATH}")
message("  LIBRARY_OUTPUT_PATH = ${LIBRARY_OUTPUT_PATH}")
message("  CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")

include(cmake/Compiler.cmake)

##Eigen
SET(MBSLIB_INCLUDE_DIR
    ${MBSLIB_EIGEN_PATH}
    ${MBSLIB_BASE_DIR}/dependencies
    ${MBSLIB_BASE_DIR}/include
    ${MBSLIB_BASE_DIR}/src
    ${MBSLIB_BASE_DIR}/src/include
    ${CMAKE_CURRENT_BINARY_DIR}/include
        CACHE INTERNAL "mbslib include directories" FORCE)

include_directories(${MBSLIB_INCLUDE_DIR})
IF(WIN32)
  #needed if using windows.h and eigen.
  add_definitions(-DNOMINMAX)
ENDIF(WIN32)

if(USE_ADOLC)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ADOLC_Hints.txt")
  include_directories(${ADOLC_INCLUDE_DIR})
  set(MBSLIB_INCLUDE_DIR
    ${MBSLIB_INCLUDE_DIR}
    ${ADOLC_INCLUDE_DIR}
        CACHE INTERNAL "mbslib include directories" FORCE)
  add_definitions(-DUSE_ADOLC)
else(USE_ADOLC)
  set(ADOLC_LIBRARIES "")
endif(USE_ADOLC)

#  add_subdirectory(dependencies)

if(BUILD_TESTING)
  message("including CTest")
  include("CTestConfig.cmake")
  include(CTest)
  add_definitions(-DUSE_TESTS)
  enable_testing()
endif(BUILD_TESTING)
# add system preprocessor directives
if(APPLE)
 add_definitions(-DAPPLE)
endif(APPLE)
if(WIN32)
 add_definitions(-DWIN32)
endif(WIN32)
if(UNIX)
 add_definitions(-DUNIX)
endif(UNIX)

add_subdirectory(src)

IF(EXTERNAL_EIGEN)
    if(USE_EIGEN2)
        add_dependencies(mbslib eigen2)
    else()
        add_dependencies(mbslib eigen3)
    endif()
ENDIF()

if(BUILD_EXAMPLES)
message("adding examples")
add_subdirectory(examples)
endif()

if(BUILD_EXTENSIONS)
message("adding extensions")
include_directories(extensions)
add_subdirectory(extensions)
endif()

if(BUILD_TESTING)
message("adding tests")
add_subdirectory(tests)
endif()

INSTALL(DIRECTORY include/mbslib DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/mbslib/config.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mbslib)

# pkg-config configuration
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/mbslib.pc.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mbslib.pc @ONLY
)
INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/mbslib.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

# CPack
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MBSlib - Multi body systems library")
SET(CPACK_PACKAGE_DESCRIPTION "MBSlib - Multi body systems library")
SET(CPACK_PACKAGE_CONTACT "mbslib@sim.tu-darmstadt.de")
SET(CPACK_PACKAGE_VENDOR "FG SIM, TU Darmstadt")
SET(CPACK_PACKAGE_NAME "MBSlib")
SET(CPACK_PACKAGE_VERSION_MAJOR ${MBSLIB_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${MBSLIB_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${MBSLIB_VERSION_PATCH})
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
if(UNIX)
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "MBSlib-Team <mbslib@sim.tu-darmstadt.de>")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev (>= 1.46)")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CPACK_PACKAGE ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
SET(CPACK_PACKAGE_FILE_NAME "mbslib-${CMAKE_LIBRARY_ARCHITECTURE}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
include(CPack)
endif()
