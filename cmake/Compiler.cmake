if(CMAKE_COMPILER_IS_GNUCXX)
   ## add g++0x/g++11/g++14 flags for gcc
   ## + so that lambdas are enabled
   ## + required for the new integrators

   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 5.0 OR GCC_VERSION VERSION_EQUAL 5.0)
        message(STATUS "C++14 activated.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
        #add_definitions("-std=gnu++14")
   elseif(GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        #add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
        #add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11/14 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
endif(CMAKE_COMPILER_IS_GNUCXX)

IF (CMAKE_COMPILER_IS_GNUCC)
  # some additional flags for gcc compilers
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp -Wno-sign-compare -Wno-unknown-pragmas -Wno-comment -Wno-unused-variable " CACHE STRING "" FORCE)
  SET(CMAKE_CXX_RELEASE_FLAGS "${CMAKE_CXX_RELEASE_FLAGS} -O3 -msse2 -fopenmp  -Wno-sign-compare -Wno-unknown-pragmas -Wno-comment -Wno-unused-variable "  CACHE STRING "" FORCE)
ENDIF(CMAKE_COMPILER_IS_GNUCC)

IF(MSVC)
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ob2 /Oi /Ot /GL /bigobj")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
ENDIF(MSVC)
