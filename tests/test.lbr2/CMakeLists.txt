add_definitions(-DUSE_FORTRAN)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

IF(NOT USE_ADOLC)
add_executable(test.lbr2 testLBR2.cpp)
target_link_libraries(test.lbr2 mbslib model.lbr2)
ENDIF()

