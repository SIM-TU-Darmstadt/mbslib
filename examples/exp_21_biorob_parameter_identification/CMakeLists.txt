# Setup BioRob
IF(NOT ${USE_ADOLC})
  return()
ENDIF()

set(BIN_NAME exp_21_biorob_parameter_identification)
set(${BIN_NAME}_SRC
  biorob.h
  biorob.cpp
)
#add_executable(${BIN_NAME} ${${BIN_NAME}_SRC})
add_library(${BIN_NAME} SHARED ${${BIN_NAME}_SRC})
target_link_libraries(${BIN_NAME} mbslib)

set(EXP_LIBRARY_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/biorob.h")
set(EXP_LIBRARY_NAME ${BIN_NAME})
set(EXP_LIBRARY "${LIBRARY_OUTPUT_DIRECTORY}/lib${BIN_NAME}.so")
configure_file(example_library.m.in ${CMAKE_CURRENT_SOURCE_DIR}/example_library.m @ONLY)

