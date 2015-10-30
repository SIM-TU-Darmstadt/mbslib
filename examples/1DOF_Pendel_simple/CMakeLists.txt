add_executable(1DOF_pendel_simple 1DOF_Pendel_simple.cpp)
target_link_libraries(1DOF_pendel_simple mbslib)

IF(USE_ADOLC)
target_link_libraries(1DOF_pendel_simple ${ADOLC_LIBRARIES})
ENDIF(USE_ADOLC)
