add_executable(2DOF_pendel_simple 2DOF_Pendel_simple.cpp)
target_link_libraries(2DOF_pendel_simple mbslib)

IF(USE_ADOLC)
target_link_libraries(2DOF_pendel_simple ${ADOLC_LIBRARIES})
ENDIF(USE_ADOLC)
