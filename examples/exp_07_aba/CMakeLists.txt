add_executable(exp_07_aba exp_07_aba.cpp)
target_link_libraries(exp_07_aba mbslib)
if(USE_ADOLC)
 target_link_libraries(exp_07_aba ${ADOLC_LIBRARIES})
endif(USE_ADOLC)