
if(USE_ADOLC)
add_executable(exp_06_est_mbs exp_06_est_mbs.cpp)
target_link_libraries(exp_06_est_mbs mbslib)
 target_link_libraries(exp_06_est_mbs ${ADOLC_LIBRARIES})
endif(USE_ADOLC)