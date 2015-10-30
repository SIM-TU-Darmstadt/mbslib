
if(USE_ADOLC)
add_executable(exp_16_kuka_springs exp_16_kuka_springs.cpp)
target_link_libraries(exp_16_kuka_springs mbslib)
 target_link_libraries(exp_16_kuka_springs ${ADOLC_LIBRARIES})
endif(USE_ADOLC)