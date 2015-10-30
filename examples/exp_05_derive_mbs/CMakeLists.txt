
if(USE_ADOLC)
add_executable(exp_05_derive_mbs exp_05_derive_mbs.cpp)
target_link_libraries(exp_05_derive_mbs mbslib)
 target_link_libraries(exp_05_derive_mbs ${ADOLC_LIBRARIES})
endif(USE_ADOLC)