if(USE_ADOLC)
 add_executable(exp_01_aba_jac exp_01_aba_jac.cpp)
 target_link_libraries(exp_01_aba_jac
  mbslib
  ${ADOLC_LIBRARIES}
 )
endif(USE_ADOLC)
