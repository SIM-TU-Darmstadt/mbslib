if(${Boost_FOUND})

set(testName "1DOF_Pendulum")

add_executable(
    ${testName}
    1DOF_Pendulum.cpp
)

target_link_libraries(${testName} 
	mbslib 
	${ADOLC_LIBRARIES}
	${BOOST_LIBRARIES}
)

endif()
