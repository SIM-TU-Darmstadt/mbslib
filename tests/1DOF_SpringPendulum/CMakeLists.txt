if(${Boost_FOUND})

set(testName "1DOF_SpringPendulum")

add_executable(
    ${testName}
    1DOF_SpringPendulum.cpp
)

target_link_libraries(${testName} 
	mbslib 
	${ADOLC_LIBRARIES}
	${BOOST_LIBRARIES}
)

endif()
