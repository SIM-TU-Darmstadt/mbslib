if(${Boost_FOUND})

set(testName "2DOF_CompoundPendulumOnTrolley")

add_executable(
    ${testName}
    2DOF_CompoundPendulumOnTrolley.cpp
)

target_link_libraries(${testName} 
	mbslib 
	${ADOLC_LIBRARIES}
	${BOOST_LIBRARIES}
)

endif()
