if(${Boost_FOUND})

set(testName "basic_tests")

add_executable(
    ${testName}
    basic_tests.cpp
)

target_link_libraries(${testName} 
	mbslib 
	${ADOLC_LIBRARIES}
	${BOOST_LIBRARIES}
)

endif()
