SET(mbstestlib_SRC
	TesterTypes.h
	TesterTypes.cpp
	Tester.h
	Tester.cpp
	Adapter.hpp
	ErrorLogger.h
	ErrorLogger.cpp
	parserTools.h
	parserTools.cpp
	RobotModelParser.h
	RobotModelParser.cpp
)
add_library(mbstestlib ${mbstestlib_SRC}) 
project_folder("dependencies" mbstestlib)

SET(MBSLIB_INCLUDE_DIR
    ${MBSLIB_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
        CACHE INTERNAL "mbslib include directories" FORCE)
