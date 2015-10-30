FILE(GLOB_RECURSE source *.cpp *.c *.hpp *.h ../include/*.h ../include/*.hpp)

#adds the library
add_library(mbslib ${source})

SET_TARGET_PROPERTIES ( mbslib PROPERTIES
  VERSION ${MBSLIB_VERSION}
  SOVERSION ${MBSLIB_SO_VERSION}
)

install_mbslib_target(mbslib)

if(USE_ADOLC)
target_link_libraries (mbslib ${ADOLC_LIBRARIES})
endif(USE_ADOLC)
