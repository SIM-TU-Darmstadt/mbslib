prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: MBSlib
Description: MBSlib - Multibody Systems Library
URL: http://www.sim.informatik.tu-darmstadt.de/mbslib/
Version: @MBSLIB_VERSION@
Requires: @EIGENVERSION@
Conflicts:
Libs: -L${libdir} -lmbslib -Wl,-rpath ${libdir}
Libs.private:
Cflags: -I${includedir}

