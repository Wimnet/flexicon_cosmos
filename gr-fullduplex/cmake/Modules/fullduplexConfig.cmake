INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_FULLDUPLEX fullduplex)

FIND_PATH(
    FULLDUPLEX_INCLUDE_DIRS
    NAMES fullduplex/api.h
    HINTS $ENV{FULLDUPLEX_DIR}/include
        ${PC_FULLDUPLEX_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    FULLDUPLEX_LIBRARIES
    NAMES gnuradio-fullduplex
    HINTS $ENV{FULLDUPLEX_DIR}/lib
        ${PC_FULLDUPLEX_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/fullduplexTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FULLDUPLEX DEFAULT_MSG FULLDUPLEX_LIBRARIES FULLDUPLEX_INCLUDE_DIRS)
MARK_AS_ADVANCED(FULLDUPLEX_LIBRARIES FULLDUPLEX_INCLUDE_DIRS)
