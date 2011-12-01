cmake_minimum_required(VERSION 2.8)
set(TARNAME "libipe")

set(LIBIPE_API_VERSION "0.1.0")
set(LIBIPE_ABI_VERSION "0.1.0")
set(LIBIPE_ABI_MAJOR_VERSION "0")

set(PACKAGE_VERSION "0.1.0")
set(PACKAGE_NAME "${TARNAME}")
set(PACKAGE_TARNAME "${TARNAME}")
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "http://ufo.kit.edu/ufo/newticket")

if(NOT DEFINED BIN_INSTALL_DIR)
    set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
endif(NOT DEFINED BIN_INSTALL_DIR)

if(NOT DEFINED LIB_INSTALL_DIR)
    set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif(NOT DEFINED LIB_INSTALL_DIR)

if(NOT DEFINED INCLUDE_INSTALL_DIR)
    set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/${PACKAGE_TARNAME}")
endif(NOT DEFINED INCLUDE_INSTALL_DIR)

if(NOT DEFINED LOCALE_INSTALL_DIR)
    set(LOCALE_INSTALL_DIR "${DATA_INSTALL_DIR}/locale/")
endif(NOT DEFINED LOCALE_INSTALL_DIR)


# --- Build library and install ---------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions("--std=c99 -Wall -O2")

add_library(ipe SHARED src/libipe.c)

set_target_properties(ipe PROPERTIES
    VERSION ${LIBIPE_ABI_VERSION}
    SOVERSION ${LIBIPE_ABI_MAJOR_VERSION}
)

install(TARGETS ipe
    LIBRARY DESTINATION lib${LIB_SUFFIX}
)

install(FILES
    src/libipe.h 
    DESTINATION include
)

configure_file(ipe.pc.in ${CMAKE_CURRENT_BINARY_DIR}/ipe.pc)

if ("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
    set(DEBUG "1")
endif()
configure_file(src/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

install(FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/ipe.pc 
    DESTINATION ${LIB_INSTALL_DIR}/pkgconfig)


# --- Build test executable -------------------------------------------------
add_executable(ipedec test/ipedec.c)
target_link_libraries(ipedec ipe)

install(TARGETS ipedec DESTINATION ${BIN_INSTALL_DIR})
