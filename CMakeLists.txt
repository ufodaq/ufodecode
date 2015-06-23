cmake_minimum_required(VERSION 2.8)

set(LIBUFODECODE_API_VERSION "0.3")
set(LIBUFODECODE_ABI_VERSION "0.3")
set(LIBUFODECODE_ABI_MAJOR_VERSION "0")
set(LIBUFODECODE_VERSION "0.3.0")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

include(ConfigurePaths)
configure_paths(LIBUFODECODE)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ufodecode.pc.in"
               "${CMAKE_CURRENT_BINARY_DIR}/ufodecode.pc"
               @ONLY IMMEDIATE)

install(FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/ufodecode.pc 
    DESTINATION ${LIB_INSTALL_DIR}/pkgconfig)

add_subdirectory(src)
add_subdirectory(doc)
add_subdirectory(test)

include(CPack)
