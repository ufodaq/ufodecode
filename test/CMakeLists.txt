# --- Build test executable -------------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/src 
)

add_executable(ipedec ipedec.c)
target_link_libraries(ipedec ufodecode)

install(TARGETS ipedec DESTINATION ${BIN_INSTALL_DIR})
