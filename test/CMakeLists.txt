# --- Build test executable -------------------------------------------------
include(CheckIncludeFile)

check_include_file("sys/time.h" HAVE_SYS_TIME_H)

if (NOT HAVE_SYS_TIME_H)
    message(FATAL_ERROR "sys/time.h not found")
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/src 
)

add_executable(ipedec ipedec.c timer.c)

target_link_libraries(ipedec ufodecode)

install(TARGETS ipedec DESTINATION ${LIBUFODECODE_BINDIR})
