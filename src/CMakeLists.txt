# --- Look for SSE support --------------------------------------------------
include(CheckCXXSourceRuns)
set(SSE_FLAGS)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_REQUIRED_FLAGS "-msse")
    check_cxx_source_runs("
        #include <xmmintrin.h>
        int main()
        {
            __m128 a, b;
            float vals[4] = {0};
            a = _mm_loadu_ps(vals);
            b = a;
            b = _mm_add_ps(a,b);
            _mm_storeu_ps(vals,b);
            return 0;
        }"
    SSE_AVAILABLE)

    set(CMAKE_REQUIRED_FLAGS)

    if (SSE_AVAILABLE)
        option(HAVE_SSE "Use SSE extensions" ON)
        set(SSE_FLAGS "-msse")
    endif()
endif()

# --- Build library and install ---------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions("--std=c99 -Wall -O2 ${SSE_FLAGS}")

add_library(ufodecode SHARED ufodecode.c)

set_target_properties(ufodecode PROPERTIES
    VERSION ${LIBUFODECODE_ABI_VERSION}
    SOVERSION ${LIBUFODECODE_ABI_MAJOR_VERSION}
)

install(TARGETS ufodecode
    LIBRARY DESTINATION lib${LIB_SUFFIX}
)

install(FILES
    ufodecode.h
    DESTINATION include
)

if ("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
    set(DEBUG "1")
endif()

configure_file(config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

