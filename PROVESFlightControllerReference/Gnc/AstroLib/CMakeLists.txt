####
# Gnc/AstroLib
#
# F Prime independent astrodynamics kernel. Registered as an F Prime
# module so it can be a dependency for F Prime components.
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/AstroLib.cpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/AstroLib.hpp"
)

# build tests
if (BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(astrolib_test
        "${CMAKE_CURRENT_LIST_DIR}/test/AstroLibTest.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/AstroLib.cpp"
    )
    target_include_directories(astrolib_test PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../.."
        "${CMAKE_CURRENT_LIST_DIR}")
    add_test(NAME astrolib COMMAND astrolib_test)
endif()
