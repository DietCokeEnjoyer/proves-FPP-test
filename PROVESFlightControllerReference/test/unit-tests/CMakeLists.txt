cmake_minimum_required(VERSION 3.20)
project(proves_unit_tests CXX)

enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/fprime/googletest googletest-build)

# --- Helper Libraries ---

# DetumbleManager Magnetorquer
add_library(detumble_manager_magnetorquer STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../PROVESFlightControllerReference/Components/DetumbleManager/Magnetorquer.cpp
)
target_include_directories(detumble_manager_magnetorquer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
)

# DetumbleManager StrategySelector
add_library(detumble_manager_strategy_selector STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../PROVESFlightControllerReference/Components/DetumbleManager/StrategySelector.cpp
)
target_include_directories(detumble_manager_strategy_selector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
)

# RtcManager RtcHelper
add_library(rtc_manager_rtc_helper STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../PROVESFlightControllerReference/Components/Drv/RtcManager/RtcHelper.cpp
)
target_include_directories(rtc_manager_rtc_helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
)

# DetumbleManager BDot
add_library(detumble_manager_bdot STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../PROVESFlightControllerReference/Components/DetumbleManager/BDot.cpp
)
target_include_directories(detumble_manager_bdot PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
)

# --- Auto-discover and build tests ---

file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")

foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)

    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name}
        gtest_main
        detumble_manager_magnetorquer
        detumble_manager_strategy_selector
        detumble_manager_bdot
        rtc_manager_rtc_helper
    )

    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# ======================================================================
# Additions for PROVESFlightControllerReference/test/unit-tests/CMakeLists.txt
#
# Append these to the END of that file, after the existing
# `foreach(test_src ${TEST_SOURCES})` block.
#
# WHY HERE AND NOT IN Gnc/*/CMakeLists.txt
# ----------------------------------------
# This is where your host tests actually live: a standalone CMake
# project with its own project() and enable_testing(), driven by
# `make test-unit`, which never goes through the workspace CMakeLists.
#
# The `add_executable`/`add_test` blocks inside Gnc/*/CMakeLists.txt are
# inert in this workspace -- they are guarded by
# `BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING`, and neither the Zephyr
# build (cross-compiling) nor this project (never adds Gnc/) satisfies
# both. They are kept so the Gnc tree stays liftable into a deployment
# that does wire them up. If that bothers you, delete them; nothing here
# depends on them.
#
# WHY THESE ARE NOT GoogleTest
# ----------------------------
# The three GNC tests are hand-written binaries with their own main(),
# printing a readable table and returning non-zero on failure (verified:
# a forced failure exits 1). ctest keys off the exit code, so they
# integrate fine without gtest -- and they must NOT link gtest_main,
# which would give two main() symbols.
#
# Their filenames deliberately do not match the `test_*.cpp` glob above,
# so the existing foreach will not pick them up.
#
# The diagnostic output is the point: the TRIAD error-vs-separation
# sweep and the F32-vs-F64 time precision numbers are things you read,
# not just assertions that pass.
# ======================================================================

set(GNC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Gnc")
set(REPO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(WORKSPACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# --- AstroLib: 27 truth-data checks, no dependencies at all -----------
add_executable(gnc_astrolib_test
    "${GNC_DIR}/AstroLib/test/AstroLibTest.cpp"
    "${GNC_DIR}/AstroLib/AstroLib.cpp"
)
target_include_directories(gnc_astrolib_test PRIVATE
    "${REPO_DIR}"
    "${GNC_DIR}/AstroLib"
)
add_test(NAME gnc_astrolib COMMAND gnc_astrolib_test)

# --- Magnetic chain: units, frame round trip, time precision ----------
# Catches the km/m boundary and the F32 decimal-year quantization, the
# two failures that produce plausible-looking wrong numbers.
add_executable(gnc_magchain_test
    "${GNC_DIR}/Environment/MagneticFieldModel/test/MagChainTest.cpp"
    "${GNC_DIR}/AstroLib/AstroLib.cpp"
)
target_include_directories(gnc_magchain_test PRIVATE
    "${REPO_DIR}"
    "${GNC_DIR}/AstroLib"
)
add_test(NAME gnc_magchain COMMAND gnc_magchain_test)

# --- TRIAD solver: pure math, needs Eigen only ------------------------
# Eigen is found the same way eigen-embedded.cmake finds it, but this is
# a standalone project that never includes that helper, so the search is
# repeated here rather than assuming a path.
if (NOT DEFINED EIGEN_SOURCE_DIR)
    foreach (_candidate "${REPO_DIR}/lib/eigen" "${WORKSPACE_DIR}/lib/eigen")
        if (EXISTS "${_candidate}/Eigen/Core")
            get_filename_component(EIGEN_SOURCE_DIR "${_candidate}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

if (EIGEN_SOURCE_DIR AND EXISTS "${EIGEN_SOURCE_DIR}/Eigen/Core")
    add_executable(gnc_triad_test
        "${GNC_DIR}/Adcs/AttitudeDetermination/test/TriadSolverTest.cpp"
        "${GNC_DIR}/Adcs/AttitudeDetermination/TriadSolver.cpp"
    )
    target_include_directories(gnc_triad_test PRIVATE
        "${GNC_DIR}/Adcs/AttitudeDetermination"
    )
    target_include_directories(gnc_triad_test SYSTEM PRIVATE "${EIGEN_SOURCE_DIR}")

    # NOTE: deliberately NOT setting EIGEN_NO_DEBUG here. Flight code~
    # compiles Eigen's asserts out (see cmake/eigen-embedded.cmake);
    # the test wants them active.
    target_compile_definitions(gnc_triad_test PRIVATE
        EIGEN_MPL2_ONLY
        EIGEN_DONT_VECTORIZE
    )
    add_test(NAME gnc_triad COMMAND gnc_triad_test)
    message(STATUS "GNC tests: Eigen at ${EIGEN_SOURCE_DIR}")
else()
    message(WARNING
        "GNC: Eigen not found, skipping gnc_triad_test. "
        "git submodule update --init --recursive")
endif()
