####
# Adcs/AttitudeDetermination
#
# Uses the keyword form of register_fprime_module (F Prime v3.4+ / v4.x).
# Most link dependencies are inferred from the FPP model dependency
# graph; Gnc_Types is listed explicitly because the component's ports
# use Gnc.VectorSample, and eigen_embedded because Eigen is not an
# F Prime module.
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/AttitudeDetermination.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/TriadSolver.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/TriadTypes.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/AttitudeDetermination.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/AttitudeDetermination.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/TriadSolver.hpp"
    DEPENDS
        Gnc_Types
        eigen_embedded
)

# Pure-math unit test, host builds only. Needs no F Prime, no hardware.
# NOTE: do NOT inherit EIGEN_NO_DEBUG here -- you want Eigen's asserts
# active in the test even though flight code compiles them out.
if (BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(triad_solver_test
        "${CMAKE_CURRENT_LIST_DIR}/test/TriadSolverTest.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/TriadSolver.cpp"
    )
    target_include_directories(triad_solver_test PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
    target_link_libraries(triad_solver_test PRIVATE eigen_embedded)
    add_test(NAME triad_solver COMMAND triad_solver_test)
endif()
