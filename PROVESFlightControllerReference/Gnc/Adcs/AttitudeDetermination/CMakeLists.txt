####
# Adcs/AttitudeDetermination
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
        PROVESFlightControllerReference_Gnc_Types
        eigen_embedded
)

# Unit tests
if (BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(triad_solver_test
        "${CMAKE_CURRENT_LIST_DIR}/test/TriadSolverTest.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/TriadSolver.cpp"
    )
    target_include_directories(triad_solver_test PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
    target_link_libraries(triad_solver_test PRIVATE eigen_embedded)
    add_test(NAME triad_solver COMMAND triad_solver_test)
endif()
