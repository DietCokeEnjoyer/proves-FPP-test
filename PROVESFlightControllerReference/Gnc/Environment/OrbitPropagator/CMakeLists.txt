register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.hpp"
    DEPENDS
        Gnc_Types
        Gnc_AstroLib
        perturb          # CMake target exported by gunvirranu/perturb
)
