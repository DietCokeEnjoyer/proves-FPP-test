register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.hpp"
    DEPENDS
        PROVESFlightControllerReference_Gnc_Types
        PROVESFlightControllerReference_Gnc_AstroLib
        perturb          # CMake target exported by gunvirranu/perturb
)
