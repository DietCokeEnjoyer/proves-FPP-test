####
# Gnc/OrbitPropagator
# 
# Sgp4Propagator.* holds the time scales and SGP4 evaluations.
# OrbitPropagator.* is the FPrime wrapper.
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Sgp4Propagator.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/Sgp4Propagator.hpp"
    DEPENDS
        PROVESFlightControllerReference_Gnc_Types
        PROVESFlightControllerReference_Gnc_AstroLib
        perturb          # CMake target exported by gunvirranu/perturb
)
