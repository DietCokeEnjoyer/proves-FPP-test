# Note: no dependency on perturb. Splitting the propagator out means the
# solar ephemeris no longer links SGP4 at all.
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.hpp"
    DEPENDS
        Gnc_Types
        Gnc_AstroLib
)
