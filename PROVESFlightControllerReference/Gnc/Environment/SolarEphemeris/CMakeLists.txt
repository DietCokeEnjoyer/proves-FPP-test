####
# Gnc/SolarEphemeris
# 
# SunModel.* is the solar geometry math,
# SolarEphemeris.* is the Fprime wrapper.
#
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SunModel.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/SunModel.hpp"
    DEPENDS
        PROVESFlightControllerReference_Gnc_Types
        PROVESFlightControllerReference_Gnc_AstroLib
)
