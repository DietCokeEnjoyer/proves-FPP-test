####
# Gnc/Types
#
# Shared GNC data types and ports. Every other module in the chain
# depends on this one and nothing else depends on them, so it is the
# only place a type used across a component boundary may be defined.
#
# Fw/Time is an explicit dependency because Gnc.VectorSample and
# Gnc.OrbitState carry an Fw.Time member.
####
register_fprime_module(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/GncTypes.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/GncConvert.hpp"
    DEPENDS
        Fw_Time
        PROVESFlightControllerReference_Gnc_AstroLib   # GncConvert.hpp converts Astro types
)
