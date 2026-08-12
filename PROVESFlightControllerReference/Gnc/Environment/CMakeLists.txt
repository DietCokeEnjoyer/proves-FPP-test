# Environment models, in dependency order.
#
# OrbitPropagator is the source: it owns the clock, the time scales,
# GMST and SGP4. The other two consume its OrbitState and must never
# acquire their own time.
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/OrbitPropagator/")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/SolarEphemeris/")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel/")
