####
# Gnc
#
# Guidance, Navigation and Control. Registered top-to-bottom in
# DEPENDENCY ORDER, which is also the order a new reader should read it:
#
#   Types        shared FPP types. Everything depends on this; it
#                depends on nothing but Fw. Must be registered before
#                any consumer -- including sensor drivers elsewhere in
#                the project that emit Gnc.VectorSample.
#   AstroLib     framework-free astrodynamics kernel. No F Prime, no
#                Zephyr, host-testable standalone.
#   Environment  models of the world: where the spacecraft is, where
#                the Sun is, what the field is. No control authority.
#   Adcs         attitude determination and control. Consumes
#                Environment, never the reverse.
#
# The one-way arrow Adcs -> Environment -> AstroLib -> Types is the
# whole reason for the nesting: a magnetic field model has no business
# knowing about attitude, and expressing that as sibling directories
# would leave the constraint implicit in project.cmake where it is easy
# to violate by accident.
####
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Types/")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/AstroLib/")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Environment/")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Adcs/")
