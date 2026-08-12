# Attitude determination and control.
#
# Future home for BDotController, MagnetorquerAllocation, ModeManager
# and an MEKF. When this directory reaches a dozen components it will
# have earned promotion to a top-level sibling of Gnc on navigability
# grounds alone -- at which point the dependency direction is still
# one-way and nothing about the code has to change.
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/AttitudeDetermination/")
