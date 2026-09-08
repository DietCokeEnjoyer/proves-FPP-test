####
# Gnc/MagneticFieldModel
#
# XYZgeomag is header-only and vendored under lib/, so there is nothing
# extra to compile or link -- only an include path.
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.hpp"
    DEPENDS
        PROVESFlightControllerReference_Gnc_Types
        PROVESFlightControllerReference_Gnc_AstroLib
)

# Host-only integration test for the units / frame / time seam.
if (BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(magchain_test
        "${CMAKE_CURRENT_LIST_DIR}/test/MagChainTest.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../../AstroLib/AstroLib.cpp"
    )
    target_include_directories(magchain_test PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../../.."
        "${CMAKE_CURRENT_LIST_DIR}/../../AstroLib")
    add_test(NAME magchain COMMAND magchain_test)
endif()
