####
# Gnc/MagneticFieldModel
#
# XYZgeomag is header-only, only needs an include path. It is included
# by WmmModel.cpp and nowhere else, so the component never sees it.
#
# WmmModel is the framework-free half: the range gates, the TEME/ECEF
# rotations and the km/m and Tesla/nT conversions. It builds standalone
# for host testing, exactly like AstroLib and TriadSolver.
####
register_fprime_module(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/WmmModel.cpp"
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/MagneticFieldModel.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/WmmModel.hpp"
    DEPENDS
        PROVESFlightControllerReference_Gnc_Types
        PROVESFlightControllerReference_Gnc_AstroLib
)

# Host-only integration test for the units / frame / time seam.
#
# WmmModel.cpp is linked in so the test drives the real evaluateField()
# rather than a copy of it. Two include roots are needed: the repo root,
# for the PROVESFlightControllerReference/... paths the module uses, and
# PROVESFlightControllerReference itself, because AstroLib.cpp includes
# its own header as "Gnc/AstroLib/AstroLib.hpp".
if (BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(magchain_test
        "${CMAKE_CURRENT_LIST_DIR}/test/MagChainTest.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/WmmModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../../AstroLib/AstroLib.cpp"
    )
    target_include_directories(magchain_test PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/../../../.."
        "${CMAKE_CURRENT_LIST_DIR}/../../..")
    add_test(NAME magchain COMMAND magchain_test)
endif()
