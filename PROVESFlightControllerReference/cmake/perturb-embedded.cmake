####
# perturb-embedded.cmake
#
# Include from project.cmake BEFORE the Gnc subdirectory:
#     include("${CMAKE_CURRENT_LIST_DIR}/cmake/perturb-embedded.cmake")
#
# Defines the `perturb` target that Gnc/Environment/OrbitPropagator
# depends on.
#
# perturb (gunvirranu/perturb) is NOT header-only -- Vallado's SGP4 is a
# compiled translation unit -- so unlike Eigen this produces a real
# static library.
#
# Same two-location search as eigen-embedded.cmake: the submodule may
# live in the F Prime project or one level up in an enclosing workspace.
#   cmake -DPERTURB_SOURCE_DIR=/abs/path/to/perturb   to override.
####

# NOT "NOT DEFINED": a failed configure used to write an empty value into
# the cache, after which this guard was false forever and the search was
# skipped -- so fixing the submodule and re-running gave the same
# "not found" error until you purged the build directory. Testing for an
# empty value catches both the unset and the poisoned case.
if (NOT PERTURB_SOURCE_DIR)
    foreach (_candidate
             "${CMAKE_CURRENT_LIST_DIR}/../lib/perturb"      # project-local
             "${CMAKE_CURRENT_LIST_DIR}/../../lib/perturb")  # workspace parent
        if (EXISTS "${_candidate}/include/perturb/perturb.hpp")
            get_filename_component(PERTURB_SOURCE_DIR "${_candidate}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

if (NOT PERTURB_SOURCE_DIR OR NOT EXISTS "${PERTURB_SOURCE_DIR}/include/perturb/perturb.hpp")
    # Clear the cache entry before failing, so the next configure searches
    # again instead of replaying this error.
    unset(PERTURB_SOURCE_DIR CACHE)
    message(FATAL_ERROR
        "perturb not found. Searched:\n"
        "  ${CMAKE_CURRENT_LIST_DIR}/../lib/perturb\n"
        "  ${CMAKE_CURRENT_LIST_DIR}/../../lib/perturb\n"
        "Add it:  git submodule add https://github.com/gunvirranu/perturb.git lib/perturb\n"
        "Or set:  -DPERTURB_SOURCE_DIR=/abs/path/to/perturb")
endif()

# Cached only after validation, so only a good value is ever persisted.
set(PERTURB_SOURCE_DIR "${PERTURB_SOURCE_DIR}" CACHE PATH "Path to the perturb source tree (the dir containing include/perturb/)" FORCE)

message(STATUS "GNC: perturb at ${PERTURB_SOURCE_DIR}")

# Build the sources directly rather than add_subdirectory()ing perturb's
# own CMakeLists.
#
# Deliberate: perturb's CMake defines its own options, install rules and
# test targets, and dropping that into an F Prime + Zephyr build brings
# in machinery we do not want and cannot easily control. Compiling three
# files ourselves is less code than fighting it, and it puts
# PERTURB_DISABLE_IO under our control (see below).
file(GLOB PERTURB_SOURCES "${PERTURB_SOURCE_DIR}/src/*.cpp")
if (NOT PERTURB_SOURCES)
    message(FATAL_ERROR "No sources in ${PERTURB_SOURCE_DIR}/src -- "
                        "is the submodule checked out?")
endif()

add_library(perturb STATIC ${PERTURB_SOURCES})

target_include_directories(perturb SYSTEM PUBLIC "${PERTURB_SOURCE_DIR}/include")

# perturb is C++11; F Prime or Zephyr may set a higher standard globally,
# which is fine. Stating the floor documents the requirement.
target_compile_features(perturb PUBLIC cxx_std_11)

# PERTURB_DISABLE_IO strips the sscanf-based TLE parser, which is the
# single largest flash contributor in the library.
#
# OFF by default because LOAD_TLE needs it: uplinking a raw TLE is a real
# operational convenience and giving it up should be a deliberate choice,
# not a default. Turn it ON only if flash forces it -- and then parse the
# TLE on the ground and uplink a pre-parsed element set instead.
option(PERTURB_DISABLE_IO "Strip perturb's TLE string parser to save flash" OFF)
if (PERTURB_DISABLE_IO)
    target_compile_definitions(perturb PUBLIC PERTURB_DISABLE_IO)
    message(STATUS "GNC: perturb TLE string parsing DISABLED -- LOAD_TLE will not work")
endif()

# Vendored third-party code: do not let its warnings drown out yours.
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(perturb PRIVATE -w)
endif()

# ---------------------------------------------------------------------
# Zephyr flags.
#
# This is a plain add_library(), not a zephyr_library(), so it does NOT
# inherit Zephyr's include paths, autoconf.h, or toolchain settings the
# way F Prime modules do. On a Zephyr build that shows up as perturb
# failing to find <cmath> / <cstring> while every F Prime module
# compiles fine -- which reads like a perturb problem and is not.
#
# zephyr_interface is the target that carries those flags. Linking it
# PRIVATE is the standard way to build an out-of-tree static library
# inside a Zephyr build.
#
# Guarded, because the same helper is used for the host BUILD_TESTING
# configuration where the parent CMakeLists skips find_package(Zephyr)
# and this target does not exist.
# ---------------------------------------------------------------------
if (TARGET zephyr_interface)
    target_link_libraries(perturb PRIVATE zephyr_interface)
    message(STATUS "GNC: perturb linked against zephyr_interface")
endif()
