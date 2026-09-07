####
# eigen-embedded.cmake
#
# Include this from your project.cmake BEFORE the module that uses it:
#     include("${CMAKE_CURRENT_LIST_DIR}/cmake/eigen-embedded.cmake")
#
# Eigen is header-only, so there is nothing to compile -- we just need
# an INTERFACE target carrying the include path and the compile
# definitions that make Eigen behave on a microcontroller. Rolling our
# own target rather than using Eigen's own CMake package keeps these
# definitions under our control and skips Eigen's test/doc machinery.
#
# Vendor Eigen as a submodule (recommended for flight: pinned, auditable,
# offline-buildable). Two layouts are searched, because the submodule may
# live in the F Prime project or one level up in an enclosing workspace
# alongside settings.ini:
#
#   <project>/lib/eigen            e.g. PROVESFlightControllerReference/lib/eigen
#   <workspace>/lib/eigen          e.g. proves-workspace/lib/eigen
#
# Override explicitly if it is somewhere else:
#   cmake -DEIGEN_SOURCE_DIR=/abs/path/to/eigen ...
####

# NOT "NOT DEFINED": a failed configure used to write an empty value into
# the cache, after which this guard was false forever and the search was
# skipped -- so fixing the submodule and re-running gave the same
# "not found" error until you purged the build directory. Testing for an
# empty value catches both the unset and the poisoned case.
if (NOT EIGEN_SOURCE_DIR)
    foreach (_candidate
             "${CMAKE_CURRENT_LIST_DIR}/../lib/eigen"      # project-local
             "${CMAKE_CURRENT_LIST_DIR}/../../lib/eigen")  # workspace parent
        if (EXISTS "${_candidate}/Eigen/Core")
            get_filename_component(EIGEN_SOURCE_DIR "${_candidate}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

if (NOT EIGEN_SOURCE_DIR OR NOT EXISTS "${EIGEN_SOURCE_DIR}/Eigen/Core")
    # Clear the cache entry before failing, so the next configure searches
    # again instead of replaying this error.
    unset(EIGEN_SOURCE_DIR CACHE)
    message(FATAL_ERROR
        "Eigen not found. Searched:\n"
        "  ${CMAKE_CURRENT_LIST_DIR}/../lib/eigen\n"
        "  ${CMAKE_CURRENT_LIST_DIR}/../../lib/eigen\n"
        "Add it:  git submodule add https://gitlab.com/libeigen/eigen.git lib/eigen\n"
        "Or set:  -DEIGEN_SOURCE_DIR=/abs/path/to/eigen")
endif()

# Cached only after validation, so only a good value is ever persisted.
set(EIGEN_SOURCE_DIR "${EIGEN_SOURCE_DIR}" CACHE PATH "Path to the Eigen source tree (the dir containing Eigen/)" FORCE)

message(STATUS "GNC: Eigen at ${EIGEN_SOURCE_DIR}")

add_library(eigen_embedded INTERFACE)

# SYSTEM so Eigen's own warnings do not drown out yours -- relevant if
# you build flight code with -Wall -Wextra -Werror, which you should.
target_include_directories(eigen_embedded SYSTEM INTERFACE "${EIGEN_SOURCE_DIR}")

target_compile_definitions(eigen_embedded INTERFACE
    # Exclude the LGPL-licensed corners of Eigen. Keeps the license
    # story simple for flight software.
    EIGEN_MPL2_ONLY

    # Hard-fail at compile time on any code path that would heap
    # allocate. Fixed-size 3x3 and 3x1 types never do, so this is a
    # guarantee that nobody later slips a MatrixXf into the ADCS chain.
    EIGEN_NO_MALLOC

    # Compile out eigen_assert. Flight code must not abort() on a
    # numerical edge case -- the solver returns status codes instead.
    # DO NOT define this for host unit tests; you want the asserts there.
    EIGEN_NO_DEBUG

    # Do not pull in <iostream> / ostream operators. This alone is worth
    # tens of KB of flash on a Cortex-M target. (Eigen 3.4+.)
    EIGEN_NO_IO

    # No SIMD on Cortex-M33. Leaving this off can lead to Eigen trying
    # to reason about alignment it cannot satisfy.
    EIGEN_DONT_VECTORIZE

    # With vectorization off, over-alignment buys nothing and costs
    # stack. Setting both to 0 also removes the alignment-related
    # operator new overloads and their failure modes.
    EIGEN_MAX_ALIGN_BYTES=0
    EIGEN_MAX_STATIC_ALIGN_BYTES=0

    # Cap any temporary Eigen would place on the stack. Thread stacks
    # here are a few KB, not the 1 MB Eigen assumes by default.
    EIGEN_STACK_ALLOCATION_LIMIT=2048
)
