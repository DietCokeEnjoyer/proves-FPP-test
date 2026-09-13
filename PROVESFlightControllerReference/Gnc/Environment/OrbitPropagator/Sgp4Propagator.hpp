/**
 * \file Sgp4Propagator.hpp
 * \brief Time scales and SGP4 propagation.
 *
 * \details No FPrime. Uses AstroLib and the perturb SGP4 library.
 *
 * Per-cycle pipeline in solveOrbit():
 *   1. POSIX seconds + offsets -> UTC / UT1 / TT Julian dates -> GMST
 *   2. time + TLE -> spacecraft r, v in TEME  (SGP4)
 *   3. r -> ECEF -> WGS-84 geodetic ground track
 *
 * Stage 1 always runs. Stages 2 and 3 need a loaded TLE.
 */
#ifndef Gnc_Environment_Sgp4Propagator_HPP
#define Gnc_Environment_Sgp4Propagator_HPP

#include <cstdint>

#include <perturb/perturb.hpp>

#include "PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief How far a cycle got, and why it stopped.
 *
 * \details Mirrors the FPP Gnc::OrbitValidity.
 */
enum class OrbitResult : std::uint8_t {
    NO_TLE = 0,      //!< No TLE loaded. Time and GMST are still valid.
    NO_TIME = 1,     //!< Time source unusable. Produced by the caller, never here.
    PROP_ERROR = 2,  //!< SGP4 returned a non-zero error code
    STALE = 3,       //!< Solution is good but the TLE is older than maxTleAgeDays
    VALID = 4        //!< Fully valid
};

/**
 * \brief Time-scale offsets and the staleness policy.
 *
 * \details Defaults must mirror the OrbitPropagator FPP parameter
 * defaults.
 */
struct OrbitConfig {
    /**
     * Offset added to the incoming POSIX seconds to obtain UTC.
     */
    double utcOffsetSec = 0.0;

    //! TAI - UTC, seconds. Bump when a leap second is announced.
    double leapSec = 37.0;

    /**
     * UT1 - UTC, seconds, from IERS Bulletin A. Always < 0.9 s, so
     * leaving this at 0 is at most 0.004 deg of Earth rotation error.
     */
    double dut1Sec = 0.0;

    //! TLE age in days beyond which the result is reported STALE.
    double maxTleAgeDays = 7.0;
};

/**
 * \brief Everything one cycle produces.
 * 
 * \details Contains all information needed for an OrbitState.
 */
struct OrbitSolution {
    // Time, written on every solveOrbit() call.
    Astro::TimeScales ts{};  //!< UTC / UT1 / TT scales for this instant
    double jdUt1 = 0.0;      //!< UT1 Julian date, flattened
    double jdTt = 0.0;       //!< TT Julian date, flattened
    double gmstRad = 0.0;    //!< GMST at jdUt1, radians

    // Position, written when the TLE is VALID or STALE
    Astro::Vec3 posTemeKm{0.0, 0.0, 0.0};   //!< Position, TEME, kilometers
    Astro::Vec3 velTemeKmS{0.0, 0.0, 0.0};  //!< Velocity, TEME, kilometers per second
    double latRad = 0.0;                    //!< Sub-satellite geodetic latitude, radians
    double lonRad = 0.0;                    //!< Sub-satellite geodetic longitude, radians
    double altKm = 0.0;                     //!< Geodetic altitude above the WGS-84 ellipsoid, km

    //! Time since TLE epoch, days. May be negative for a future epoch.
    double tleAgeDays = 0.0;

    //! Same quantity in minutes, which is what SGP4's series are in
    double minsFromEpoch = 0.0;

    //! perturb::Sgp4Error value from the last propagation attempt
    std::uint8_t sgp4Code = 0;
};

/**
 * \brief The SGP4 orbital record and its validity flag.
 *
 * \details Holds the parsed TLE between cycles. 
 * Only overwrites the old TLE when the new one is parsed successfully.
 * 
 */
class Sgp4Propagator {
  public:
    //! Construct with no TLE loaded.
    Sgp4Propagator();

    /**
     * \brief Parse a two-line element set and install it if well formed.
     *
     * \param line1     TLE line 1, NUL terminated
     * \param line2     TLE line 2, NUL terminated
     * \param errCode   [out] perturb::Sgp4Error value from the parse
     * \return true when the TLE was accepted and installed; false when
     *         it was rejected.
     */
    bool loadTle(const char* line1, const char* line2, std::uint8_t& errCode);

    //! Mark the loaded TLE unusable. The record itself is left in place.
    void clearTle();

    //! Whether a successfully parsed TLE is loaded.
    bool hasTle() const;

    //! NORAD catalog number of the loaded TLE. Meaningless unless hasTle().
    std::uint32_t satnum() const;

    //! Epoch of the loaded TLE, Julian date. Meaningless unless hasTle().
    double epochJd() const;

    /**
     * \brief Time since the loaded TLE's epoch.
     *
     * \param jdUt1  Current UT1 as a split Julian date.
     * 
     * \return Days since epoch. Negative for a future epoch.
     */
    double ageDaysAt(const Astro::JulianDate2& jdUt1) const;

    /**
     * \brief Run SGP4 forward from the TLE epoch.
     *
     * \details
     *
     * \param minsFromEpoch  Minutes since the TLE epoch
     * \param posTeme        [out] Position, TEME, km
     * \param velTeme        [out] Velocity, TEME, km/s
     * \param errCode        [out] perturb::Sgp4Error value
     * \return true on success; false when SGP4 reported an error, in
     *         which case posTeme and velTeme are untouched
     */
    bool propagateFromEpoch(double minsFromEpoch,
                            Astro::Vec3& posTeme,
                            Astro::Vec3& velTeme,
                            std::uint8_t& errCode);

  private:
    perturb::Satellite m_sat;  //!< Valid only while m_tleValid is true
    bool m_tleValid = false;   //!< Whether m_sat holds a successfully parsed TLE
};

/**
 * \brief Build every time scale, then propagate the loaded TLE.
 *
 * \param unixSecondsUtc  Seconds since the POSIX epoch, before
 *                        OrbitConfig::utcOffsetSec is applied
 *
 * \param cfg             Time-scale offsets and the staleness policy
 *
 * \param sat             [in,out] The loaded TLE. Passed by reference
 *                        because SGP4 propagation mutates the record.
 *
 * \param out             [out] Time products always; position products
 *                        only when VALID or STALE is returned
 *
 * \return OrbitResult::VALID or STALE on success, otherwise the stage
 *         that stopped the cycle
 */
OrbitResult solveOrbit(double unixSecondsUtc, const OrbitConfig& cfg, Sgp4Propagator& sat, OrbitSolution& out);

}  // namespace Environment
}  // namespace Gnc

#endif  // Gnc_Environment_Sgp4Propagator_HPP
