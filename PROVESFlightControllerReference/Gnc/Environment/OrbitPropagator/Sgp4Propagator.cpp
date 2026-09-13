/**
 * \file Sgp4Propagator.cpp
 * \brief Time scales and SGP4
 */
#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/Sgp4Propagator.hpp"

#include <cstring>

namespace Gnc {
namespace Environment {


/**
 * \brief The SGP4 orbital record and its validity flag.
 *
 * \details Holds the parsed TLE between cycles. 
 * Only overwrites the old TLE when the new one is parsed successfully.
 * 
 */
Sgp4Propagator::Sgp4Propagator()
    /*
     * perturb::Satellite has no default constructor, so it's seeded
     * with a value-initialized elsetrec and gated by m_tleValid.
     */
    : m_sat(perturb::sgp4::elsetrec{}) {}

/**
 * \brief Parse a two-line element set and install it if well formed.
 *
 * \param line1     TLE line 1, NUL terminated
 * \param line2     TLE line 2, NUL terminated
 * \param errCode   [out] perturb::Sgp4Error value from the parse
 * \return true when the TLE was accepted and installed; false when
 *         it was rejected.
 */
bool Sgp4Propagator::loadTle(const char* line1, const char* line2, std::uint8_t& errCode) {

    //Copy into local buffers because perturb::Satellite::from_tle writes in the buffer while parsing.

    char l1[perturb::TLE_LINE_LEN + 1];
    char l2[perturb::TLE_LINE_LEN + 1];

    (void)std::memset(l1, 0, sizeof l1);
    (void)std::memset(l2, 0, sizeof l2);
    (void)std::strncpy(l1, line1, perturb::TLE_LINE_LEN);
    (void)std::strncpy(l2, line2, perturb::TLE_LINE_LEN);

    const perturb::Satellite candidate = perturb::Satellite::from_tle(l1, l2);
    const perturb::Sgp4Error err = candidate.last_error();
    errCode = static_cast<std::uint8_t>(err);

    // Reject bad TLE and leave old one in place.
    if (err != perturb::Sgp4Error::NONE) {
        return false;
    }

    m_sat = candidate;
    m_tleValid = true;
    return true;
}

//! Mark the loaded TLE unusable. The record itself is left in place.
void Sgp4Propagator::clearTle() {
    m_tleValid = false;
}

//! Whether a successfully parsed TLE is loaded.
bool Sgp4Propagator::hasTle() const {
    return m_tleValid;
}
//! NORAD catalog number of the loaded TLE, as a string. Meaningless unless hasTle().
void Sgp4Propagator::satnum(char* buf, std::size_t len) const {
    if (buf == nullptr || len == 0) {
        return;
    }

    const std::size_t field = sizeof(m_sat.sat_rec.satnum);

    std::size_t n = (len - 1);
    
    if(field < (len - 1)){
        n = field;
    }

    (void)std::memcpy(buf, m_sat.sat_rec.satnum, n);
    buf[n] = '\0';
}

//! Epoch of the loaded TLE, Julian date. Meaningless unless hasTle().
double Sgp4Propagator::epochJd() const {
    const perturb::JulianDate epoch = m_sat.epoch();
    return epoch.jd + epoch.jd_frac;
}

/**
 * \brief Time since the loaded TLE's epoch.
 *
 * \param jdUt1  Current UT1 as a split Julian date.
 * 
 * \return Days since epoch. Negative for a future epoch.
 */
double Sgp4Propagator::ageDaysAt(const Astro::JulianDate2& jdUt1) const {
    // Split Julian date used to preserve the fraction
    const perturb::JulianDate nowJd(jdUt1.day, jdUt1.frac);
    return nowJd - m_sat.epoch();
}

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
bool Sgp4Propagator::propagateFromEpoch(double minsFromEpoch,
                                        Astro::Vec3& posTeme,
                                        Astro::Vec3& velTeme,
                                        std::uint8_t& errCode) {
    perturb::StateVector sv;
    const perturb::Sgp4Error err = m_sat.propagate_from_epoch(minsFromEpoch, sv);
    errCode = static_cast<std::uint8_t>(err);

    if (err != perturb::Sgp4Error::NONE) {
        return false;
    }

    posTeme = Astro::Vec3{sv.position[0], sv.position[1], sv.position[2]};
    velTeme = Astro::Vec3{sv.velocity[0], sv.velocity[1], sv.velocity[2]};
    return true;
}

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
OrbitResult solveOrbit(double unixSecondsUtc, const OrbitConfig& cfg, Sgp4Propagator& sat, OrbitSolution& out) {
    /*
     * ----------------------------------------------------------------------------
     * Stage 1: time scales.
     * ----------------------------------------------------------------------------
     */

    out.ts = Astro::computeTimeScales(unixSecondsUtc + cfg.utcOffsetSec, cfg.dut1Sec, cfg.leapSec);
    out.jdUt1 = Astro::jdFlatten(out.ts.jdUt1);

    /*
     * TT as a Julian date, reconstructed from the centuries value that
     * computeTimeScales produced. Carried for solar ephemeris.
     */
    out.jdTt = Astro::JD_J2000 + out.ts.tTt * Astro::DAYS_PER_JCENT;

    // GMST, computed once for the whole subsystem.
    out.gmstRad = Astro::gmst1982Rad(out.ts.tUt1);

    /*
     * Time is still valid when the TLE is missing or invalid.
     */
    if (!sat.hasTle()) {
        return OrbitResult::NO_TLE;
    }

    /*
     * ----------------------------------------------------------------------------
     * Stage 2: SGP4 orbit propagation.
     * ----------------------------------------------------------------------------
     */
    out.tleAgeDays = sat.ageDaysAt(out.ts.jdUt1);
    out.minsFromEpoch = out.tleAgeDays * Astro::MINUTES_PER_DAY;

    if (!sat.propagateFromEpoch(out.minsFromEpoch, out.posTemeKm, out.velTemeKmS, out.sgp4Code)) {
        return OrbitResult::PROP_ERROR;
    }

    /*
     * ----------------------------------------------------------------------------
     * Stage 3: ground track.
     *
     * WGS-84 geodetic conversion
     * ----------------------------------------------------------------------------
     */
    const Astro::Vec3 ecefKm = Astro::temeToEcef(out.posTemeKm, out.gmstRad);
    Astro::ecefToGeodetic(ecefKm, out.latRad, out.lonRad, out.altKm);

    /*
     * SGP4 accuracy degrades 1-3 km per day past TLE epoch.
     */
    return (out.tleAgeDays > cfg.maxTleAgeDays) ? OrbitResult::STALE : OrbitResult::VALID;
}

}  // namespace Environment
}  // namespace Gnc
