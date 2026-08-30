/**
 * \file OrbitPropagator.cpp
 * \brief Per-tick pipeline:
 *
 * \details   1. acquireTime()  Fw::Time -> UTC / UT1 / TT Julian dates -> GMST
 *   2. propagate()    time + TLE -> spacecraft r, v in TEME  (SGP4)
 *   3. publish()      telemetry, including the ground track
 *   4. emit()         OrbitState -> downstream, synchronously
 *
 * Stage 1 is the reason this component exists in this shape. Time
 * acquisition, the UTC/UT1/TT scale conversions, and GMST are a single
 * tightly-bound unit that must happen exactly once per cycle. Splitting
 * them across components -- or letting a downstream component acquire
 * its own time -- is what produced the 16.1 deg Earth rotation error in
 * the original code.
 */
#include "Gnc/Environment/OrbitPropagator/OrbitPropagator.hpp"

#include "Gnc/Types/GncConvert.hpp"

#include <cstring>

namespace Gnc {
namespace Environment {

OrbitPropagator::OrbitPropagator(const char* const compName)
    : OrbitPropagatorComponentBase(compName),
      /**
       * perturb::Satellite has no default constructor, so it is seeded
       * with a value-initialized elsetrec. Every other member uses an
       * in-class initialiser.
       */
      m_sat(perturb::sgp4::elsetrec {}) {}

OrbitPropagator::~OrbitPropagator() {}

/*
 * ============================================================================
 * Stage 1: time
 *
 * A bad timestamp silently corrupts every output downstream. The Sun
 * moves 0.0417 deg/hour, so a one-minute clock error is ~0.0007 deg of
 * Sun direction (negligible) but 0.25 deg of Earth rotation and ~450 km
 * of along-track position (very much not).
 * ============================================================================
 */

bool OrbitPropagator::acquireTime(Astro::TimeScales& ts, Fw::Time& stamp) {
    stamp = this->getTime();

    /*
     * MUST be wall-clock time, not uptime.
     *
     * Drv::RtcManager falls back to TB_PROC_TIME (milliseconds since
     * boot) whenever the RTC is absent, unreadable, or holds an
     * out-of-range value. Accepting anything that is merely "not
     * TB_NONE" would feed SGP4 a timestamp of a few hundred seconds
     * past the POSIX epoch -- i.e. propagate the spacecraft to 1970,
     * roughly 56 years and 3e5 revolutions from the TLE epoch. SGP4
     * will not error on that; it will return a confident, absurd
     * state vector, and GMST will be wrong by an arbitrary angle.
     *
     * TB_WORKSTATION_TIME is what RtcManager sets only after a
     * successful rtc_get_time() that converted cleanly, so it is the
     * correct and only acceptable gate.
     */
    if (stamp.getTimeBase() != Fw::TimeBase::TB_WORKSTATION_TIME) {
        this->log_WARNING_HI_TimeMissing();
        return false;
    }

    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F64 utcOffset = this->paramGet_UTC_OFFSET_SEC(valid);
    valid = Fw::ParamValid::INVALID;
    const F64 leap = this->paramGet_LEAP_SEC(valid);
    valid = Fw::ParamValid::INVALID;
    const F64 dut1 = this->paramGet_DUT1_SEC(valid);

    /*
     * UTC_OFFSET_SEC exists so a GPS-time or TAI-time source can be
     * shifted onto the UTC scale the rest of this file assumes.
     */
    const F64 unixUtc = static_cast<F64>(stamp.getSeconds())
                      + static_cast<F64>(stamp.getUSeconds()) * 1.0e-6
                      + utcOffset;

    ts = Astro::computeTimeScales(unixUtc, dut1, leap);
    return true;
}

/*
 * ============================================================================
 * Stage 2: SGP4
 * ============================================================================
 */

bool OrbitPropagator::propagate(const Astro::TimeScales& ts,
                                Astro::Vec3& posTeme,
                                Astro::Vec3& velTeme,
                                F64& ageDays) {
    /*
     * Hand perturb the split Julian date so the sub-second part is not
     * thrown away against the 2.46e6 day magnitude.
     */
    const perturb::JulianDate nowJd(ts.jdUt1.day, ts.jdUt1.frac);

    ageDays = nowJd - m_sat.epoch();
    const F64 minsFromEpoch = ageDays * Astro::MIN_PER_DAY;

    /*
     * propagate_from_epoch is the primitive; propagate(jd) wraps it.
     * Calling it directly makes time-since-epoch explicit, which is the
     * quantity SGP4's drag and secular terms are actually series in.
     */
    perturb::StateVector sv;
    const perturb::Sgp4Error err = m_sat.propagate_from_epoch(minsFromEpoch, sv);

    if (err != perturb::Sgp4Error::NONE) {
        this->log_WARNING_HI_Sgp4Failure(static_cast<U8>(err), minsFromEpoch);
        return false;
    }

    posTeme = Astro::Vec3 { sv.position[0], sv.position[1], sv.position[2] };
    velTeme = Astro::Vec3 { sv.velocity[0], sv.velocity[1], sv.velocity[2] };
    return true;
}

/*
 * ============================================================================
 * The tick
 * ============================================================================
 */

void OrbitPropagator::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    const Fw::Time tStart = this->getTime();

    Gnc::OrbitState state;
    state.set_validity(Gnc::OrbitValidity::NO_TIME);
    state.set_timeUsable(false);
    state.set_positionUsable(false);

    // ---- Stage 1 -----------------------------------------------------
    Astro::TimeScales ts;
    Fw::Time stamp;
    if (!this->acquireTime(ts, stamp)) {
        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TIME);
        /*
         * Emit anyway. Silence is indistinguishable from a hung
         * component; an explicit "invalid this cycle" is not.
         */
        this->emit(state);
        return;
    }

    state.set_stamp(stamp);
    state.set_timeUsable(true);
    state.set_jdUt1(Astro::jdFlatten(ts.jdUt1));

    /*
     * TT as a Julian date, reconstructed from the centuries value that
     * computeTimeScales already produced. Carried because the solar
     * ephemeris needs it and receives only this struct; it previously
     * approximated jdTt = jdUt1, a silent ~69 s error.
     */
    state.set_jdTt(Astro::JD_J2000 + ts.tTt * Astro::DAYS_PER_JCENT);

    // GMST, computed once for the whole subsystem.
    const F64 gmst = Astro::gmst1982Rad(ts.tUt1);
    state.set_gmstRad(gmst);
    this->tlmWrite_GmstDeg(static_cast<F32>(gmst * Astro::RAD2DEG));

    /*
     * ---- No TLE ------------------------------------------------------
     * Time and GMST still publish. This matters: the solar ephemeris
     * downstream needs only a clock, so a safe-mode sun search keeps
     * working with no orbit knowledge at all.
     */
    if (!m_tleValid) {
        this->log_WARNING_LO_TleMissing();
        state.set_validity(Gnc::OrbitValidity::NO_TLE);
        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TLE);
        this->emit(state);
        return;
    }

    // ---- Stage 2 -----------------------------------------------------
    Astro::Vec3 posTeme;
    Astro::Vec3 velTeme;
    F64 ageDays = 0.0;
    if (!this->propagate(ts, posTeme, velTeme, ageDays)) {
        state.set_validity(Gnc::OrbitValidity::PROP_ERROR);
        this->tlmWrite_Validity(Gnc::OrbitValidity::PROP_ERROR);
        this->emit(state);
        return;
    }

    /*
     * SGP4 accuracy degrades roughly 1-3 km per day past epoch, so age
     * is a first-class quality indicator rather than a footnote.
     */
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F32 maxAge = this->paramGet_MAX_TLE_AGE_DAYS(valid);
    const bool isStale = (ageDays > static_cast<F64>(maxAge));
    if (isStale) {
        this->log_WARNING_LO_TleStale(static_cast<F32>(ageDays));
    }

    /*
     * WGS-84 geodetic conversion, run ONCE per cycle. It previously ran
     * twice on the same position -- here for the ground track and again
     * in the magnetic model's altitude gate -- which on a soft-float
     * M33 is several wasted transcendentals.
     */
    const Astro::Vec3 ecefKm = Astro::temeToEcef(posTeme, gmst);
    F64 latRad = 0.0;
    F64 lonRad = 0.0;
    F64 altKm = 0.0;
    Astro::ecefToGeodetic(ecefKm, latRad, lonRad, altKm);

    state.set_posTemeKm(toVec3d(posTeme));
    state.set_velTemeKmS(toVec3d(velTeme));
    state.set_latRad(latRad);
    state.set_lonRad(lonRad);
    state.set_altKm(altKm);
    state.set_tleAgeDays(static_cast<F32>(ageDays));
    state.set_validity(isStale ? Gnc::OrbitValidity::STALE : Gnc::OrbitValidity::VALID);
    state.set_positionUsable(true);

    // ---- Stages 3 and 4 ----------------------------------------------
    this->publish(state);
    this->emit(state);

    /*
     * Execution budget. This now covers the WHOLE synchronous fan-out:
     * SGP4, the WMM evaluation, and the solar model all run before we
     * get here. On a soft-float M33 this is the number that tells you
     * whether the 1 Hz rate group is still comfortable.
     */
    const Fw::Time tEnd = this->getTime();
    const I64 usec =
        (static_cast<I64>(tEnd.getSeconds()) - static_cast<I64>(tStart.getSeconds())) * 1000000LL
        + (static_cast<I64>(tEnd.getUSeconds()) - static_cast<I64>(tStart.getUSeconds()));
    this->tlmWrite_CycleUsec(static_cast<U32>((usec > 0) ? usec : 0));
}

/*
 * ============================================================================
 * Stage 3: telemetry
 * ============================================================================
 */

void OrbitPropagator::publish(const Gnc::OrbitState& state) {
    /*
     * Geodetic position is read from the state, not recomputed. See the
     * note in run_handler.
     */
    this->tlmWrite_PosTemeKm(state.get_posTemeKm());
    this->tlmWrite_VelTemeKmS(state.get_velTemeKmS());
    this->tlmWrite_LatDeg(static_cast<F32>(state.get_latRad() * Astro::RAD2DEG));
    this->tlmWrite_LonDeg(static_cast<F32>(state.get_lonRad() * Astro::RAD2DEG));
    this->tlmWrite_AltKm(static_cast<F32>(state.get_altKm()));
    this->tlmWrite_TleAgeDays(state.get_tleAgeDays());
    this->tlmWrite_Validity(state.get_validity());
}

/*
 * ============================================================================
 * Stage 4: broadcast
 * ============================================================================
 */

void OrbitPropagator::emit(const Gnc::OrbitState& state) {
    if (!this->isConnected_orbitOut_OutputPort(0)) {
        return;
    }
    Gnc::OrbitState copy = state;
    this->orbitOut_out(0, copy);
}

/*
 * ============================================================================
 * Commands
 * ============================================================================
 */

void OrbitPropagator::LOAD_TLE_cmdHandler(FwOpcodeType opCode,
                                          U32 cmdSeq,
                                          const Fw::CmdStringArg& line1,
                                          const Fw::CmdStringArg& line2) {
    /*
     * perturb::Satellite::from_tle takes char* because Vallado's
     * twoline2rv writes into the buffer while parsing. Copy into local
     * mutable buffers rather than casting away const on command args.
     */
    char l1[perturb::TLE_LINE_LEN + 1];
    char l2[perturb::TLE_LINE_LEN + 1];

    (void)std::memset(l1, 0, sizeof l1);
    (void)std::memset(l2, 0, sizeof l2);
    (void)std::strncpy(l1, line1.toChar(), perturb::TLE_LINE_LEN);
    (void)std::strncpy(l2, line2.toChar(), perturb::TLE_LINE_LEN);

    const perturb::Satellite candidate = perturb::Satellite::from_tle(l1, l2);
    const perturb::Sgp4Error err = candidate.last_error();

    if (err != perturb::Sgp4Error::NONE) {
        /*
         * Reject without disturbing the currently loaded TLE: a bad
         * uplink must never leave the spacecraft worse off than before.
         */
        this->log_WARNING_HI_TleRejected(static_cast<U8>(err));
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    m_sat = candidate;
    m_tleValid = true;

    const perturb::JulianDate epoch = m_sat.epoch();
    this->log_ACTIVITY_HI_TleAccepted(static_cast<U32>(m_sat.sat_rec.satnum),
                                      epoch.jd + epoch.jd_frac);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void OrbitPropagator::CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_tleValid = false;
    this->log_ACTIVITY_HI_TleCleared();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
