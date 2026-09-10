/**
 * \file OrbitPropagator.cpp
 * \brief Gets the time stamp needed for all GNC computations, computes SGP4, and broadcasts the OrbitState to its consumers.
 *
 * \details 1. acquireTime()  Fw::Time -> UTC / UT1 / TT Julian dates -> GMST
 *          2. propagate()    time + TLE -> spacecraft r, v in TEME  (SGP4)
 *          3. publish()      telemetry, including the ground track
 *          4. emit()         OrbitState -> downstream, synchronously
 */
#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/OrbitPropagator.hpp"

#include <cstring>

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Construct the component with no TLE loaded.
 * \param compName  F Prime component instance name
 */
OrbitPropagator::OrbitPropagator(const char* const compName)
    : OrbitPropagatorComponentBase(compName),
      /*
       * perturb::Satellite has no default constructor, so it is seeded
       * with a value-initialized elsetrec. Every other member uses an
       * in-class initialiser.
       */
      m_sat(perturb::sgp4::elsetrec{}) {}

//! Destroy the component. Holds no resources.
OrbitPropagator::~OrbitPropagator() {}

/**
 * \brief Rate group tick. Runs all four stages and times itself.
 *
 * \details Degrades in steps rather than all at once. NO_TIME stops
 * everything. NO_TLE still publishes time and GMST, which is all the
 * solar ephemeris needs. PROP_ERROR does the same. STALE publishes a
 * full solution and flags it. Each of those paths emits, so a consumer
 * always learns why it did not get a position.
 *
 * The CycleUsec measurement at the end covers the synchronous fan-out in
 * stage 4, so it is the execution cost of the entire GNC chain and not
 * just of this component.
 *
 * \param portNum  Port index, unused
 * \param context  Rate group context, unused
 */
void OrbitPropagator::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    const Fw::Time tStart = this->getTime();

    Gnc::OrbitState state;
    state.set_validity(Gnc::OrbitValidity::NO_TIME);
    state.set_timeUsable(false);
    state.set_positionUsable(false);

    /*
     * ----------------------------------------------------------------------------
     * Stage 1: Time Acquisition
     * ----------------------------------------------------------------------------
     */
    Astro::TimeScales ts;
    Fw::Time stamp;

    // Still emit invalid state for visibility
    if (!this->acquireTime(ts, stamp)) {
        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TIME);
        this->emit(state);
        return;
    }

    state.set_stamp(stamp);
    state.set_timeUsable(true);
    state.set_jdUt1(Astro::jdFlatten(ts.jdUt1));

    /*
     * TT as a Julian date, reconstructed from the centuries value that
     * computeTimeScales already produced. Carried for solar ephemeris.
     */
    state.set_jdTt(Astro::JD_J2000 + ts.tTt * Astro::DAYS_PER_JCENT);

    // GMST, computed once for the whole subsystem.
    const F64 gmst = Astro::gmst1982Rad(ts.tUt1);
    state.set_gmstRad(gmst);
    this->tlmWrite_GmstDeg(static_cast<F32>(gmst * Astro::RAD2DEG));

    /*
     * Time is still valid when the TLE is missing or invalid. Solar Ephemeris just needs the time.
     */
    if (!m_tleValid) {
        this->log_WARNING_LO_TleMissing();
        state.set_validity(Gnc::OrbitValidity::NO_TLE);
        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TLE);
        this->emit(state);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Stage 2: SGP4 orbit propagation
     * ----------------------------------------------------------------------------
     */

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
     * SGP4 accuracy degrades 1-3 km per day past TLE epoch.
     */
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F32 maxAge = this->paramGet_MAX_TLE_AGE_DAYS(valid);
    const bool isStale = (ageDays > static_cast<F64>(maxAge));
    if (isStale) {
        this->log_WARNING_LO_TleStale(static_cast<F32>(ageDays));
    }

    // WGS-84 geodetic conversion.
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

    // Stage 3
    this->publish(state);

    // Stage 4
    this->emit(state);

    /*
     * Execution budget. SGP4, WMM evaluation, and the solar model all run before this.
     * Tells us if 1 Hz rate group is feasible or needs to be reduced.
     */
    const Fw::Time tEnd = this->getTime();
    const I64 usec = (static_cast<I64>(tEnd.getSeconds()) - static_cast<I64>(tStart.getSeconds())) * 1000000LL +
                     (static_cast<I64>(tEnd.getUSeconds()) - static_cast<I64>(tStart.getUSeconds()));
    this->tlmWrite_CycleUsec(static_cast<U32>((usec > 0) ? usec : 0));
}

/**
 * \brief Stage 1. Read the clock and derive UTC / UT1 / TT.
 *
 * \details Rejects anything that isn't wall-clock time. The three
 * parameters applied here are the only ground-adjustable knobs on the
 * time scale: UTC_OFFSET_SEC shifts a GPS- or TAI-referenced source onto
 * UTC, LEAP_SEC supplies TAI-UTC, and DUT1_SEC supplies UT1-UTC.
 *
 * A stale DUT1 costs at most 0.9 s of Earth rotation (~0.004 deg), so a
 * missing parameter degrades the ground track slightly rather than
 * invalidating the cycle.
 *
 * \param ts     [out] Populated time scales
 * \param stamp  [out] The Fw::Time the scales were derived from
 * \return true when time was usable; false after WARNING_HI TimeMissing
 */
bool OrbitPropagator::acquireTime(Astro::TimeScales& ts, Fw::Time& stamp) {
    stamp = this->getTime();

    /*
     * Must be wall-clock time, not uptime.
     * RtcManager sets the timebase to TB_WORKSTATION_TIME when time is successfully acquired from the RTC.
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
    const F64 unixUtc =
        static_cast<F64>(stamp.getSeconds()) + static_cast<F64>(stamp.getUSeconds()) * 1.0e-6 + utcOffset;

    ts = Astro::computeTimeScales(unixUtc, dut1, leap);
    return true;
}


/**
 * \brief Stage 2. Run SGP4 from the TLE epoch to the current UT1 instant.
 *
 * \details Calls propagate_from_epoch() rather than propagate(jd) so the
 * minutes-since-epoch argument is explicit. Position and velocity come
 * back in TEME, which is the frame the rest of the subsystem works in
 * precisely so that no conversion is needed here.
 *
 * Age is not gated in this function -- it is returned so run_handler can
 * apply the MAX_TLE_AGE_DAYS policy and still publish a stale-but-usable
 * solution.
 *
 * \param ts       Time scales from acquireTime()
 * \param posTeme  [out] Position, TEME, km
 * \param velTeme  [out] Velocity, TEME, km/s
 * \param ageDays  [out] Time since TLE epoch, days
 * \return true on success; false after WARNING_HI Sgp4Failure
 */
bool OrbitPropagator::propagate(const Astro::TimeScales& ts, Astro::Vec3& posTeme, Astro::Vec3& velTeme, F64& ageDays) {
    // Split Julian date used to preserve the fraction
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

    posTeme = Astro::Vec3{sv.position[0], sv.position[1], sv.position[2]};
    velTeme = Astro::Vec3{sv.velocity[0], sv.velocity[1], sv.velocity[2]};
    return true;
}

/**
 * \brief Stage 3. Write this cycle's telemetry.
 *
 * \details Only called on the fully valid path. The early-exit paths
 * write Validity directly and skip the rest, so the ground never sees a
 * position channel updating while Validity says there is no position.
 *
 * \param state  Fully populated orbit state
 */
void OrbitPropagator::publish(const Gnc::OrbitState& state) {
    this->tlmWrite_PosTemeKm(state.get_posTemeKm());
    this->tlmWrite_VelTemeKmS(state.get_velTemeKmS());
    this->tlmWrite_LatDeg(static_cast<F32>(state.get_latRad() * Astro::RAD2DEG));
    this->tlmWrite_LonDeg(static_cast<F32>(state.get_lonRad() * Astro::RAD2DEG));
    this->tlmWrite_AltKm(static_cast<F32>(state.get_altKm()));
    this->tlmWrite_TleAgeDays(state.get_tleAgeDays());
    this->tlmWrite_Validity(state.get_validity());
}

/**
 * \brief Stage 4. Broadcast the orbit state downstream.
 *
 * \details Called on every path, including the failure ones, so
 * consumers see an explicit invalid state. Copies
 * because the output port takes a mutable reference.
 *
 * \param state  Orbit state to broadcast
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

/**
 * \brief Parse an uplinked TLE and install it if it is well formed.
 *
 * \details Parsing is destructive (Vallado's twoline2rv writes into the
 * buffer), so the command arguments are copied into local mutable
 * buffers.
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 * \param line1   TLE line 1
 * \param line2   TLE line 2
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
    this->log_ACTIVITY_HI_TleAccepted(static_cast<U32>(m_sat.sat_rec.satnum), epoch.jd + epoch.jd_frac);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

/**
 * \brief Mark the loaded TLE unusable.
 *
 * \details Used to stop a known-bad element set from driving pointing
 * or ground-track products until a replacement is uplinked.
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 */
void OrbitPropagator::CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_tleValid = false;
    this->log_ACTIVITY_HI_TleCleared();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
