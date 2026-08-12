// ======================================================================
// \file   OrbitPropagator.cpp
//
// Per-tick pipeline:
//
//   1. acquireTime()  Fw::Time -> UTC / UT1 / TT Julian dates -> GMST
//   2. propagate()    time + TLE -> spacecraft r, v in TEME  (SGP4)
//   3. publish()      telemetry, including the ground track
//   4. emit()         OrbitState -> downstream, synchronously
//
// Stage 1 is the reason this component exists in this shape. Time
// acquisition, the UTC/UT1/TT scale conversions, and GMST are a single
// tightly-bound unit that must happen exactly once per cycle. Splitting
// them across components -- or letting a downstream component acquire
// its own time -- is what produced the 16.1 deg Earth rotation error in
// the original code.
// ======================================================================
#include "Gnc/Environment/OrbitPropagator/OrbitPropagator.hpp"

#include <cstring>

namespace Gnc {
namespace Environment {

namespace {

Gnc::Vector3 toFpp(const Astro::Vec3& v) {
    return Gnc::Vector3(v.x, v.y, v.z);
}

}  // namespace

OrbitPropagator::OrbitPropagator(const char* const compName)
    : OrbitPropagatorComponentBase(compName),
      m_sat(perturb::sgp4::elsetrec {}),
      m_tleValid(false) {}

OrbitPropagator::~OrbitPropagator() {}

// ----------------------------------------------------------------------
// Stage 1: time
//
// A bad timestamp silently corrupts every output downstream. The Sun
// moves 0.0417 deg/hour, so a one-minute clock error is ~0.0007 deg of
// Sun direction (negligible) but 0.25 deg of Earth rotation and ~450 km
// of along-track position (very much not).
// ----------------------------------------------------------------------

bool OrbitPropagator::acquireTime(Astro::TimeScales& ts, Fw::Time& stamp) {
    stamp = this->getTime();

    if (stamp.getTimeBase() == Fw::TimeBase::TB_NONE) {
        this->log_WARNING_HI_TimeInvalid();
        return false;
    }

    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F64 utcOffset = this->paramGet_UTC_OFFSET_SEC(valid);
    valid = Fw::ParamValid::INVALID;
    const F64 leap = this->paramGet_LEAP_SECONDS(valid);
    valid = Fw::ParamValid::INVALID;
    const F64 dut1 = this->paramGet_DUT1_SEC(valid);

    // UTC_OFFSET_SEC exists so a GPS-time or TAI-time source can be
    // shifted onto the UTC scale the rest of this file assumes.
    const F64 unixUtc = static_cast<F64>(stamp.getSeconds())
                      + static_cast<F64>(stamp.getUSeconds()) * 1.0e-6
                      + utcOffset;

    ts = Astro::computeTimeScales(unixUtc, dut1, leap);
    return true;
}

// ----------------------------------------------------------------------
// Stage 2: SGP4
// ----------------------------------------------------------------------

bool OrbitPropagator::propagate(const Astro::TimeScales& ts,
                                Astro::Vec3& posTeme,
                                Astro::Vec3& velTeme,
                                F64& ageDays) {
    // Hand perturb the split Julian date so the sub-second part is not
    // thrown away against the 2.46e6 day magnitude.
    const perturb::JulianDate nowJd(ts.jdUt1.day, ts.jdUt1.frac);

    ageDays = nowJd - m_sat.epoch();
    const F64 minsFromEpoch = ageDays * Astro::MIN_PER_DAY;

    // propagate_from_epoch is the primitive; propagate(jd) wraps it.
    // Calling it directly makes time-since-epoch explicit, which is the
    // quantity SGP4's drag and secular terms are actually series in.
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

// ----------------------------------------------------------------------
// The tick
// ----------------------------------------------------------------------

void OrbitPropagator::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    const Fw::Time tStart = this->getTime();

    Gnc::OrbitState state;
    state.set_validity(Gnc::OrbitValidity::NO_TIME);

    // ---- Stage 1 -----------------------------------------------------
    Astro::TimeScales ts;
    Fw::Time stamp;
    if (!this->acquireTime(ts, stamp)) {
        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TIME);
        // Emit anyway. Silence is indistinguishable from a hung
        // component; an explicit "invalid this cycle" is not.
        this->emit(state);
        return;
    }

    state.set_stamp(stamp);
    state.set_jdUt1(Astro::jdFlatten(ts.jdUt1));

    // GMST, computed once for the whole subsystem.
    const F64 gmst = Astro::gmst1982Rad(ts.tUt1);
    state.set_gmstRad(gmst);
    this->tlmWrite_GmstDeg(static_cast<F32>(gmst * Astro::RAD2DEG));

    // ---- No TLE ------------------------------------------------------
    // Time and GMST still publish. This matters: the solar ephemeris
    // downstream needs only a clock, so a safe-mode sun search keeps
    // working with no orbit knowledge at all.
    if (!m_tleValid) {
        this->log_WARNING_LO_NoTleLoaded();
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

    // SGP4 accuracy degrades roughly 1-3 km per day past epoch, so age
    // is a first-class quality indicator rather than a footnote.
    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F32 maxAge = this->paramGet_MAX_TLE_AGE_DAYS(valid);
    const bool stale = (ageDays > static_cast<F64>(maxAge));
    if (stale) {
        this->log_WARNING_LO_TleStale(static_cast<F32>(ageDays));
    }

    state.set_posTeme(toFpp(posTeme));
    state.set_velTeme(toFpp(velTeme));
    state.set_nadirTeme(toFpp(Astro::nadirUnit(posTeme)));
    state.set_tleAgeDays(static_cast<F32>(ageDays));
    state.set_validity(stale ? Gnc::OrbitValidity::STALE : Gnc::OrbitValidity::VALID);

    // ---- Stages 3 and 4 ----------------------------------------------
    this->publish(state);
    this->emit(state);

    // Execution budget. This now covers the WHOLE synchronous fan-out:
    // SGP4, the WMM evaluation, and the solar model all run before we
    // get here. On a soft-float M33 this is the number that tells you
    // whether the 1 Hz rate group is still comfortable.
    const Fw::Time tEnd = this->getTime();
    const I64 usec =
        (static_cast<I64>(tEnd.getSeconds()) - static_cast<I64>(tStart.getSeconds())) * 1000000LL
        + (static_cast<I64>(tEnd.getUSeconds()) - static_cast<I64>(tStart.getUSeconds()));
    this->tlmWrite_CycleUsec(static_cast<U32>((usec > 0) ? usec : 0));
}

// ----------------------------------------------------------------------
// Stage 3: telemetry
// ----------------------------------------------------------------------

void OrbitPropagator::publish(const Gnc::OrbitState& state) {
    const Gnc::Vector3 p = state.get_posTeme();
    const Astro::Vec3 posTeme { p.get_x(), p.get_y(), p.get_z() };

    // Ground track. Uses the GMST already in the state -- no second
    // evaluation anywhere in the system.
    const Astro::Vec3 ecef = Astro::temeToEcef(posTeme, state.get_gmstRad());

    F64 lat = 0.0;
    F64 lon = 0.0;
    F64 alt = 0.0;
    Astro::ecefToGeodetic(ecef, lat, lon, alt);

    this->tlmWrite_PosTeme(state.get_posTeme());
    this->tlmWrite_VelTeme(state.get_velTeme());
    this->tlmWrite_LatDeg(static_cast<F32>(lat * Astro::RAD2DEG));
    this->tlmWrite_LonDeg(static_cast<F32>(lon * Astro::RAD2DEG));
    this->tlmWrite_AltKm(static_cast<F32>(alt));
    this->tlmWrite_TleAgeDays(state.get_tleAgeDays());
    this->tlmWrite_Validity(state.get_validity());
}

// ----------------------------------------------------------------------
// Stage 4: broadcast
// ----------------------------------------------------------------------

void OrbitPropagator::emit(const Gnc::OrbitState& state) {
    if (!this->isConnected_orbitOut_OutputPort(0)) {
        return;
    }
    Gnc::OrbitState copy = state;
    this->orbitOut_out(0, copy);
}

// ----------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------

void OrbitPropagator::LOAD_TLE_cmdHandler(FwOpcodeType opCode,
                                          U32 cmdSeq,
                                          const Fw::CmdStringArg& line1,
                                          const Fw::CmdStringArg& line2) {
    // perturb::Satellite::from_tle takes char* because Vallado's
    // twoline2rv writes into the buffer while parsing. Copy into local
    // mutable buffers rather than casting away const on command args.
    char l1[perturb::TLE_LINE_LEN + 1];
    char l2[perturb::TLE_LINE_LEN + 1];

    (void)std::memset(l1, 0, sizeof l1);
    (void)std::memset(l2, 0, sizeof l2);
    (void)std::strncpy(l1, line1.toChar(), perturb::TLE_LINE_LEN);
    (void)std::strncpy(l2, line2.toChar(), perturb::TLE_LINE_LEN);

    const perturb::Satellite candidate = perturb::Satellite::from_tle(l1, l2);
    const perturb::Sgp4Error err = candidate.last_error();

    if (err != perturb::Sgp4Error::NONE) {
        // Reject without disturbing the currently loaded TLE: a bad
        // uplink must never leave the spacecraft worse off than before.
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
