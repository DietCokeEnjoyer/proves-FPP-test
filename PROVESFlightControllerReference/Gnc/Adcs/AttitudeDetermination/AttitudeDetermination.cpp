/**
 * \file AttitudeDetermination.cpp
 * \brief cpp file for AttitudeDetermination component implementation
 *
 * \details Framework layer. Owns latching, staleness, frame
 * checking, parameters, telemetry, events, and fault reporting.
 * TRIAD math lives in TriadSolver.cpp.
 */

#include "PROVESFlightControllerReference/Gnc/Adcs/AttitudeDetermination/AttitudeDetermination.hpp"

#include <cmath>

namespace Gnc {
namespace Adcs {

namespace {

/**
 * Defaults used when a parameter has never been set.
 * Match with the FPP defaults.
 */
constexpr U32 DEFAULT_BODY_AGE_MS = 500U;
constexpr U32 DEFAULT_REF_AGE_MS = 3000U;
constexpr U32 DEFAULT_AGE_CYCLES = 20U;

/**
 * \brief FPP F32 3-D vector -> Eigen.
 *
 * \details Kept here and not in GncConvert.hpp because only this component needs Eigen.
 *
 * \param v  FPP F32 vector
 * \return The same components as an Eigen(the library) vector
 */
Eigen::Vector3f toEigen(const Gnc::Vec3f& v) {
    return Eigen::Vector3f(v.get_x(), v.get_y(), v.get_z());
}

/**
 * \brief Eigen quaternion -> FPP array, preserving the [x,y,z,w] order.
 *
 * \details Eigen's constructor argument order is (w,x,y,z) but its
 * storage order is [x,y,z,w]. This follows storage order.
 *
 * \param q  Attitude quaternion
 * \return Four-element FPP array, [x,y,z,w]
 */
Adcs::Quatf toFpp(const Eigen::Quaternionf& q) {
    Adcs::Quatf out;
    out[0] = q.x();
    out[1] = q.y();
    out[2] = q.z();
    out[3] = q.w();
    return out;
}

/**
 * \brief Age in milliseconds between two Fw::Time values.
 *
 * \param now   Current time
 * \param then  Time the sample was observed
 * \param out   [out] Age in ms, saturated at U32 max. Untouched on failure.
 * \return false when the time bases differ, when there is no time base,
 *         or when the clock has stepped backwards; true otherwise
 */
bool ageMs(const Fw::Time& now, const Fw::Time& then, U32& out) {
    if (now.getTimeBase() != then.getTimeBase()) {
        return false;
    }
    if (now.getTimeBase() == TimeBase::TB_NONE) {
        return false;
    }

    const I64 d = (static_cast<I64>(now.getSeconds()) - static_cast<I64>(then.getSeconds())) * 1000LL +
                  (static_cast<I64>(now.getUSeconds()) - static_cast<I64>(then.getUSeconds())) / 1000LL;

    if (d < 0) {
        return false;  // clock stepped backwards
    }

    constexpr I64 MAX_U32 = 0xFFFFFFFFLL;
    out = static_cast<U32>((d > MAX_U32) ? MAX_U32 : d);
    return true;
}

}  // namespace

/*
 * ============================================================================
 * Construction / destruction
 * ============================================================================
 */

/**
 * \brief Construct the component with an identity attitude and empty latches.
 * \param compName  F Prime component instance name
 */
AttitudeDetermination ::AttitudeDetermination(const char* const compName)
    : AttitudeDeterminationComponentBase(compName) {}

//! Destroy the component. Holds no resources.
AttitudeDetermination ::~AttitudeDetermination() {}

/*
 * ============================================================================
 * Input port handlers
 * ============================================================================
 */

/**
 * \brief Latch the measured sun vector, body frame.
 * \param portNum  Port index, unused
 * \param sample   Incoming sample
 */
void AttitudeDetermination ::sunBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_sunBody, sample);
}

/**
 * \brief Latch the measured magnetic field vector, body frame.
 * \param portNum  Port index, unused
 * \param sample   Incoming sample
 */
void AttitudeDetermination ::magBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_magBody, sample);
}

/**
 * \brief Latch the modeled sun direction, reference frame.
 * \param portNum  Port index, unused
 * \param sample   Incoming sample
 */
void AttitudeDetermination ::sunRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_sunRef, sample);
}

/**
 * \brief Latch the modeled magnetic field direction, reference frame.
 * \param portNum  Port index, unused
 * \param sample   Incoming sample
 */
void AttitudeDetermination ::magRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_magRef, sample);
}

/**
 * \brief Overwrite a latch slot and reset its cycle age.
 * \param slot    [out] Slot to overwrite
 * \param sample  Incoming sample
 */
void AttitudeDetermination ::latch(LatchedSample& slot, const Gnc::VectorSample& sample) {
    slot.vec = toEigen(sample.get_vec());
    slot.stamp = sample.get_stamp();
    slot.frame = sample.get_frame();
    slot.valid = sample.get_valid();
    slot.ageCycles = 0;
    slot.everSet = true;
}

/**
 * \brief Rate group tick. Age every slot by one cycle, then solve once.
 * \param portNum  Port index, unused
 * \param context  Rate group context, unused
 */
void AttitudeDetermination ::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    /*
     * Saturating increment so a long dropout can't wrap the counter
     * back around into "fresh"
     */
    LatchedSample* const slots[] = {&this->m_sunBody, &this->m_magBody, &this->m_sunRef, &this->m_magRef};
    for (LatchedSample* slot : slots) {
        if (slot->ageCycles < 0xFFFFFFFFU) {
            slot->ageCycles++;
        }
    }

    this->solveAndPublish();
}


/**
 * \brief Check a latched sample's frame tag against the expected value.
 * \param slot      Slot to check
 * \param name      Slot name, for the FrameMismatch event
 * \param expected  Required frame tag
 * \return true when the tag matches or nothing has arrived yet
 */
bool AttitudeDetermination ::checkFrame(const LatchedSample& slot, const char* name, Gnc::FrameId expected) {
    if (!slot.everSet) {
        return true;  // nothing received yet; staleness will catch it
    }
    if (slot.frame == expected) {
        return true;
    }
    Fw::LogStringArg arg(name);
    this->log_WARNING_HI_FrameMismatch(arg, slot.frame, expected);
    return false;
}

/**
 * \brief Check a latched sample's freshness, wall clock first.
 * \param slot          Slot to check
 * \param name          Slot name, for the SampleStale event
 * \param now           Current time
 * \param maxAgeMs      Age limit when the wall clock is usable
 * \param maxAgeCycles  Age limit in rate group cycles, for the fallback
 * \return Verdict plus the measured age
 */
AttitudeDetermination::Freshness AttitudeDetermination ::checkFresh(const LatchedSample& slot,
                                                                    const char* name,
                                                                    const Fw::Time& now,
                                                                    U32 maxAgeMs,
                                                                    U32 maxAgeCycles) {
    Freshness f;

    if (!slot.valid || !slot.everSet) {
        f.usable = false;
        return f;
    }

    U32 ms = 0;
    if (ageMs(now, slot.stamp, ms)) {
        f.ageMs = ms;
        f.ageKnown = true;
        f.usable = (ms <= maxAgeMs);
    } else {
        // No usable clock: fall back to counting rate group cycles.
        f.ageKnown = false;
        f.usable = (slot.ageCycles <= maxAgeCycles);
    }

    if (!f.usable) {
        Fw::LogStringArg arg(name);
        this->log_WARNING_LO_SampleStale(arg, f.ageMs);
    }
    return f;
}

/**
 * \brief Assemble inputs, run TRIAD, publish, telemeter.
 *
 * \details Gates in order: frame tags, then availability and
 * freshness, then the solver's own checks. Early exits still
 * publish the last good solution and explain the rejection.
 * 
 * \param q         Quaternion to publish, [x,y,z,w]
 * \param status    Why this solution is or is not valid
 * \param stamp     Epoch of the oldest contributing input
 * \param refFrame  Reference frame the solution is relative to
 * \param valid     Whether q came from a successful solve this cycle
 */
void AttitudeDetermination ::solveAndPublish() {
    const Fw::Time now = this->getTime();

    Fw::ParamValid pv = Fw::ParamValid::INVALID;
    const U32 rawBodyMs = this->paramGet_MAX_BODY_AGE_MS(pv);
    
    U32 maxBodyMs = DEFAULT_BODY_AGE_MS;
    if(pv == Fw::ParamValid::VALID){
        maxBodyMs = rawBodyMs;
    }

    pv = Fw::ParamValid::INVALID;
    const U32 rawRefMs = this->paramGet_MAX_REF_AGE_MS(pv);

    U32 maxRefMs = DEFAULT_REF_AGE_MS;
    if(pv == Fw::ParamValid::VALID){
        maxRefMs = rawRefMs;
    }

    pv = Fw::ParamValid::INVALID;
    const U32 rawCycles = this->paramGet_MAX_SAMPLE_AGE_CYCLES(pv);

    U32 maxCycles = DEFAULT_AGE_CYCLES;
    if(pv == Fw::ParamValid::VALID){
        maxCycles = rawCycles;
    }

    pv = Fw::ParamValid::INVALID;
    const Gnc::FrameId rawFrame = this->paramGet_REFERENCE_FRAME(pv);

    Gnc::FrameId refFrame = Gnc::FrameId::TEME;
    if (pv == Fw::ParamValid::VALID) {
        refFrame = rawFrame;
    }

    // Frame gate. Wiring or config error.
    const bool framesMatch = this->checkFrame(this->m_sunBody, "sunBody", Gnc::FrameId::BODY) &&
                             this->checkFrame(this->m_magBody, "magBody", Gnc::FrameId::BODY) &&
                             this->checkFrame(this->m_sunRef, "sunRef", refFrame) &&
                             this->checkFrame(this->m_magRef, "magRef", refFrame);

    if (!framesMatch) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::FRAME_MISMATCH, now, refFrame, false);
        return;
    }

    // Availabilty/freshness gate

    const Freshness fSunBody = this->checkFresh(this->m_sunBody, "sunBody", now, maxBodyMs, maxCycles);
    const Freshness fMagBody = this->checkFresh(this->m_magBody, "magBody", now, maxBodyMs, maxCycles);
    const Freshness fSunRef = this->checkFresh(this->m_sunRef, "sunRef", now, maxRefMs, maxCycles);
    const Freshness fMagRef = this->checkFresh(this->m_magRef, "magRef", now, maxRefMs, maxCycles);

    // Report the worst age and which time source produced it.
    U32 oldestMs = 0;
    bool anyUnknown = false;
    const Freshness* const all[] = {&fSunBody, &fMagBody, &fSunRef, &fMagRef};
    for (const Freshness* f : all) {
        if (f->ageKnown) {
            if (f->ageMs > oldestMs) {
                oldestMs = f->ageMs;
            }
        } else {
            anyUnknown = true;
        }
    }
    this->tlmWrite_OldestInputAgeMs(oldestMs);
    this->tlmWrite_UsingCycleFallback(anyUnknown);

    // Solution freshness is determined by it's oldest input, stamp with its epoch.
    Fw::Time solutionStamp = now;
    {
        const LatchedSample* const slots[] = {&this->m_sunBody, &this->m_magBody, &this->m_sunRef, &this->m_magRef};
        U32 worst = 0;
        for (U32 i = 0; i < 4U; i++) {
            if (all[i]->ageKnown && all[i]->ageMs >= worst) {
                worst = all[i]->ageMs;
                solutionStamp = slots[i]->stamp;
            }
        }
    }

    if (!fSunBody.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::SUN_UNAVAILABLE, solutionStamp, refFrame, false);
        return;
    }
    if (!fMagBody.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::MAG_UNAVAILABLE, solutionStamp, refFrame, false);
        return;
    }
    if (!fSunRef.usable || !fMagRef.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::REFERENCE_UNAVAILABLE, solutionStamp, refFrame,
                      false);
        return;
    }

    // Assign the primary TRIAD vector
    pv = Fw::ParamValid::INVALID;
    const Adcs::PrimaryVector primary = this->paramGet_PRIMARY_VECTOR(pv);

    const bool sunIsPrimary = (pv == Fw::ParamValid::VALID) && (primary == Adcs::PrimaryVector::SUN);

    TriadObservation obs;
    if (sunIsPrimary) {
        obs.primaryBody = this->m_sunBody.vec;
        obs.primaryRef = this->m_sunRef.vec;
        obs.secondaryBody = this->m_magBody.vec;
        obs.secondaryRef = this->m_magRef.vec;
    } else {
        obs.primaryBody = this->m_magBody.vec;
        obs.primaryRef = this->m_magRef.vec;
        obs.secondaryBody = this->m_sunBody.vec;
        obs.secondaryRef = this->m_sunRef.vec;
    }

    /*
     * ----------------------------------------------------------------------------
     * Run TRIAD
     * ----------------------------------------------------------------------------
     */
    TriadSolution sol;
    const TriadResult result = triadSolve(obs, this->currentConfig(), sol);

    this->tlmWrite_SeparationDeg(sol.separationRad * RAD2DEG);
    this->tlmWrite_GeometryErrDeg(sol.geometryErrorRad * RAD2DEG);

    if (result != TriadResult::OK) {
        if (result == TriadResult::BODY_COLINEAR || result == TriadResult::REFERENCE_COLINEAR) {
            this->log_WARNING_LO_VectorsColinear(sol.separationRad * RAD2DEG);

        } else if (result == TriadResult::GEOMETRY_MISMATCH) {
            this->log_WARNING_HI_GeometryMismatch(sol.geometryErrorRad * RAD2DEG);
        }

        this->publish(toFpp(this->m_lastQuat), toFppStatus(result), solutionStamp, refFrame, false);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Sign continuity:
     *
     * q and -q represent the same rotation. The solver canonicalizes to
     * w >= 0, which makes the output jump discontinuously whenever w
     * crosses zero. Pick the sign closer to the previous solution so the jump 
     * isn't misinterpreted.
     * ----------------------------------------------------------------------------
     */
    Eigen::Quaternionf q = sol.quatBodyFromRef;
    if (q.coeffs().dot(this->m_lastQuat.coeffs()) < 0.0f) {
        q.coeffs() *= -1.0f;
    }
    this->m_lastQuat = q;

    this->publish(toFpp(q), Adcs::TriadStatus::OK, solutionStamp, refFrame, true);
}

/*
 * ============================================================================
 * Publication
 * ============================================================================
 */

/**
 * \brief Emit on attitudeOut, update counters, write telemetry.
 * \param q         Quaternion to publish, [x,y,z,w]
 * \param status    Why this solution is or is not valid
 * \param stamp     Epoch of the oldest contributing input
 * \param refFrame  Reference frame the solution is relative to
 * \param valid     Whether q came from a successful solve this cycle
 */
void AttitudeDetermination ::publish(const Adcs::Quatf& q,
                                     Adcs::TriadStatus status,
                                     const Fw::Time& stamp,
                                     Gnc::FrameId refFrame,
                                     bool valid) {
    Adcs::AttitudeSolution solution;
    solution.set_q(q);
    solution.set_refFrame(refFrame);
    solution.set_status(status);
    solution.set_stamp(stamp);
    solution.set_valid(valid);

    if (this->isConnected_attitudeOut_OutputPort(0)) {
        this->attitudeOut_out(0, solution);
    }

    if (valid) {
        this->m_solutionCount++;
    } else {
        this->m_rejectCount++;
    }

    this->tlmWrite_AttQuat(q);
    this->tlmWrite_Status(status);
    this->tlmWrite_SolutionCount(this->m_solutionCount);
    this->tlmWrite_SolveRejectCount(this->m_rejectCount);

    /*
     * Edge-triggered events to prevent flooding the telemetry.
     */
    if (valid && !this->m_lastSolveValid) {
        this->log_ACTIVITY_HI_SolutionRestored();
    } else if (!valid && this->m_lastSolveValid) {
        this->log_WARNING_HI_SolutionLost(status);
    }
    this->m_lastSolveValid = valid;
}

/**
 * \brief Build a TriadConfig from the current parameter values.
 * \return Thresholds to pass to triadSolve(); unset parameters keep the
 *         TriadConfig defaults
 */
TriadConfig AttitudeDetermination ::currentConfig() {
    TriadConfig cfg;
    Fw::ParamValid pv = Fw::ParamValid::INVALID;

    const F32 minSepDeg = this->paramGet_MIN_SEPARATION_DEG(pv);
    if (pv == Fw::ParamValid::VALID) {
        cfg.minSinSeparation = std::sin(minSepDeg * DEG2RAD);
    }

    pv = Fw::ParamValid::INVALID;
    const F32 maxGeomDeg = this->paramGet_MAX_GEOMETRY_ERR_DEG(pv);
    if (pv == Fw::ParamValid::VALID) {
        cfg.maxGeometryErrorRad = maxGeomDeg * DEG2RAD;
    }

    return cfg;
}

/**
 * \brief Map the solver's enum to the FPP enum.
 * \param result  Solver result code
 * \return Equivalent Adcs::TriadStatus; DEGENERATE_INPUT for anything
 *         unrecognized
 */
Adcs::TriadStatus AttitudeDetermination ::toFppStatus(Adcs::TriadResult result) {
    switch (result) {
        case TriadResult::OK:
            return Adcs::TriadStatus::OK;
        case TriadResult::DEGENERATE_INPUT:
            return Adcs::TriadStatus::DEGENERATE_INPUT;
        case TriadResult::BODY_COLINEAR:
            return Adcs::TriadStatus::BODY_COLINEAR;
        case TriadResult::REFERENCE_COLINEAR:
            return Adcs::TriadStatus::REFERENCE_COLINEAR;
        case TriadResult::GEOMETRY_MISMATCH:
            return Adcs::TriadStatus::GEOMETRY_MISMATCH;
        case TriadResult::NOT_ORTHONORMAL:
            return Adcs::TriadStatus::NOT_ORTHONORMAL;
        default:
            return Adcs::TriadStatus::DEGENERATE_INPUT;
    }
}

/*
 * ============================================================================
 * Command handlers
 * ============================================================================
 */

/**
 * \brief Run one solve immediately, outside the rate group.
 *
 * \details Doesn't age the samples, so it reports on the
 * data present when the command arrived. Always responds OK, the
 * outcome of the solve is visible in the Status channel.
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 */
void AttitudeDetermination ::SOLVE_NOW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->solveAndPublish();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

/**
 * \brief End-to-end solver check against a synthetic known answer.
 *
 * \details Uses no flight data and doesn't disturb the latched state.
 * Responds EXECUTION_ERROR when the recovered attitude error exceeds
 * 0.01 deg, far above the float noise.
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 */
void AttitudeDetermination ::SELF_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    const Eigen::Quaternionf truth =
        Eigen::Quaternionf(Eigen::AngleAxisf(0.7f, Eigen::Vector3f(0.3f, -0.5f, 0.8f).normalized()));

    const Eigen::Vector3f sunRef(0.0f, 0.0f, 1.0f);
    const Eigen::Vector3f magRef = Eigen::Vector3f(0.6f, 0.0f, 0.8f).normalized();

    TriadObservation obs;
    obs.primaryRef = sunRef;
    obs.secondaryRef = magRef;
    obs.primaryBody = truth * sunRef;  // Eigen applies the rotation
    obs.secondaryBody = truth * magRef;

    TriadSolution sol;
    const TriadResult result = triadSolve(obs, TriadConfig(), sol);

    if (result != TriadResult::OK) {
        this->log_ACTIVITY_HI_SelfTestResult(-1.0f, false);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    const float errDeg = sol.quatBodyFromRef.angularDistance(truth) * RAD2DEG;
    const bool passed = (errDeg < 0.01f);

    this->log_ACTIVITY_HI_SelfTestResult(errDeg, passed);
    this->cmdResponse_out(opCode, cmdSeq, passed ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

}  // namespace Adcs
}  // namespace Gnc
