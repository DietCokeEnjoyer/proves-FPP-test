/**
 * \file AttitudeDetermination.cpp
 * \brief cpp file for AttitudeDetermination component implementation
 *
 * \details This file is the FRAMEWORK layer. It owns latching, staleness, frame
 * checking, parameters, telemetry, events, and fault reporting. It
 * contains no TRIAD math -- that all lives in TriadSolver.cpp, which
 * knows nothing about F Prime and can be unit tested on a workstation.
 */

#include "Gnc/Adcs/AttitudeDetermination/AttitudeDetermination.hpp"

#include <cmath>

namespace Gnc {
namespace Adcs {

namespace {

/**
 * Defaults used when a parameter has never been set. These must match
 * the FPP defaults; they exist because paramGet returns garbage-safe
 * but meaningless values when ParamValid is not VALID, and silently
 * using a zero age limit would suppress every solution.
 */
constexpr U32 DEFAULT_BODY_AGE_MS = 500U;
constexpr U32 DEFAULT_REF_AGE_MS = 3000U;
constexpr U32 DEFAULT_AGE_CYCLES = 20U;

/**
 * FPP struct -> Eigen. Kept here, not in GncConvert.hpp: Eigen is an
 * attitude-domain dependency and must not leak into the shared types.
 */
Eigen::Vector3f toEigen(const Gnc::Vec3f& v) {
    return Eigen::Vector3f(v.get_x(), v.get_y(), v.get_z());
}

//! Eigen quaternion -> FPP array, preserving the [x,y,z,w] order
Adcs::Quatf toFpp(const Eigen::Quaternionf& q) {
    Adcs::Quatf out;
    out[0] = q.x();
    out[1] = q.y();
    out[2] = q.z();
    out[3] = q.w();
    return out;
}

/**
 * Age in milliseconds, or false if the two times cannot be meaningfully
 * differenced.
 *
 * Returning false rather than a large number is the important part: a
 * spacecraft that has not acquired time yet, or that just stepped its
 * clock after a GPS fix, must fall back to cycle counting instead of
 * concluding that every input is infinitely stale. That is precisely
 * the regime -- first minutes after boot -- where a coarse attitude
 * solution is most needed.
 */
bool ageMs(const Fw::Time& now, const Fw::Time& then, U32& out) {
    if (now.getTimeBase() != then.getTimeBase()) {
        return false;
    }
    if (now.getTimeBase() == Fw::TimeBase::TB_NONE) {
        return false;
    }

    const I64 d = (static_cast<I64>(now.getSeconds()) - static_cast<I64>(then.getSeconds())) * 1000LL
                + (static_cast<I64>(now.getUSeconds()) - static_cast<I64>(then.getUSeconds())) / 1000LL;

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

AttitudeDetermination ::AttitudeDetermination(const char* const compName)
    : AttitudeDeterminationComponentBase(compName) {}

AttitudeDetermination ::~AttitudeDetermination() {}

/*
 * ============================================================================
 * Input port handlers
 *
 * These are GUARDED, so the framework holds the component mutex for the
 * duration. Each handler does the minimum possible work -- copy and
 * stamp -- and returns, so a producer thread is never blocked behind a
 * solve. All four are identical apart from which slot they write.
 * ============================================================================
 */

void AttitudeDetermination ::sunBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_sunBody, sample);
}

void AttitudeDetermination ::magBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_magBody, sample);
}

void AttitudeDetermination ::sunRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_sunRef, sample);
}

void AttitudeDetermination ::magRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) {
    (void)portNum;
    this->latch(this->m_magRef, sample);
}

void AttitudeDetermination ::latch(LatchedSample& slot, const Gnc::VectorSample& sample) {
    slot.vec = toEigen(sample.get_vec());
    slot.stamp = sample.get_stamp();
    slot.frame = sample.get_frame();
    slot.valid = sample.get_valid();
    slot.ageCycles = 0;
    slot.everSet = true;
}

/*
 * ============================================================================
 * Rate group tick
 *
 * One tick == age everything by one cycle, then attempt one solve.
 * ============================================================================
 */

void AttitudeDetermination ::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    /*
     * Saturating increment so a long dropout cannot wrap the counter
     * back around into "fresh".
     */
    LatchedSample* const slots[] = {&this->m_sunBody, &this->m_magBody,
                                    &this->m_sunRef, &this->m_magRef};
    for (LatchedSample* slot : slots) {
        if (slot->ageCycles < 0xFFFFFFFFU) {
            slot->ageCycles++;
        }
    }

    this->solveAndPublish();
}

/*
 * ============================================================================
 * Frame checking
 *
 * This is the check that could not exist before the refactor, because
 * the samples carried no frame tag. It matters because TRIAD cannot
 * detect a frame error numerically: if both reference vectors are
 * expressed in J2000 while the component believes they are TEME, they
 * are both rotated by the same 0.36 deg of precession, the angle
 * between them is unchanged, and the geometry consistency check in
 * STEP 3 of the solver passes cleanly. The attitude is then wrong by
 * 0.36 deg with every health channel reading nominal.
 * ============================================================================
 */

bool AttitudeDetermination ::checkFrame(const LatchedSample& slot,
                                        const char* name,
                                        Gnc::FrameId expected) {
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

/*
 * ============================================================================
 * Freshness
 * ============================================================================
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

/*
 * ============================================================================
 * The main cycle
 * ============================================================================
 */

void AttitudeDetermination ::solveAndPublish() {
    const Fw::Time now = this->getTime();

    Fw::ParamValid pv = Fw::ParamValid::INVALID;

    const U32 rawBodyMs = this->paramGet_MAX_BODY_AGE_MS(pv);
    const U32 maxBodyMs = (pv == Fw::ParamValid::VALID) ? rawBodyMs : DEFAULT_BODY_AGE_MS;

    pv = Fw::ParamValid::INVALID;
    const U32 rawRefMs = this->paramGet_MAX_REF_AGE_MS(pv);
    const U32 maxRefMs = (pv == Fw::ParamValid::VALID) ? rawRefMs : DEFAULT_REF_AGE_MS;

    pv = Fw::ParamValid::INVALID;
    const U32 rawCycles = this->paramGet_MAX_SAMPLE_AGE_CYCLES(pv);
    const U32 maxCycles = (pv == Fw::ParamValid::VALID) ? rawCycles : DEFAULT_AGE_CYCLES;

    pv = Fw::ParamValid::INVALID;
    const Gnc::FrameId rawFrame = this->paramGet_REFERENCE_FRAME(pv);
    const Gnc::FrameId refFrame =
        (pv == Fw::ParamValid::VALID) ? rawFrame : Gnc::FrameId::TEME;

    /*
     * ----------------------------------------------------------------------------
     * Frame gate, before anything else. A frame fault is a wiring or
     * configuration error: it will not clear on its own, and continuing
     * to publish a confident attitude through it is worse than
     * publishing nothing.
     * ----------------------------------------------------------------------------
     */
    const bool framesMatch =
        this->checkFrame(this->m_sunBody, "sunBody", Gnc::FrameId::BODY) &&
        this->checkFrame(this->m_magBody, "magBody", Gnc::FrameId::BODY) &&
        this->checkFrame(this->m_sunRef, "sunRef", refFrame) &&
        this->checkFrame(this->m_magRef, "magRef", refFrame);

    if (!framesMatch) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::FRAME_MISMATCH,
                      now, refFrame, false);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Availability gate. Distinguish the failure reasons so operators
     * can tell an eclipse (expected, recurring, benign) from a broken
     * magnetometer (not benign) straight from the Status channel.
     * ----------------------------------------------------------------------------
     */
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

    /*
     * The solution is only as fresh as its stalest input, so stamp it
     * with the oldest one rather than with "now".
     */
    Fw::Time solutionStamp = now;
    {
        const LatchedSample* const slots[] = {&this->m_sunBody, &this->m_magBody,
                                              &this->m_sunRef, &this->m_magRef};
        U32 worst = 0;
        for (U32 i = 0; i < 4U; i++) {
            if (all[i]->ageKnown && all[i]->ageMs >= worst) {
                worst = all[i]->ageMs;
                solutionStamp = slots[i]->stamp;
            }
        }
    }

    if (!fSunBody.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::SUN_UNAVAILABLE,
                      solutionStamp, refFrame, false);
        return;
    }
    if (!fMagBody.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::MAG_UNAVAILABLE,
                      solutionStamp, refFrame, false);
        return;
    }
    if (!fSunRef.usable || !fMagRef.usable) {
        this->publish(toFpp(this->m_lastQuat), Adcs::TriadStatus::REFERENCE_UNAVAILABLE,
                      solutionStamp, refFrame, false);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Assign the primary / secondary roles.
     *
     * This is the one genuinely consequential configuration choice in
     * TRIAD. The primary observation is reproduced EXACTLY by the
     * solution; the secondary only fixes rotation about it. So the
     * primary must be the more accurate sensor. Normally that is the
     * sun sensor, but if it degrades, ground can flip this parameter
     * and TRIAD keeps working with the magnetometer as the trusted leg.
     * ----------------------------------------------------------------------------
     */
    pv = Fw::ParamValid::INVALID;
    const Adcs::PrimaryVector primary = this->paramGet_PRIMARY_VECTOR(pv);

    /*
     * The not-VALID fallback MUST match the FPP default. This
     * deployment uses Components::NullPrmDb, so paramGet never returns
     * VALID and this branch is the one that actually flies -- a
     * mismatch here would silently invert the most consequential
     * configuration choice in TRIAD while the FPP file said otherwise.
     */
    const bool sunIsPrimary =
        (pv == Fw::ParamValid::VALID) && (primary == Adcs::PrimaryVector::SUN);

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
        /*
         * Specific diagnostics for the two failures an operator can
         * actually act on.
         */
        if (result == TriadResult::BODY_COLINEAR || result == TriadResult::REFERENCE_COLINEAR) {
            this->log_WARNING_LO_VectorsColinear(sol.separationRad * RAD2DEG);
        } else if (result == TriadResult::GEOMETRY_MISMATCH) {
            this->log_WARNING_HI_GeometryMismatch(sol.geometryErrorRad * RAD2DEG);
        }
        this->publish(toFpp(this->m_lastQuat), toFppStatus(result),
                      solutionStamp, refFrame, false);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Sign continuity.
     *
     * q and -q represent the same rotation. The solver canonicalizes to
     * w >= 0, which makes the output jump discontinuously whenever w
     * crosses zero. Anything that differentiates the quaternion or
     * feeds it to a filter will read that as an enormous slew. Pick the
     * sign nearer the previous solution instead.
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
 * Publication and bookkeeping
 * ============================================================================
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
     * Edge-triggered events only. A rate group running at 10 Hz through
     * a 20 minute eclipse would otherwise generate 12000 identical
     * events and flood the downlink.
     */
    if (valid && !this->m_lastSolveValid) {
        this->log_ACTIVITY_HI_SolutionRestored();
    } else if (!valid && this->m_lastSolveValid) {
        this->log_WARNING_HI_SolutionLost(status);
    }
    this->m_lastSolveValid = valid;
}

TriadConfig AttitudeDetermination ::currentConfig() {
    TriadConfig cfg;
    Fw::ParamValid pv = Fw::ParamValid::INVALID;

    const F32 minSepDeg = this->paramGet_MIN_SEPARATION_DEG(pv);
    if (pv == Fw::ParamValid::VALID) {
        /*
         * The solver thresholds on sin(separation) because that is the
         * quantity that actually appears in the denominator.
         */
        cfg.minSinSeparation = std::sin(minSepDeg * DEG2RAD);
    }

    pv = Fw::ParamValid::INVALID;
    const F32 maxGeomDeg = this->paramGet_MAX_GEOMETRY_ERR_DEG(pv);
    if (pv == Fw::ParamValid::VALID) {
        cfg.maxGeometryErrorRad = maxGeomDeg * DEG2RAD;
    }

    return cfg;
}

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

void AttitudeDetermination ::SOLVE_NOW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    this->solveAndPublish();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void AttitudeDetermination ::SELF_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    /*
     * ----------------------------------------------------------------------------
     * End-to-end check with a known answer.
     *
     * Rotate two arbitrary reference vectors by a truth attitude to
     * synthesize "measurements", run TRIAD, and compare. In exact
     * arithmetic the recovered error is zero, so anything above float
     * noise means the FPU is not enabled, the compiler flags are wrong,
     * or the library was miscompiled. Worth running once after every
     * load, and after any radiation event.
     * ----------------------------------------------------------------------------
     */
    const Eigen::Quaternionf truth =
        Eigen::Quaternionf(Eigen::AngleAxisf(0.7f, Eigen::Vector3f(0.3f, -0.5f, 0.8f).normalized()));

    const Eigen::Vector3f sunRef(0.0f, 0.0f, 1.0f);
    const Eigen::Vector3f magRef = Eigen::Vector3f(0.6f, 0.0f, 0.8f).normalized();

    TriadObservation obs;
    obs.primaryRef = sunRef;
    obs.secondaryRef = magRef;
    obs.primaryBody = truth * sunRef;    // Eigen applies the rotation
    obs.secondaryBody = truth * magRef;

    TriadSolution sol;
    const TriadResult result = triadSolve(obs, TriadConfig(), sol);

    if (result != TriadResult::OK) {
        this->log_ACTIVITY_HI_SelfTestResult(-1.0f, false);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::EXECUTION_ERROR);
        return;
    }

    /*
     * angularDistance is the actual rotation angle between the two
     * attitudes -- the correct error metric, and sign-agnostic.
     */
    const float errDeg = sol.quatBodyFromRef.angularDistance(truth) * RAD2DEG;
    const bool passed = (errDeg < 0.01f);

    this->log_ACTIVITY_HI_SelfTestResult(errDeg, passed);
    this->cmdResponse_out(opCode, cmdSeq,
                          passed ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

}  // namespace Adcs
}  // namespace Gnc
