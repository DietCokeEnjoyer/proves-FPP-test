/**
 * \file OrbitPropagator.cpp
 * \brief Gets the time stamp needed for all GNC computations, computes SGP4, and broadcasts the OrbitState.
 *
 * \details FPrime layer: handles time acquisition, parameters,
 * commands, telemetry, and events. Propagation is computed in Sgp4Propagator.cpp.
 * 
 * Per-tick pipeline:
 *  1. Acquire the timestamp: acquireTime() 
 *  2. Propagate the orbit: solveOrbit()
 *  3. Publish telemetry: publish()
 *  4. Push the OrbitState to consumers: emit()  
 */
#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/OrbitPropagator.hpp"

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Construct the component with no TLE loaded.
 * \param compName  F Prime component instance name
 */
OrbitPropagator::OrbitPropagator(const char* const compName) : OrbitPropagatorComponentBase(compName) {}

//! Destroy the component. Holds no resources.
OrbitPropagator::~OrbitPropagator() {}

/**
 * \brief Rate group tick. Runs all four stages and times itself.
 *
 * \details Degrades in steps rather than all at once. NO_TIME stops
 * everything. NO_TLE still publishes time and GMST. PROP_ERROR does the same. 
 * STALE publishes a full solution and flags it.
 *
 * The CycleUsec measurement at the end covers the execution cost of the entire 
 * AD chain and not just of this component.
 *
 * \param portNum  Port index, unused
 * \param context  Rate group context, unused
 */
void OrbitPropagator::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    const Fw::Time tStart = this->getTime();

    /*
     * ----------------------------------------------------------------------------
     * Stage 1: Time Acquisition
     * ----------------------------------------------------------------------------
     */
    Fw::Time stamp;
    F64 unixSeconds = 0.0;

    // Still emit invalid state for visibility
    if (!this->acquireTime(stamp, unixSeconds)) {
        Gnc::OrbitState state;
        state.set_validity(Gnc::OrbitValidity::NO_TIME);
        state.set_timeUsable(false);
        state.set_positionUsable(false);

        this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TIME);
        this->emit(state);
        return;
    }

    /*
     * ----------------------------------------------------------------------------
     * Stage 2: time scales, GMST, SGP4 and the ground track
     * ----------------------------------------------------------------------------
     */
    OrbitSolution sol;
    const OrbitResult result = solveOrbit(unixSeconds, this->currentConfig(), m_sgp4, sol);

    Gnc::OrbitState state;
    fillState(sol, result, stamp, state);

    // GMST, computed once for the whole subsystem.
    this->tlmWrite_GmstDeg(static_cast<F32>(sol.gmstRad * Astro::RAD2DEG));

    switch (result) {
        case OrbitResult::NO_TLE:
            this->log_WARNING_LO_TleMissing();
            this->tlmWrite_Validity(Gnc::OrbitValidity::NO_TLE);
            this->emit(state);
            return;

        case OrbitResult::PROP_ERROR:
            this->log_WARNING_HI_Sgp4Failure(sol.sgp4Code, sol.minsFromEpoch);
            this->tlmWrite_Validity(Gnc::OrbitValidity::PROP_ERROR);
            this->emit(state);
            return;

        case OrbitResult::STALE:
            // SGP4 accuracy degrades 1-3 km per day past TLE epoch.
            this->log_WARNING_LO_TleStale(static_cast<F32>(sol.tleAgeDays));
            break;

        case OrbitResult::VALID:
        default:
            break;
    }

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
 * \brief Stage 1. Read the clock and decide whether it is usable.
 *
 * \details Rejects anything that isn't wall-clock time. The scale
 * conversions happen in solveOrbit(),  this just reads
 * the port and produces POSIX seconds.
 *
 * \param stamp        [out] The Fw::Time read
 * \param unixSeconds  [out] The same instant as POSIX seconds
 * \return true when time was usable; false after WARNING_HI TimeMissing
 */
bool OrbitPropagator::acquireTime(Fw::Time& stamp, F64& unixSeconds) {
    stamp = this->getTime();
    const TimeBase::T base = stamp.getTimeBase();
    /*
     * Must be wall-clock time, not uptime.
     * Timebase is TB_WORKSTATION_TIME when time is successfully acquired from the RTC.
     */
    if (base != TimeBase::TB_WORKSTATION_TIME && base != TimeBase::TB_SC_TIME) {
        this->log_WARNING_HI_TimeMissing();
        return false;
    }

    unixSeconds = static_cast<F64>(stamp.getSeconds()) + static_cast<F64>(stamp.getUSeconds()) * 1.0e-6;
    return true;
}

/**
 * \brief Read the time-scale and staleness parameters into an OrbitConfig.
 *
 * \details
 *
 * \return Config for solveOrbit(), defaulted per field where the
 *         parameter is not VALID
 */
OrbitConfig OrbitPropagator::currentConfig() {
    OrbitConfig cfg;
    Fw::ParamValid valid = Fw::ParamValid::INVALID;

    /*
     * UTC_OFFSET_SEC exists so a GPS-time or TAI-time source can be
     * shifted onto the UTC scale.
     */
    const F64 utcOffset = this->paramGet_UTC_OFFSET_SEC(valid);
    if (valid == Fw::ParamValid::VALID) {
        cfg.utcOffsetSec = utcOffset;
    }

    valid = Fw::ParamValid::INVALID;
    const F64 leap = this->paramGet_LEAP_SEC(valid);
    if (valid == Fw::ParamValid::VALID) {
        cfg.leapSec = leap;
    }

    valid = Fw::ParamValid::INVALID;
    const F64 dut1 = this->paramGet_DUT1_SEC(valid);
    if (valid == Fw::ParamValid::VALID) {
        cfg.dut1Sec = dut1;
    }

    valid = Fw::ParamValid::INVALID;
    const F32 maxAge = this->paramGet_MAX_TLE_AGE_DAYS(valid);
    if (valid == Fw::ParamValid::VALID) {
        cfg.maxTleAgeDays = static_cast<F64>(maxAge);
    }

    return cfg;
}

/**
 * \brief Pack the information from an OrbitSolution into an OrbitState to be broadcast.
 *
 * \param sol     Products of this cycle
 * \param result  How far the cycle got
 * \param stamp   Wall-clock instant the solution describes
 * \param state   [out] State to populate
 */
void OrbitPropagator::fillState(const OrbitSolution& sol,
                                OrbitResult result,
                                const Fw::Time& stamp,
                                Gnc::OrbitState& state) {
    state.set_validity(toFppValidity(result));

    // Time already validated
    state.set_stamp(stamp);
    state.set_timeUsable(true);
    state.set_jdUt1(sol.jdUt1);

    /*
     * TT as a Julian date. Carried for solar ephemeris.
     */
    state.set_jdTt(sol.jdTt);
    state.set_gmstRad(sol.gmstRad);

    const bool hasPosition = (result == OrbitResult::VALID) || (result == OrbitResult::STALE);
    state.set_positionUsable(hasPosition);
    if (!hasPosition) {
        return;
    }

    state.set_posTemeKm(toVec3d(sol.posTemeKm));
    state.set_velTemeKmS(toVec3d(sol.velTemeKmS));
    state.set_latRad(sol.latRad);
    state.set_lonRad(sol.lonRad);
    state.set_altKm(sol.altKm);
    state.set_tleAgeDays(static_cast<F32>(sol.tleAgeDays));
}

/**
 * \brief Stage 3. Write this cycle's telemetry.
 *
 * \details Only called on the fully valid path.
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
 * \brief Stage 4. Broadcast the orbit state.
 *
 * \details Copies because the output port takes a mutable reference.
 *
 * \param state  Orbit state to broadcast
 */
void OrbitPropagator::emit(const Gnc::OrbitState& state) {
    
    const FwIndexType ports = getNum_orbitOut_OutputPorts();

    for (FwIndexType p = 0; p < ports; ++p) {
        if (this->isConnected_orbitOut_OutputPort(p)) {
            Gnc::OrbitState copy = state;
            this->orbitOut_out(p, copy);
        }
    }
}

/**
 * \brief Map OrbitResult onto the FPP validity enum.
 *
 * \param result  Propagator result code
 * \return Equivalent Gnc::OrbitValidity
 */
Gnc::OrbitValidity OrbitPropagator::toFppValidity(OrbitResult result) {
    switch (result) {
        case OrbitResult::VALID:
            return Gnc::OrbitValidity::VALID;
        case OrbitResult::STALE:
            return Gnc::OrbitValidity::STALE;
        case OrbitResult::PROP_ERROR:
            return Gnc::OrbitValidity::PROP_ERROR;
        case OrbitResult::NO_TLE:
            return Gnc::OrbitValidity::NO_TLE;
        case OrbitResult::NO_TIME:
        default:
            return Gnc::OrbitValidity::NO_TIME;
    }
}

/*
 * ============================================================================
 * Commands
 * ============================================================================
 */

/**
 * \brief Parse an uplinked TLE and install it if it is well formed.
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
    U8 code = 0;
    
    // Failures leave already loaded TLE in place.
    if (!m_sgp4.loadTle(line1.toChar(), line2.toChar(), code)) {
        this->log_WARNING_HI_TleRejected(code);
        this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
        return;
    }

    char satnum[Sgp4Propagator::SATNUM_BUF_LEN];
    m_sgp4.satnum(satnum, sizeof satnum);

    const Fw::String satnumArg(satnum);
    this->log_ACTIVITY_HI_TleAccepted(satnumArg, m_sgp4.epochJd());
    
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

/**
 * \brief Mark the loaded TLE unusable.
 *
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 */
void OrbitPropagator::CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_sgp4.clearTle();
    this->log_ACTIVITY_HI_TleCleared();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
