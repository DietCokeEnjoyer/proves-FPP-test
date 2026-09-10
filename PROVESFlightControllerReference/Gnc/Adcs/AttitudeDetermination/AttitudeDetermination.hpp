/**
 * \file AttitudeDetermination.hpp
 * \brief hpp file for AttitudeDetermination component implementation
 */
#ifndef Gnc_Adcs_AttitudeDetermination_HPP
#define Gnc_Adcs_AttitudeDetermination_HPP

#include "PROVESFlightControllerReference/Gnc/Adcs/AttitudeDetermination/AttitudeDeterminationComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/Adcs/AttitudeDetermination/TriadSolver.hpp"

namespace Gnc {
namespace Adcs {

/**
 * \brief Framework wrapper around the TRIAD solver.
 *
 * \details Latches four vector inputs (sun and magnetic field, each in
 * body and reference frames), gates them on frame tag and freshness,
 * runs TRIAD once per rate group tick, and publishes an
 * Adcs::AttitudeSolution.
 *
 */
class AttitudeDetermination final : public AttitudeDeterminationComponentBase {
  public:
    /**
     * \brief Construct the component.
     * \param compName  F Prime component instance name
     */
    explicit AttitudeDetermination(const char* const compName);

    //! Destroy the component. Holds no resources.
    ~AttitudeDetermination();

  private:
    /*
     * ============================================================================
     * Latched input state
     *
     * Each producer runs on its own thread and pushes at its own rate.
     * Keeps only the newest sample so the solve always uses the freshest available data.
     * ============================================================================
     */

    /**
     * \brief Newest sample received on one input port, plus its metadata.
     */
    struct LatchedSample {
        Eigen::Vector3f vec = Eigen::Vector3f::Zero();    //!< the sample 
        Fw::Time stamp;                                   //!< observation time
        Gnc::FrameId frame = Gnc::FrameId::UNKNOWN;       //!< producer's frame tag
        U32 ageCycles = 0xFFFFFFFFU;                      //!< fallback metric
        bool valid = false;                               //!< producer's flag
        bool everSet = false;
    };

    /**
     * Per-input freshness verdict, kept so the caller can report the
     * worst age without recomputing it.
     */
    struct Freshness {
        bool usable = false;    //!< within the age limit and flagged valid
        U32 ageMs = 0;          //!< age in ms; meaningless unless ageKnown
        bool ageKnown = false;  //!< false when the cycle-count fallback was used
    };

    /*
     * ============================================================================
     * Handlers for user-defined typed input ports
     * ============================================================================
     */

    /**
     * \brief Latch a measured sun vector in the body frame.
     * \param portNum  Port index, unused (single port)
     * \param sample   Sample from the sun sensor driver, expected FrameId::BODY
     */
    void sunBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;

    /**
     * \brief Latch a measured magnetic field vector in the body frame.
     * \param portNum  Port index, unused (single port)
     * \param sample   Sample from the magnetometer driver, expected FrameId::BODY
     */
    void magBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;

    /**
     * \brief Latch the modelled sun direction in the reference frame.
     * \param portNum  Port index, unused (single port)
     * \param sample   Sample from SolarEphemeris, expected to match
     *                 the REFERENCE_FRAME parameter
     */
    void sunRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;

    /**
     * \brief Latch the modelled magnetic field direction in the
     *        reference frame.
     * \param portNum  Port index, unused (single port)
     * \param sample   Sample from MagneticFieldModel, expected to match
     *                 the REFERENCE_FRAME parameter
     */
    void magRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;

    /**
     * \brief Rate group tick: age every latched sample by one cycle,
     *        then attempt one solve.
     * \param portNum  Port index, unused (single port)
     * \param context  Rate group context, unused
     */
    void run_handler(FwIndexType portNum, U32 context) override;

    /*
     * ============================================================================
     * Command handlers
     * ============================================================================
     */
    /**
     * \brief Run one solve immediately
     *
     * \details Doesn't age the samples, so it reports on the
     * data present when the command arrived. Always responds OK, the
     * outcome of the solve is visible in the Status channel.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void SOLVE_NOW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /**
     * \brief End-to-end solver check against a synthetic known answer.
     *
     * \details Rotates two fixed reference vectors by a hard-coded truth
     * attitude, runs TRIAD, and compares. Uses no flight data and doesn't 
     * disturb the latched state. A failure means the FPU, the
     * compiler flags, or the library build is wrong.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void SELF_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /*
     * ============================================================================
     * Internal helpers
     * ============================================================================
     */

    /**
     * \brief Store a newly arrived sample and reset its age.
     *
     * \details Just stores the sample. Validation happens at solve.
     *
     * \param slot    [out] Latch slot to overwrite
     * \param sample  Incoming sample
     */
    void latch(LatchedSample& slot, const Gnc::VectorSample& sample);

    /**
     * \brief Check a latched sample's frame tag against what is expected.
     *
     * \details A slot that hasn't received anything passes, staleness will
     * catch it instead.
     *
     * \param slot      Latch slot to check
     * \param name      Slot name, for the FrameMismatch event
     * \param expected  Frame the producer is required to have tagged
     * \return true when the tag matches or nothing has arrived yet;
     *         false after emitting WARNING_HI FrameMismatch
     */
    bool checkFrame(const LatchedSample& slot, const char* name, Gnc::FrameId expected);

    /**
     * \brief Check a latched sample's freshness.
     *
     * \details Uses RTC when possible, falls back to cycle counting when necessary.
     *
     * \param slot          Latch slot to check
     * \param name          Slot name, for the SampleStale event
     * \param now           Current time, read once per solve
     * \param maxAgeMs      Age limit when the wall clock is usable
     * \param maxAgeCycles  Age limit in rate group cycles, for the fallback
     * \return Verdict plus the measured age. Emits WARNING_LO SampleStale
     *         when the sample is unusable.
     */
    Freshness checkFresh(const LatchedSample& slot,
                         const char* name,
                         const Fw::Time& now,
                         U32 maxAgeMs,
                         U32 maxAgeCycles);

    /**
     * \brief Assemble inputs, run TRIAD, publish, telemeter.
     *
     * \details Gates in order: frame tags, then availability and
     * freshness, then the solver's own checks. Early exits still
     * publish the last good solution and explain the rejection.
     */
    void solveAndPublish();

    /**
     * \brief Emit the result on the output port and update telemetry.
     *
     * \details Also maintains the solution and reject counters and fires
     * SolutionLost / SolutionRestored on edges only, so a long eclipse
     * doesn't flood the downlink with identical events.
     *
     * \param q         Quaternion to publish, [x,y,z,w]
     * \param status    Why this solution is or is not valid
     * \param stamp     Epoch of the oldest contributing input
     * \param refFrame  Reference frame the solution is relative to
     * \param valid     Whether q came from a successful solve this cycle
     */
    void publish(const Adcs::Quatf& q,
                 Adcs::TriadStatus status,
                 const Fw::Time& stamp,
                 Gnc::FrameId refFrame,
                 bool valid);

    /**
     * \brief Map the solver's enum onto the FPP enum used on the wire.
     *
     * \details 
     *
     * \param result  Solver result code
     * \return Equivalent Adcs::TriadStatus
     */
    static Adcs::TriadStatus toFppStatus(Adcs::TriadResult result);

    /**
     * \brief Build a TriadConfig from the current parameter values.
     *
     * \details Fields whose parameter is not VALID keep the TriadConfig
     * default. MIN_SEPARATION_DEG is converted to its sine here, because
     * that's the quantity used in the solver's denominator.
     *
     * \return Thresholds to pass to triadSolve()
     */
    TriadConfig currentConfig();

    /*
     * ============================================================================
     * State
     * ============================================================================
     */
    LatchedSample m_sunBody;  //!< Measured sun vector, body frame
    LatchedSample m_magBody;  //!< Measured B-field, body frame
    LatchedSample m_sunRef;   //!< Modelled sun direction, reference frame
    LatchedSample m_magRef;   //!< Modelled B-field direction, reference frame

    /**
     * Previous published quaternion, used to keep the sign of the
     * new one continuous with it.
     */
    Eigen::Quaternionf m_lastQuat = Eigen::Quaternionf::Identity();

    //! Latch so SolutionLost / SolutionRestored fire on edges only
    bool m_lastSolveValid = false;

    U32 m_solutionCount = 0;  //!< Cumulative successful solves
    U32 m_rejectCount = 0;    //!< Cumulative rejected solves, any reason
};

}  // namespace Adcs
}  // namespace Gnc

#endif
