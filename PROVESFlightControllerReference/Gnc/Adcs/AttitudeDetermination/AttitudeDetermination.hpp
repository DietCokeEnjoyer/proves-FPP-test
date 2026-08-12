// ======================================================================
// \title  AttitudeDetermination.hpp
// \brief  hpp file for AttitudeDetermination component implementation
// ======================================================================
#ifndef Gnc_Adcs_AttitudeDetermination_HPP
#define Gnc_Adcs_AttitudeDetermination_HPP

#include "Gnc/Adcs/AttitudeDetermination/AttitudeDeterminationComponentAc.hpp"
#include "Gnc/Adcs/AttitudeDetermination/TriadSolver.hpp"

namespace Gnc {
namespace Adcs {

class AttitudeDetermination final : public AttitudeDeterminationComponentBase {
  public:
    explicit AttitudeDetermination(const char* const compName);
    ~AttitudeDetermination();

  private:
    // ------------------------------------------------------------------
    // Latched input state
    //
    // Each producer runs on its own thread and pushes at its own rate.
    // We keep only the newest sample plus its timestamp, so a fast
    // magnetometer never backs up behind a slow orbit propagator and
    // the solve always uses the freshest available data.
    // ------------------------------------------------------------------
    struct LatchedSample {
        Eigen::Vector3f vec = Eigen::Vector3f::Zero();
        Fw::Time stamp;                                   //!< when observed
        Gnc::FrameId frame = Gnc::FrameId::UNKNOWN;
        U32 ageCycles = 0xFFFFFFFFU;                      //!< fallback metric
        bool valid = false;                               //!< producer's flag
        bool everSet = false;
    };

    //! Per-input freshness verdict, kept so the caller can report the
    //! worst age without recomputing it.
    struct Freshness {
        bool usable = false;
        U32 ageMs = 0;
        bool ageKnown = false;
    };

    // ------------------------------------------------------------------
    // Handlers for user-defined typed input ports
    // ------------------------------------------------------------------
    void sunBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;
    void magBodyIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;
    void sunRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;
    void magRefIn_handler(FwIndexType portNum, Gnc::VectorSample& sample) override;
    void schedIn_handler(FwIndexType portNum, U32 context) override;

    // ------------------------------------------------------------------
    // Command handlers
    // ------------------------------------------------------------------
    void SOLVE_NOW_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
    void SELF_TEST_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    //! Store a newly arrived sample and reset its age
    void latch(LatchedSample& slot, const Gnc::VectorSample& sample);

    //! Check the frame tag. Emits FrameMismatch and returns false on
    //! disagreement.
    bool checkFrame(const LatchedSample& slot, const char* name, Gnc::FrameId expected);

    //! Check freshness, preferring wall clock and falling back to cycle
    //! counting when the timestamps cannot be differenced.
    Freshness checkFresh(const LatchedSample& slot,
                         const char* name,
                         const Fw::Time& now,
                         U32 maxAgeMs,
                         U32 maxAgeCycles);

    //! Assemble inputs, run TRIAD, publish, telemeter. The whole cycle.
    void solveAndPublish();

    //! Emit the result on the output port and update telemetry
    void publish(const Adcs::Quatf& q,
                 Adcs::TriadStatus status,
                 const Fw::Time& stamp,
                 Gnc::FrameId refFrame,
                 bool valid);

    //! Map the solver's enum onto the FPP enum used on the wire
    static Adcs::TriadStatus toFppStatus(Adcs::TriadResult result);

    //! Build a TriadConfig from the current parameter values
    TriadConfig currentConfig();

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    LatchedSample m_sunBody;
    LatchedSample m_magBody;
    LatchedSample m_sunRef;
    LatchedSample m_magRef;

    //! Previous published quaternion, used only to keep the sign of the
    //! new one continuous with it (q and -q are the same rotation, but a
    //! sign flip looks like a 360 deg jump to a downstream filter).
    Eigen::Quaternionf m_lastQuat = Eigen::Quaternionf::Identity();

    //! Latch so SolutionLost / SolutionRestored fire on edges only
    bool m_lastSolveValid = false;

    U32 m_solutionCount = 0;
    U32 m_rejectCount = 0;
};

}  // namespace Adcs
}  // namespace Gnc

#endif
