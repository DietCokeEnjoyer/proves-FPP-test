// ======================================================================
// \file   SolarEphemeris.hpp
// \brief  Solar direction, eclipse state and beta angle.
// ======================================================================
#ifndef Gnc_Environment_SolarEphemeris_HPP
#define Gnc_Environment_SolarEphemeris_HPP

#include "Gnc/Environment/SolarEphemeris/SolarEphemerisComponentAc.hpp"
#include "Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

class SolarEphemeris final : public SolarEphemerisComponentBase {
  public:
    explicit SolarEphemeris(const char* const compName);
    ~SolarEphemeris();

  private:
    //! Orbit state arrived. Drives one evaluation.
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;

    void RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    //! Solar direction in TEME, with interpolation between full
    //! recomputations. Owns the segment cache.
    Astro::Vec3 solarDirectionTeme(F64 jdUt1, F64& rangeKm);

    //! Evaluate the solar model and rotate MOD -> TEME at one instant.
    static void sunTemeAt(F64 tUt1, F64 tTt, Astro::Vec3& unitTeme, F64& rangeKm);

    //! Publish on both output ports and update telemetry.
    void emit(const Gnc::SolarState& solar, const Fw::Time& stamp);

    // ------------------------------------------------------------------
    // Interpolation cache
    //
    // Endpoints of the current segment, in TEME. Touched by orbitIn and
    // by RESYNC_SUN, both of which are guarded.
    // ------------------------------------------------------------------
    Astro::Vec3 m_sunPrev;
    Astro::Vec3 m_sunNext;
    F64         m_sunRangePrevKm;
    F64         m_sunRangeNextKm;
    F64         m_sunPrevJd;
    F64         m_sunNextJd;
    bool        m_sunSeeded;

    //! Previous illumination, for edge-triggered eclipse events.
    Astro::Illumination m_lastIllum;
    bool                m_illumSeeded;
};

}  // namespace Environment
}  // namespace Gnc

#endif
