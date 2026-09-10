/**
 * \file SolarEphemeris.hpp
 * 
 * \brief Solar direction, eclipse state and beta angle.
 */
#ifndef Gnc_Environment_SolarEphemeris_HPP
#define Gnc_Environment_SolarEphemeris_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SolarEphemerisComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Solar direction, eclipse state and beta angle.
 *
 * \details Driven by the arrival of OrbitState. This component never
 * reads the clock itself. The work is split in two: the Sun direction needs
 * only a time, while parallax, shadow and beta need a position. The
 * split lets the component produce a usable direction when the TLE is invalid or missing.
 */
class SolarEphemeris final : public SolarEphemerisComponentBase {
  public:
    /**
     * \brief Construct the component with an empty interpolation cache.
     * \param compName  F Prime component instance name
     */
    explicit SolarEphemeris(const char* const compName);

    //! Destroy the component. Holds no resources.
    ~SolarEphemeris();

  private:
    /**
     * \brief Orbit state arrived. Triggers one evaluation.
     *
     * \details Only an unusable clock stops the evaluation. An
     * unusable position downgrades to a geocentric solution, which is
     * good enough for attitude determination because the error from skipping 
     * the parallax is smaller than the solar model's own error.
     *
     * \param portNum  Port index, unused (single port)
     * \param state    Orbit state from OrbitPropagator
     */
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;

    /**
     * \brief Discard the interpolation cache and force a fresh
     *        evaluation on the next orbit state.
     *
     * \details 
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /**
     * \brief Solar direction in TEME, interpolated between full
     *        recomputations.
     *
     * \details Evaluates the model at both ends of a SUN_SEGMENT_SEC
     * window and SLERPs across it. Setting SUN_SEGMENT_SEC <= 0 
     * disables caching and evaluates every call.
     *
     * Re-anchors when the cache is empty, when the clock steps
     * backwards, or when the segment has been run off the end.
     *
     * \param jdUt1    UT1 Julian date, flattened, from OrbitState
     * \param jdTt     TT Julian date, flattened, from OrbitState
     * \param rangeKm  [out] Earth-Sun distance, km, interpolated linearly
     * \return Geocentric unit vector Earth -> Sun, TEME
     */
    Astro::Vec3 solarDirectionTeme(F64 jdUt1, F64 jdTt, F64& rangeKm);

    /**
     * \brief Evaluate the solar model and rotate MOD -> TEME at one instant.
     *
     * \details Computationally expensive, so solarDirectionTeme() calls it
     * twice per segment rather than once per tick.
     *
     * \param tUt1      Julian centuries of UT1 since J2000
     * \param tTt       Julian centuries of TT since J2000
     * \param unitTeme  [out] Geocentric unit vector Earth -> Sun, TEME
     * \param rangeKm   [out] Earth-Sun distance, km
     */
    static void sunTemeAt(F64 tUt1, F64 tTt, Astro::Vec3& unitTeme, F64& rangeKm);

    /**
     * \brief Publish on both output ports and update telemetry.
     *
     * \details sunRefOut carries the direction for TRIAD; solarOut
     * carries the full state. Telemetry channels are written only for 
     * the fields the current validity level supports.
     *
     * \param solar  State to publish, valid or not
     * \param stamp  Epoch of the orbit state.
     */
    void emit(const Gnc::SolarState& solar, const Fw::Time& stamp);

    /*
     * ============================================================================
     * Interpolation cache
     *
     * Endpoints of the current segment, in TEME. Modified by orbitIn and
     * RESYNC_SUN, both of which are guarded.
     * ============================================================================
     */
    Astro::Vec3 m_sunPrevTeme { 1.0, 0.0, 0.0 };   //!< Sun direction at segment start
    Astro::Vec3 m_sunNextTeme { 1.0, 0.0, 0.0 };   //!< Sun direction at segment end
    F64         m_sunRangePrevKm = Astro::AU_KM;   //!< Earth-Sun range at segment start
    F64         m_sunRangeNextKm = Astro::AU_KM;   //!< Earth-Sun range at segment end
    F64         m_sunPrevJdUt1 = 0.0;              //!< UT1 JD at segment start
    F64         m_sunNextJdUt1 = 0.0;              //!< UT1 JD at segment end
    bool        m_sunSeeded = false;               //!< Whether the cache holds a real segment

    //! Previous illumination, for edge-triggered eclipse events.
    Astro::Illumination m_lastIllum = Astro::Illumination::SUNLIT;
    bool                m_illumSeeded = false;  //!< Suppresses an event on the first evaluation
};

}  // namespace Environment
}  // namespace Gnc

#endif
