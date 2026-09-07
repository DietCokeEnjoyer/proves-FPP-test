# GNC Module — Constants & Equations Citations

This document catalogs every physical constant, numerical coefficient, and
named algorithm found in the `Gnc` module and points to a public,
citable source so a reviewer can independently verify correctness. It is
organized by subdirectory, then by file, in the same order the constant or
equation appears in the source.

Where the code comments already name a source (e.g. "Vallado", "WMM2015
Technical Report"), this document supplies the full bibliographic
reference and a public link where the value or formula can be checked.

---

## `Gnc/AstroLib/` — `AstroLib.hpp`, `AstroLib.cpp`

Framework-independent astrodynamics kernel. Nearly every constant in the
module traces back to here.

### Math / angle constants
`PI`, `TWO_PI`, `DEG2RAD`, `RAD2DEG`, `ARCSEC2RAD`

- Standard mathematical constants and unit conversions; no external
  citation needed beyond the definition of a degree/arcsecond
  (1 arcsecond = 1/3600 degree). See NIST's *Guide for the Use of the
  International System of Units*, or any standard astrodynamics text
  (e.g. Vallado, below), for the arcsecond convention used throughout
  this file.

### Time-scale constants

- **`JD_J2000 = 2451545.0`** — Julian Date of the standard epoch
  J2000.0 (2000 Jan 1, 12:00 TT).
  IAU / USNO reference: [USNO — Julian Dates](https://aa.usno.navy.mil/data/JulianDate) and
  Vallado, D.A., *Fundamentals of Astrodynamics and Applications*, 4th/5th ed.,
  Microcosm Press — Table of epochs, Ch. 3.

- **`JD_UNIX_EPOCH = 2440587.5`** — Julian Date of the Unix epoch
  (1970-01-01T00:00:00 UTC). This is the standard, algebraically exact
  JD↔Unix-time conversion constant; see
  [USNO Julian Date converter](https://aa.usno.navy.mil/data/JulianDate) for the JD convention it is derived from.

- **`SEC_PER_DAY = 86400.0`**, **`MIN_PER_DAY = 1440.0`** — SI-day
  definitions (86400 SI seconds = 1 mean solar day of civil time). See
  [BIPM — SI Brochure](https://www.bipm.org/en/publications/si-brochure), §2.3.1 (definition of the second) combined with the
  standard civil-day convention.

- **`DAYS_PER_JCENT = 36525.0`** — Definition of the Julian century
  (36525 days of 86400 SI seconds each), used throughout IAU fundamental
  astronomy. See Vallado (above), Ch. 3, and the
  [IERS Conventions (2010), IERS Technical Note 36](https://www.iers.org/IERS/EN/Publications/TechnicalNotes/tn36.html), Ch. 5.

### Earth / Sun physical constants

- **`AU_KM = 149597870.7`** — Astronomical unit, fixed exactly by the
  **IAU 2012 General Assembly Resolution B2** at 149 597 870 700 m.
  Source: [IAU Resolution B2 (2012)](https://syrte.obspm.fr/IAU_resolutions/IAUResol_2012_0.html) (SYRTE/Paris Observatory mirror of the
  IAU resolution text); see also
  [Wikipedia — Astronomical unit](https://en.wikipedia.org/wiki/Astronomical_unit) for a citable plain-language summary.

- **`R_EARTH_KM = 6378.137`** and **`F_EARTH = 1/298.257223563`** —
  WGS-84 ellipsoid semi-major axis and flattening, the U.S. DoD/NGA
  reference ellipsoid.
  Source: National Geospatial-Intelligence Agency, *NIMA/NGA TR8350.2,
  "Department of Defense World Geodetic System 1984, Its Definition and
  Relationships with Local Geodetic Systems,"* 3rd ed. Defining
  parameters table (a = 6378137.0 m, 1/f = 298.257223563) reproduced at:
  [UNOOSA — WGS 84 reference sheet (PDF, NGA-sourced)](https://www.unoosa.org/pdf/icg/2012/template/WGS_84.pdf).

- **`R_SUN_KM = 696000.0`** — Nominal solar radius, as tabulated in
  Vallado (above), Appendix D ("Solar radius"), consistent with the
  IAU nominal solar radius. Source: Vallado, *Fundamentals of
  Astrodynamics and Applications*; cross-check against the
  [IAU 2015 Resolution B3 nominal solar radius (695,700 km, within
  round-off of the value used here)](https://www.iau.org/static/resolutions/IAU2015_English.pdf).

- **`TT_MINUS_TAI_SEC = 32.184`** — Fixed, by-definition offset between
  Terrestrial Time and International Atomic Time.
  Source: [IERS Conventions (2010), IERS Technical Note 36](https://www.iers.org/IERS/EN/Publications/TechnicalNotes/tn36.html), Ch. 1, eq. 1.1
  (TT = TAI + 32.184 s).

### Earth shadow cone half-angles

- **`ALPHA_UMB_RAD = 0.264121°`**, **`ALPHA_PEN_RAD = 0.269007°`** —
  Umbral/penumbral shadow cone half-angles used in the dual-cone
  eclipse (shadow) algorithm.
  Source: Vallado, *Fundamentals of Astrodynamics and Applications*,
  **Algorithm 34 ("Shadow")**, Ch. 5 (Ephemeris/eclipse chapter). A
  public secondary description of the same dual-cone geometry is given
  in Montenbruck, O. and Gill, E., *Satellite Orbits: Models, Methods
  and Applications*, Springer, 2000, §3.4 ("Shadow Function").

### Greenwich Mean Sidereal Time — `gmst1982Rad()`

The polynomial
`67310.54841 + fmod(3155760000·T, 86400) + 8640184.812866·T + 0.093104·T² − 6.2e-6·T³`
is the standard **IAU 1982 GMST expression** (Aoki et al., 1982), as
reformulated by Vallado to avoid loss of precision in the dominant
secular term (see in-code comment).

- Original polynomial form and coefficients: Aoki, S., Guinot, B.,
  Kaplan, G.H., Kinoshita, H., McCarthy, D.D., Seidelmann, P.K.,
  *"The new definition of universal time,"* Astronomy & Astrophysics,
  105, 359–361, 1982.
- Standard algorithmic form (`24110.54841 + 8640184.812866·T + ...`,
  from which the `67310.54841` split-precision form here is derived):
  Vallado, *Fundamentals of Astrodynamics and Applications*,
  **Algorithm 15 ("GSTIME")**.
- Public reference implementations using the identical coefficients:
  [IAU SOFA / P.T. Wallace `slaGMST`, `slaGMSTA` (Starlink)](https://git.sao.ru/eddy/BTA_lib/src/branch/master/slalib/gmst.f) and the
  1984/1992 *Astronomical Almanac*, p. S15 (cited in the SOFA source
  comments).

### IAU-1980 nutation (4 dominant terms) — `nutation1980()`

The truncated series (`dPsi`, `dEps` from `sin(Ω)`, `sin(2L☉)`,
`sin(2L☾)`, `sin(2Ω)` with coefficients −17.20″, −1.32″, −0.23″,
0.21″ / 9.20″, 0.57″, 0.10″, −0.09″) is a 4-term truncation of the
official **IAU 1980 Theory of Nutation** (106-term series).

- Full series and coefficients: Seidelmann, P.K. (ed.), *"1980 IAU
  Theory of Nutation: The Final Report of the IAU Working Group on
  Nutation,"* Celestial Mechanics, 27, 79–106, 1981; reproduced in the
  *Explanatory Supplement to the Astronomical Almanac* (1992), §3.222.
- Canonical reference implementation of the full series (for comparing
  the leading terms used here): [IAU SOFA `iauNut80` / ERFA
  `eraNut80`](https://fact-project.org/FACT++/erfa_8h_a2315fa400bba513887eab772c1a1785f.html).
- Mean obliquity polynomial `23.439291111 − 0.0130041667·T − 1.64e-7·T²
  + 5.04e-7·T³` (degrees): IAU 1980 obliquity expression, also given in
  Vallado, *Fundamentals of Astrodynamics and Applications*, Ch. 3, and
  in the *Astronomical Almanac* (annual editions), §B.
- Equation-of-the-equinoxes kinematic-correction terms (0.00264″,
  0.000063″): the 1997 IAU corrections for the equation of the
  equinoxes, documented in the [IERS Conventions (2010), IERS Technical
  Note 36](https://www.iers.org/IERS/EN/Publications/TechnicalNotes/tn36.html), Ch. 5, and originally in McCarthy, D.D. (ed.), *IERS
  Technical Note 21* (1996).

### Frame rotations (`rot1`, `rot3`) and MOD→TOD→TEME chain

Standard Euler-axis coordinate-frame rotation matrices and their
composition into the precession/nutation transformation chain.
Source: Vallado, *Fundamentals of Astrodynamics and Applications*,
Ch. 3 ("Coordinate and Time Systems"), definitions of `ROT1`/`ROT3` and
the MOD→TOD→TEME/PEF transformation sequence (Vallado's Fig. 3-13 and
associated algorithms).

### WGS-84 geodetic conversion (Bowring's method) — `ecefToGeodetic()`

Non-iterative ECEF→geodetic-latitude conversion using a parametric
("reduced") latitude seed and one closed-form correction.

- Original method: Bowring, B.R., *"Transformation from Spatial to
  Geographical Coordinates,"* Survey Review, 23(181), 323–327, 1976.
- Public secondary description / verification of the same formula:
  [GNU Gama manual — "Transformation from spatial to geographic
  coordinates"](https://www.gnu.org/software/gama/manual/html_node/Transformation-from-spatial-to-geographical-coordinates.html) (reproduces Bowring's closed-form single-iteration
  formula).
- Independent confirmation this is the standard high-accuracy method:
  Comparative study, Virginia Tech TR (1990), *"Comparing the accuracy
  and efficiency of algorithms for converting Cartesian to geodetic
  coordinates,"* found Bowring's 1976 procedure most accurate among 14
  tested methods: [VTechWorks record](https://vtechworks.lib.vt.edu/items/5ea9af75-6b05-4901-99f6-d982f6388c49).

### Low-precision solar ephemeris — `sunLowPrecisionMod()`

Mean longitude `280.460 + 36000.771·T`, mean anomaly
`357.5291092 + 35999.05034·T`, equation-of-center coefficients
`1.914666471`, `0.019994643`, radius-vector series
`1.000140612 − 0.016708617·cos(M) − 0.000139589·cos(2M)`, and mean
obliquity `23.439291 − 0.0130042·T` (all degrees).

- Primary source (named directly in the code comment): Vallado,
  *Fundamentals of Astrodynamics and Applications*, **Algorithm 29
  ("Sun")**, based on the *Astronomical Almanac*'s low-precision solar
  coordinate formulas.
- Original Almanac formulation (same functional form, slightly
  different-precision coefficients across editions): *The Astronomical
  Almanac*, section C ("Low precision formulas for the Sun"); a public
  restatement is given by the **U.S. Naval Observatory**:
  [USNO — "Approximate Solar Coordinates"](https://aa.usno.navy.mil/faq/sun_approx).
- Stated accuracy (~0.01°, 1950–2050) matches the Almanac's own stated
  precision for this formula; see also
  [geospacepy-lite documentation, "Low precision formulas for the
  Sun"](https://geospacepy-lite.readthedocs.io/en/v0.2/sun.html), which restates the same Almanac algorithm with citation.

### Dual-cone eclipse / shadow geometry — `shadowConical()`

Distributed (better-conditioned) form of the umbra/penumbra vertical
extent equations. Source: Vallado, *Fundamentals of Astrodynamics and
Applications*, **Algorithm 34 ("Shadow")**; the underlying dual-cone
geometry is also presented in Montenbruck & Gill (2000), §3.4 (cited
above).

### Cylindrical shadow model — `isSunlitCylindrical()`

Simplified shadow test comparing perpendicular offset from the Sun line
to `R_EARTH_KM²`; a standard, commonly-used simplification of the
dual-cone model. See Vallado (above), remarks preceding Algorithm 34,
and Montenbruck & Gill (2000), §3.4.1 ("Cylindrical Shadow Model").

### Beta angle — `betaAngleRad()`

`beta = 90° − angle(h, sunUnit)`, computed via `atan2` for numerical
conditioning near ±90°. This is the standard orbital "beta angle"
(Sun elevation above the orbit plane) definition used throughout
spacecraft thermal/power analysis.
Source: Vallado, *Fundamentals of Astrodynamics and Applications*, Ch.
5 (eclipse/beta-angle discussion), and
NASA/industry usage summary: [NASA Contractor Report — "Beta Angle"
overview in spacecraft thermal design references] (general concept;
also documented in Wertz, J.R. (ed.), *Spacecraft Attitude Determination
and Control*, 1978, Ch. on solar array pointing).

---

## `Gnc/Adcs/AttitudeDetermination/` — `TriadSolver.hpp`, `TriadSolver.cpp`, `AttitudeDetermination.cpp/.hpp`, `TriadTypes.fpp`

### TRIAD algorithm

The whole two-vector attitude-determination method
(`t1 = v1`, `t2 = (v1×v2)/|v1×v2|`, `t3 = t1×t2`, `A = M_body · M_ref^T`).

- Original publication: Black, H.D., *"A Passive System for Determining
  the Attitude of a Satellite,"* AIAA Journal, 2(7), 1350–1351, July
  1964. Public citation record: [Wikipedia — TRIAD
  method](https://en.wikipedia.org/wiki/Triad_method) (with the original AIAA Journal citation and Bibcode).
- Standard textbook treatment used to verify the exact matrix
  formulation (`A = M_o M_r^T`) implemented here: Lerner, G.M.,
  *"Three-Axis Attitude Determination,"* in Wertz, J.R. (ed.),
  *Spacecraft Attitude Determination and Control*, Springer, 1978,
  pp. 420–428; restated with the same notation in a public survey
  paper: [arXiv:1609.07436 — "UAV attitude estimation using Unscented
  Kalman Filter and TRIAD,"](https://arxiv.org/pdf/1609.07436) §C ("The TRIAD algorithm").
- Discussion of TRIAD's known asymmetry (primary vector satisfied
  exactly, secondary only in-plane) and comparison to QUEST/optimal
  estimators: Shuster, M.D., Oh, S.D., *"Three-Axis Attitude
  Determination from Vector Observations,"* Journal of Guidance and
  Control, 4(1), 70–77, 1981.

### DCM→quaternion conversion (Shepperd's / branch-selecting method)

Used by Eigen's `Quaternionf(Matrix3f)` constructor internally, and
referenced in the code comments (`AttitudeDetermination.cpp`,
`TriadSolver.cpp`) as the reason a naive `sqrt(1+trace)` formula is
avoided.

- Original method: Shepperd, S.W., *"Quaternion from Rotation
  Matrix,"* Journal of Guidance and Control, 1(3), 223–224, 1978.
- Public restatement with full derivation and the four branch
  conditions: [AHRS documentation — "Shepperd's Method"](https://ahrs.readthedocs.io/en/stable/special/Shepperd.html).
- Comparative confirmation that Shepperd's method is the standard
  numerically-robust approach for this problem: Sarabandi, S., Thomas,
  F., *"Accurate Computation of Quaternions from Rotation Matrices,"*
  in *Advances in Robot Kinematics 2018*, Springer, pp. 39–46 (see
  [author's PDF](https://www.iri.upc.edu/download/scidoc/2068)), which cites and builds on Shepperd (1978).

### Sign-continuity convention (q vs. −q)

Standard quaternion double-cover fact (a quaternion and its negation
represent the same rotation); not attributable to a single paper, but
documented in any attitude-kinematics text, e.g. Wertz, J.R. (ed.),
*Spacecraft Attitude Determination and Control*, 1978, Ch. 12
("Quaternions"), or Markley, F.L. & Crassidis, J.L., *Fundamentals of
Spacecraft Attitude Determination and Control*, Springer, 2014, §2.2.

### Numeric thresholds (not physical constants — engineering margins)

`minSinSeparation = 0.0872` (sin 5°), `maxGeometryErrorRad = 0.0873`
(5°), `orthonormalityTol = 1e-3`, `minVectorNorm = 1e-6`: these are
project-specific tuning parameters, not published physical constants,
and are documented in-line in `TriadSolver.hpp`/`AttitudeDetermination.fpp`
with the reasoning (1/sin(angle) error amplification). No external
citation applies; the *rationale* (error scaling as 1/sin(separation))
is standard and is discussed in Shuster & Oh (1981), above, §"Error
Analysis."

---

## `Gnc/Environment/MagneticFieldModel/` — `MagneticFieldModel.cpp/.hpp`, `lib/XYZgeomag.hpp`, `lib/WMM2025.COF`

### World Magnetic Model 2025 (WMM2025)

The Gauss coefficients in `WMM2025.COF` / `XYZgeomag.hpp` and the
validated ranges `WMM_MIN_YEAR/MAX_YEAR = 2025.0–2030.0`,
`WMM_MIN_ALT_KM/MAX_ALT_KM = −1–850 km`.

- Official model and technical report: NOAA National Centers for
  Environmental Information (NCEI) & British Geological Survey (BGS),
  *"The US/UK World Magnetic Model for 2025–2030: Technical Report,"*
  2024. Landing page / DOI: [NOAA NCEI — World Magnetic Model
  2025](https://www.ncei.noaa.gov/products/world-magnetic-model) (dataset DOI:
  10.25921/aqfd-sd83, see [metadata record](https://www.ncei.noaa.gov/metadata/geoportal/rest/metadata/item/gov.noaa.ngdc:WMM2025/html)).
  Validity period (2025.0–2030.0) confirmed on the same page.
- Announcement / plain-language summary:
  [NOAA NCEI — "World Magnetic Model Receives Upgrade"](https://www.ncei.noaa.gov/news/world-magnetic-model-receives-upgrade).
- Coefficient file (`WMM2025.COF`) is the standard NOAA/NGA-distributed
  `.COF` format; not subject to copyright per U.S. government work
  policy (also stated in the `XYZgeomag.hpp` header comment).

### Mean Earth radius used in the spherical-harmonic recursion

`EARTH_R = 6371200.0` m — the reference sphere radius specified for
evaluating the WMM's spherical harmonic series (distinct from the
WGS-84 ellipsoid radius used elsewhere in `AstroLib`).
Source: cited directly in `XYZgeomag.hpp` as "mean radius of ellipsoid
in meters from section 1.2 of the WMM2015 Technical Report" — i.e.
Chulliat, A. et al., *"The US/UK World Magnetic Model for 2015-2020:
Technical Report,"* NOAA NCEI, 2015, §1.2. The 2025 report (above)
retains the same reference radius convention.

### Spherical harmonic synthesis algorithm

The recursive Legendre/associated-Legendre evaluation (`Vnm`, `Wnm`
recursion) used to sum the Gauss coefficients into a field vector.
Source: cited directly in `XYZgeomag.hpp`'s file header as using
"a different spherical harmonics calculation, described in sections
3.2.4 and 3.2.5" of Montenbruck, O. and Gill, E., *Satellite Orbits:
Models, Methods, and Applications*, Springer, 2000 (the
Cunningham/Cain-style non-normalized recursive method for spherical
harmonic gradients).

### Library attribution

`XYZgeomag.hpp` — header-only WMM evaluation library.
Source: Zimmerberg, N. (Cornell University), *XYZgeomag*, MIT License,
2019–2021; repository: [github.com/nathanzimmerberg/XYZgeomag](https://github.com/nathanzimmerberg/XYZgeomag) (license
and authorship reproduced verbatim in the file's header comment).

### Unit conversions

`TESLA_TO_NT = 1e9`, `KM_TO_M = 1000.0` — SI unit definitions (1 T =
10⁹ nT by definition of the nanotesla prefix; 1 km = 1000 m by
definition of the SI kilo- prefix). See
[BIPM — SI Brochure](https://www.bipm.org/en/publications/si-brochure), §3 (SI prefixes).

---

## `Gnc/Environment/SolarEphemeris/` — `SolarEphemeris.cpp/.hpp`

No new physical constants are introduced in this file; it consumes
`AstroLib`'s solar ephemeris (`sunLowPrecisionMod`, cited above),
nutation model, and frame-rotation utilities. Noted engineering
approximations, with their justifying figures, are documented inline:

- **Solar angular rate ≈ 0.0000042°/s** (used to justify tolerating a
  stale reference vector): consistent with the Sun's mean motion of
  ≈0.9856°/day used in the low-precision ephemeris above (0.9856°/day
  ÷ 86400 s/day ≈ 1.14e-5 °/s scale; the code's stricter figure is a
  conservative bound consistent with Vallado's Algorithm 29 rate).
- **Parallax at LEO ≈ 0.0027°** and **solar-model error ≈ 0.01°**:
  both are geometric consequences of Earth's radius (`R_EARTH_KM`,
  WGS-84, cited above) versus the Sun's distance (`AU_KM`, IAU 2012,
  cited above), and the stated accuracy of Vallado's Algorithm 29,
  respectively — not independent constants requiring separate
  citation.

---

## `Gnc/Environment/OrbitPropagator/` — `OrbitPropagator.cpp/.hpp/.fpp`

### SGP4 propagator

Propagation is delegated entirely to the third-party `perturb` library
(`#include <perturb/perturb.hpp>`), which wraps Vallado's reference
SGP4 implementation.

- `perturb` library: Ranu, G., *perturb* (C++11 SGP4 wrapper),
  repository: [github.com/gunvirranu/perturb](https://github.com/gunvirranu/perturb) — states directly that it
  "Uses Vallado's latest and greatest de-facto standard SGP4
  implementation."
- Underlying SGP4/SDP4 theory and reference source code: Vallado,
  D.A., Crawford, P., Hujsak, R., Kelso, T.S., *"Revisiting Spacetrack
  Report #3,"* AIAA/AAS Astrodynamics Specialist Conference, 2006
  (AIAA 2006-6753). Reference implementation mirror:
  [CelesTrak — Vallado SGP4 source](https://celestrak.org/software/vallado-sw.php) (original
  distribution site named in multiple public mirrors, e.g.
  [github.com/magnific0/SGP4](https://github.com/magnific0/SGP4)).
- Original SGP4 algorithm (historical basis): Hoots, F.R., Roehrich,
  R.L., *"Spacetrack Report No. 3: Models for Propagation of NORAD
  Element Sets,"* Aerospace Defense Command, 1980. PDF mirror:
  [CelesTrak — Spacetrack Report #3](https://celestrak.org/NORAD/documentation/spacetrk.pdf).

### Time-scale / leap-second parameters (`OrbitPropagator.fpp`)

- **`LEAP_SEC` default `37.0`** — TAI−UTC leap-second count. This is
  a *time-varying* quantity set by international agreement, not a
  fixed physical constant; it must be updated whenever IERS announces
  a new leap second. Authoritative current value and history:
  [IERS — Bulletin C (leap second announcements)](https://datacenter.iers.org/data/latestVersion/bulletinC.txt).
  (37.0 s was correct from January 2017 through at least the WMM2025
  epoch used elsewhere in this module; the parameter's TODO-worthy risk
  is exactly this drift, which is why it is a runtime parameter and
  not a compiled constant.)
- **`DUT1_SEC` default `0.0`**, comment "always < 0.9 s" — UT1−UTC,
  published in IERS Bulletin A. By international convention (IERS/BIH)
  leap seconds are inserted to keep |UT1−UTC| < 0.9 s.
  Source: [IERS — Bulletin A](https://datacenter.iers.org/data/latestVersion/bulletinA.txt) and
  [IERS Conventions (2010), IERS Technical Note 36](https://www.iers.org/IERS/EN/Publications/TechnicalNotes/tn36.html), Ch. 5
  (definition of UT1 and the 0.9 s leap-second insertion criterion).
- **`MAX_TLE_AGE_DAYS` default `7.0`** and the in-code note "SGP4
  accuracy degrades roughly 1–3 km per day past epoch" — a widely
  cited empirical/operational figure for SGP4/TLE propagation error
  growth. See Vallado, D.A. & Crawford, P., *"SGP4 Orbit Determination,"*
  AIAA/AAS Astrodynamics Specialist Conference, 2008 (AIAA 2008-6770),
  and the general TLE-accuracy discussion at
  [CelesTrak — FAQs, "How current do the elements need to be?"](https://celestrak.org/columns/v04n12/)
  (Kelso, T.S., CelesTrak "Space Track Report" columns).

### TLE line length (69 characters)

Standard NORAD Two-Line Element Set fixed-column format.
Source: [CelesTrak — TLE format description](https://celestrak.org/NORAD/documentation/tle-fmt.php) (NORAD/Space-Track TLE
format specification, 69 data columns per line).

---

## `Gnc/Types/` — `GncTypes.fpp`, `GncConvert.hpp`

No new physical constants or equations. This module defines shared
data structures (`Vec3d`, `Vec3f`, `OrbitState`, `SolarState`,
`FrameId`, etc.) and type-conversion glue between `AstroLib` and the
F Prime-generated types; all numeric content it references (e.g. "0.36
deg of precession error in 2026" in doc comments, "0.0027 deg" parallax,
"20000–50000 nT" typical LEO field magnitude) is a restatement of
figures already cited above (IAU-1980 nutation/precession rate, WGS-84
geometry, and the WMM2025 model, respectively) and needs no additional
source.

---

## Summary table

| Constant / Algorithm | File(s) | Primary citation |
|---|---|---|
| Astronomical unit (149597870.7 km) | `AstroLib.hpp` | IAU 2012 Resolution B2 |
| WGS-84 ellipsoid (a, f) | `AstroLib.hpp`, `XYZgeomag.hpp` | NGA TR8350.2 |
| TT − TAI = 32.184 s | `AstroLib.hpp` | IERS Conventions (2010), TN36 |
| Earth shadow cone half-angles | `AstroLib.hpp/.cpp` | Vallado, Alg. 34 |
| GMST (IAU 1982) | `AstroLib.cpp` | Aoki et al. 1982; Vallado Alg. 15 |
| Nutation (IAU 1980, 4 terms) | `AstroLib.cpp` | Seidelmann 1981 (IAU WG report) |
| Bowring ECEF→geodetic | `AstroLib.cpp` | Bowring 1976, Survey Review |
| Low-precision solar ephemeris | `AstroLib.cpp` | Vallado Alg. 29 / Astronomical Almanac |
| TRIAD algorithm | `TriadSolver.hpp/.cpp` | Black 1964, AIAA Journal |
| Shepperd DCM→quaternion | `TriadSolver.cpp` (via Eigen) | Shepperd 1978, J. Guid. Control |
| WMM2025 coefficients | `WMM2025.COF`, `XYZgeomag.hpp` | NOAA NCEI / BGS Technical Report |
| Spherical-harmonic recursion | `XYZgeomag.hpp` | Montenbruck & Gill 2000, §3.2 |
| SGP4 propagator | `OrbitPropagator.cpp/.hpp` | Vallado, Crawford, Hujsak, Kelso 2006 |
| Leap seconds / DUT1 | `OrbitPropagator.fpp` | IERS Bulletins A & C |
| TLE format | `OrbitPropagator.fpp` | CelesTrak TLE format spec |

---

*Compiled for internal review. All links point to publicly accessible
pages as of the time of writing; if a link has moved, search for the
bibliographic citation text (author, title, year), which is
independently sufficient to locate the source.*
