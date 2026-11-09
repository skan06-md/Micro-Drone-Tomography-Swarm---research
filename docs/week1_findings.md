# Week 1 findings — can a ring of small 77 GHz radars see inside a drone?

**Verdict: conditional GO.** In this simplified 2D simulation a hidden 3 cm block
is detectable through **plastic or foam shells**. It is **not detectable through
carbon-filled or carbon-fibre shells or metal**. Coherent imaging needs each drone
to know its own position to roughly 0.1 mm, which is the hardest requirement found
so far. Treat everything below as a model result, not a measurement.

Setup: 77 GHz, 4 GHz bandwidth, 64 frequency steps, 3 m orbit radius, 15 cm square
target body, 3 cm hidden block, 5% false-alarm rate, 60 trials per hypothesis
(sampling noise on any Pd value is about ±6 percentage points; values at 0.03–0.10
are the false-alarm floor, not real detections).

## 1. Shell material decides everything (E1, E3)

| Shell | One-way loss at 77 GHz | Source quality |
|-------|------------------------|----------------|
| PP, ABS, PC, PEI, PBT+glass | about 0.5–1 dB for a 2 mm shell | permittivity from datasheets/patents; loss tangent partly assumed |
| PBT + carbon-fibre filler | about 15 dB (patent: <3% power transmitted) | published |
| Carbon-fibre laminate (CFRP) | 30 dB used here | **assumed** — real laminates can be worse |
| Aluminium / metal | opaque | physics |

Detection (6 drones, SNR −10 dB per sample, block as bright as a motor):
- **Pd = 1.0** through losses up to 6 dB one-way (coherent), 3 dB (incoherent).
- **Pd ≈ 0.2 at 10 dB loss, chance at 15 dB and beyond.**
- A weaker block (30% of reference brightness) is lost by about 3–6 dB of shell loss.

So plastics are fine; anything carbon-loaded or metal is a no-go at 77 GHz. A
real attacker's drone is often carbon fibre, which limits the usefulness of the
idea against real targets and matches the risk flagged in the requirements document.

## 2. Imaging needs the swarm to sweep, not snapshot (E2)

Six drones in a single snapshot give an image dominated by sidelobes; the arms,
motors and battery are not recognisable. Thirty-six drones in one snapshot are
better but still noisy. **Six drones each sweeping their own 60° slice while the
ring orbits** (a circular-SAR geometry) produce a clean picture of the arms,
motors, battery and the hidden block (`e2_gallery_abs.png`). This supports the
document's "single circular pass" concept and suggests the orbit motion is a
feature, not just a positioning problem.

## 3. Position knowledge is the hard requirement (E6)

Error in each drone's *known* position (not how well it holds its slot):

| Position error (1σ per axis) | Coherent Pd | Incoherent Pd |
|------------------------------|-------------|---------------|
| 0 | 1.00 | 1.00 |
| 0.1 mm | 0.67 | 1.00 |
| 0.25 mm | 0.27 | 1.00 |
| 0.5–2 mm | 0.10–0.13 | 1.00 |
| 5 mm | 0.27 (noise) | 0.70 |
| 10 mm | 0.10 | 0.13 |

Coherent imaging collapses past roughly 0.1–0.25 mm (about λ/40 to λ/15). The
incoherent mode survives to about 2 mm but gives a blurrier image and relies on
range resolution (about 3.75 cm at 4 GHz). Millimetre-level self-localisation of
flying drones is not available from GNSS, UWB or vision alone. This is the key
engineering risk, and more important than the requirements document's 1 µs time
sync figure.

## 4. Noise (E4)

With coherent change detection, 8 drones reach Pd ≥ 0.8 at −20 dB per-sample SNR,
6 drones at about −15 dB, and 5 drones at about −15 dB (Pd 0.77). More drones buy
roughly 5 dB of margin. The per-sample SNR here is an input assumption, **not a
link budget**: a real link budget for the small radars at 3 m has not been done.

## 5. Swarm size: inconclusive (E5)

Pd versus number of drones is a sawtooth, not a trend: 4 and 8 drones score high
(~0.7–0.9), 5 and 6 score low (~0.1–0.4). Re-running with two other random seeds
gave the same pattern, so it is not sampling noise. Likely cause (not yet tested):
rings of 4 and 8 line up with the symmetry axes of the square body. Practical
consequence: detection depends on ring geometry relative to target orientation,
and a real target will not hold a convenient orientation. **No conclusion on swarm
size should be drawn from E5.**

## Limitations of this model

- 2D slice, point scatterers, Born approximation: no shadowing, multipath or
  specular reflections. Real flat panels reflect mostly one way.
- Static target; no motion compensation.
- Detection compares against a noise-free image of the same normal drone. This is
  a best case; in practice a library of reference drones would be needed.
- Detection statistics use single snapshots; the sweep geometry that images best
  (E2) has not yet been run through the Monte Carlo.
- The "hidden block" is as bright as a motor and sits in free interior space.
- mmWave sees shape and reflectivity contrast. It cannot identify what a material
  is, so this is **anomaly detection** ("unexpected object here"), not substance
  identification.
- Several loss-tangent values and the CFRP figure are assumptions (see
  `mdts/materials.py`).

## Suggested next steps

1. Run the detection Monte Carlo with the **sweep** geometry and a rotating target
   orientation; this should also resolve E5.
2. Add a simple link budget so SNR is computed rather than assumed.
3. Go to 3D (staggered altitudes) to see how much elevation resolution the ring gets.
4. Then start the formation simulation (Week 2 in the plan).

## Sources

- Avient PREPERM automotive radar bulletin, permittivity and loss tangent at 77 GHz:
  https://www.avient.com/sites/default/files/2022-10/PREPERM%20Automotive%20Application%20Bulletin.pdf
- Radar-cover patent, dielectric constants at 76.5 GHz (PP 2.2, ABS 2.7, PC 3.0):
  https://patents.justia.com/patent/20040227663
- PBT–carbon-fibre composites, <3% transmission at 75–110 GHz:
  https://patents.justia.com/patent/20240093004
- Dorbath, Schür, Vossiek (2023), transmission loss of unidirectional CFRP at mm-wave, IEEE TMTT:
  https://cris.fau.de/publications/310741327/
- Bosch radar fascia guidelines (ABS acceptable; glass- or carbon-fibre compounds not recommended):
  https://www.manualslib.com/manual/2509149/Bosch-Cr5cbcc.html?page=11
- RF Essentials, radome thickness and loss at mm-wave:
  https://rfessentials.com/rf-knowledge-base/how-does-radome-material-and-thickness-affect-antenna-performance-at-millimeter-/
