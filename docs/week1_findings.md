# Week 1 findings — can a ring of small 77 GHz radars see inside a drone?

**Verdict: conditional GO.** In this simplified 2D simulation a hidden 3 cm block
is detectable through **plastic or foam shells**. It is **not detectable through
carbon-filled or carbon-fibre shells or metal**. Coherent imaging needs each drone
to know its own position to roughly 0.1 mm, which is the hardest requirement found
so far. Treat everything below as a model result, not a measurement.

**Week 1b update (sections 6–8):** flying an orbit *sweep* (each drone images while
covering its own slice of the circle) beats a snapshot ring by about 10–20 dB of SNR,
and ring rotation matters as much as swarm size for snapshot rings. In the sweep, a
non-coherent (magnitude-only) detector tolerates position errors up to about 10 mm,
while coherent imaging still needs about 0.1 mm.

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

## 5. Swarm size: E5 was misleading (resolved in section 6)

Pd versus number of drones is a sawtooth, not a trend: 4 and 8 drones score high
(~0.7–0.9), 5 and 6 score low (~0.1–0.4). Re-running with two other random seeds
gave the same pattern. E5 used **one fixed ring orientation**, and section 6 shows
that orientation, not the drone count, produced the sawtooth. **Do not draw any
conclusion on swarm size from E5; use section 6.**

## 6. Week 1b — ring rotation explains the E5 sawtooth (S1)

Snapshot rings, coherent change detection, ABS shell, SNR −20 dB per sample, 12 ring
rotations (0–82.5°) with 40 noise draws each. Pd depends strongly on how the ring is
rotated relative to the target body:

| Drones | Pd range across rotations | Pd pooled over rotations |
|--------|---------------------------|--------------------------|
| 4 | 0.03 – 0.60 | 0.25 |
| 5 | 0.03 – 0.25 | 0.12 |
| 6 | 0.07 – 0.55 | 0.22 |
| 8 | 0.10 – 0.82 | 0.31 |
| 12 | 0.12 – 1.00 | 0.44 |

The pattern repeats with the ring's own symmetry (the 12-drone ring reaches Pd 1.0 at
15°, 45° and 75°, which are the same ring position modulo 30°), so it is a geometry
effect, not noise. The mechanism is not identified. Pooled over rotations, which is
the realistic case because you cannot choose the angle, more drones help on average
(5 is the odd one out), but rotation changes Pd by more than going from 4 to 8
drones does. Figure: `results/s1_pd_vs_ring_orientation.png`.

## 7. Week 1b — orbit sweep vs snapshot ring (S2)

Six drones, each flying its own 60° slice of the circle in 0.5° steps (720 looks in
total, monostatic), against the snapshot rings above. Same scene, same ABS shell,
block reflectivity 0.5.

| Geometry | Per-sample SNR needed for Pd ≥ 0.8 (5 dB grid) |
|----------|-----------------------------------------------|
| Snapshot, 4 or 5 drones | −5 dB |
| Snapshot, 6 or 8 drones | −10 dB |
| Snapshot, 12 drones | −15 dB |
| **Sweep, 6 drones** | **−25 dB** |

That is a 10–20 dB improvement. Most of it is plain integration gain: the sweep
combines 720 looks, while a 6-drone snapshot has 18 transmit/receive pairs, and
10·log10(720/18) ≈ 16 dB. So the benefit comes from collecting more measurements,
at the cost of time (each drone covers about 3 m of path) and of needing the target
to hold still or be motion-compensated during the sweep. In this model each look is
monostatic, so no shared RF phase reference between drones is needed; real hardware
would still need matched frequency sweeps and good oscillators. Figure:
`results/s2_sweep_vs_snapshot.png`.

Method note: the first S2 run showed a 0.20 false-alarm floor at very low SNR. The
cause was my splitting of noise draws into a threshold set and a test set, which let
a mild drift in 120 draws bias the threshold. The final results use a leave-one-out
threshold (each draw is compared with the 95th percentile of all the other draws);
the floor is now 0.05–0.06 as it should be.

## 8. Week 1b — position error in the sweep (S3)

Each drone has a constant unknown position offset (1σ per axis) and the imager assumes
the nominal positions. 30 random draws per setting, **no receiver noise** (this
isolates the position error, so Pd values are upper bounds). Values near 0.07 are the
false-alarm floor.

| Position error | Coherent, 6 drones | Incoherent, 6 drones |
|----------------|--------------------|----------------------|
| 0.1 mm | 1.00 | 1.00 |
| 0.25 mm | 0.13 | 1.00 |
| 0.5 – 2 mm | 0.07 | 1.00 |
| 5 – 10 mm | 0.07 | 1.00 |
| 20 mm | 0.07 | 0.17 |

- **Coherent** imaging survives about 0.1 mm (λ/40) and is at the floor from 0.25 mm.
  That is similar to the snapshot ring (Pd 0.67 at 0.1 mm in E6).
- **Incoherent** (magnitude-only) detection survives to 10 mm in the sweep and fails by
  20 mm, much more tolerant than the snapshot ring (fails between 5 and 10 mm in E6).
  It is limited to range resolution (about 3.75 cm at 4 GHz), so it says "something
  bright changed here", not what shape the 3 cm block has.
- Using 4 or 12 drones instead of 6 changed nothing at 0.25 mm and 1 mm (all at the
  coherent floor), so averaging more independent per-drone errors did not rescue
  coherent imaging. Errors between 0.1 and 0.25 mm were not tested.
- Real errors drift with time; a constant offset per drone is the simplest case.

Figure: `results/s3_sweep_position_error.png`.

## Limitations of this model

- 2D slice, point scatterers, Born approximation: no shadowing, multipath or
  specular reflections. Real flat panels reflect mostly one way.
- Static target; no motion compensation.
- Detection compares against a noise-free image of the same normal drone. This is
  a best case; in practice a library of reference drones would be needed.
- E3–E6 use single-snapshot rings. The sweep was tested separately in Week 1b
  (S1–S3); its position-error results (S3) are noise-free upper bounds.
- The "hidden block" is as bright as a motor and sits in free interior space.
- mmWave sees shape and reflectivity contrast. It cannot identify what a material
  is, so this is **anomaly detection** ("unexpected object here"), not substance
  identification.
- Several loss-tangent values and the CFRP figure are assumptions (see
  `mdts/materials.py`).

## Suggested next steps

1. Add a simple link budget so SNR is computed rather than assumed; the sweep and
   snapshot comparisons above are all in terms of assumed per-sample SNR.
2. Test **autofocus**: estimate each drone's position error from the target's own
   bright scatterers (the motors) and correct it before imaging. Standard in SAR, and
   the most likely way to relax the 0.1 mm requirement for coherent imaging.
3. Add target motion and sweep timing (motion compensation).
4. Go to 3D (staggered altitudes) to see how much elevation resolution the ring gets.
5. Then start the formation simulation (Week 2 in the plan).

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
