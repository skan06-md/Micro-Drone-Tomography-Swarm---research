# Micro-Drone Tomography Swarm (MDTS) — research simulation

A swarm of 5–8 micro-drones orbits a target drone and uses their small 77 GHz
mmWave radars together as one large distributed aperture, to image the target
and flag unexpected objects inside it.

This repo builds the idea up in stages. **Week 1 is a radar feasibility check:**
before building any formation control, can a ring of small radars actually
detect a hidden object inside a drone's shell?

## Week 1 — what's here

```
mdts/
  materials.py   shell penetration at 77 GHz (dielectric slab model + table)
  scene.py       2D top-down target drone as point scatterers
                 (shell, arms, motors, battery, PCB, optional hidden 3 cm block)
  radar.py       multistatic stepped-frequency radar on a ring of drones,
                 FFT backprojection (coherent + incoherent), orbit-sweep mode
  detect.py      Monte Carlo anomaly detection (Pd at 5% false-alarm rate)
  sweep.py       orbit-sweep aperture: the ring rotates and every drone images
                 as it flies (monostatic), plus per-drone position-error model
experiments/
  run_week1.py   runs experiments E1–E6, writes figures to results/
  run_sweep.py   Week 1b experiments S1–S3 (sweep vs snapshot, position error)
docs/
  week1_findings.md   results, sources, limitations, go/no-go
```

## Run it

```bash
pip install -r requirements.txt
python experiments/run_week1.py --quick      # smoke test, ~8 min
python experiments/run_week1.py              # full run
python experiments/run_week1.py --only e3 e6 # just some experiments

# Week 1b (sweep). Each stage saves json to results/, then plots reads it.
python experiments/run_sweep.py --only s1 s2      # ~6 min
python experiments/run_sweep.py --only s3a        # ~15 min
python experiments/run_sweep.py --only s3b s3c    # ~20 min
python experiments/run_sweep.py --only plots
```

## Experiments

| ID | Question | Figure |
|----|----------|--------|
| E1 | How much 77 GHz gets through common shell materials? | `e1_shell_transmission.png` |
| E2 | What do the images look like: 6-drone snapshot vs 36-drone snapshot vs 6 drones sweeping while they orbit? | `e2_point_spread.png`, `e2_gallery_*.png` |
| E3 | Detection vs shell transmission | `e3_pd_vs_shell.png` |
| E4 | Detection vs signal-to-noise ratio | `e4_pd_vs_snr.png` |
| E5 | Detection vs number of drones (**misleading**: one fixed ring rotation, see S1) | `e5_pd_vs_drones.png` |
| E6 | How precisely must each drone know its own position? (snapshot ring) | `e6_pd_vs_position_error.png` |
| S1 | Snapshot ring: Pd vs ring rotation, 4–12 drones (explains the E5 sawtooth) | `s1_pd_vs_ring_orientation.png` |
| S2 | Orbit sweep vs snapshot ring: Pd vs SNR | `s2_sweep_vs_snapshot.png` |
| S3 | Orbit sweep: Pd vs position-knowledge error, 4/6/12 drones | `s3_sweep_position_error.png` |

**Headline results:**
- A hidden 3 cm block is detectable through plastic shells but not through
  carbon-filled/carbon-fibre or metal shells.
- An orbit sweep needs about 10–20 dB less SNR than a snapshot ring, mostly because
  it collects far more measurements.
- Ring rotation changes snapshot-ring detection by as much as swarm size does.
- Coherent imaging needs each drone to know its position to about 0.1 mm; a
  magnitude-only detector in the sweep tolerates about 10 mm (noise-free test).

Full results, limits and sources: **[docs/week1_findings.md](docs/week1_findings.md)**.

## Model assumptions (read before trusting numbers)

- 2D, top-down slice; target and swarm in the same plane.
- Born approximation: isotropic point scatterers, no shadowing, no multipath,
  no specular glints. Real flat surfaces reflect mostly in one direction.
- Shell penetration applied as a fixed amplitude factor on everything inside.
- Target is static during the scan (motion compensation is a later stage).
- Detection compares against a known noise-free image of a *normal* drone of the
  same model — a best case. Real use needs a library of reference drones.
- Pairs of drones more than 120° apart are not used (the transmitter's direct
  signal would swamp the echo at the same delay).
