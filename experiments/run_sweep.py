"""Week 1b: orbit-sweep aperture vs snapshot ring.

Questions:
  S1  Does the E5 sawtooth (Pd vs number of drones) come from ring orientation?
      -> snapshot ring, Pd vs ring offset angle, N = 4, 5, 6, 8, 12.
  S2  How much does flying the sweep help? Pd vs SNR, sweep vs snapshot rings.
  S3  How precisely must each drone know its position in the sweep, and does
      having more drones (more independent position errors) help?

Speed trick (coherent mode only): backprojection is linear in the data, so
    image(signal + sigma*noise) = image(signal) + sigma*image(noise).
Noise-only images are computed once and reused for every SNR. S3 cannot use
the trick (position error changes the signal), so it simulates every draw.

    python experiments/run_sweep.py --only s1 s2      # cheap (~5-8 min)
    python experiments/run_sweep.py --only s3a        # N=6, ~12 min
    python experiments/run_sweep.py --only s3b        # N=4 and 12, ~10 min
    python experiments/run_sweep.py --only s3c        # N=6, 5/10/20 mm, ~10 min
    python experiments/run_sweep.py --only plots      # figures from saved json
    python experiments/run_sweep.py --quick --only s1 s2 s3a s3b plots
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mdts import scene
from mdts.materials import MATERIALS, shell_factors
from mdts.radar import RadarConfig, simulate, backproject
from mdts.detect import interior_grid
from mdts.sweep import SweepAperture

OUT = Path(__file__).resolve().parents[1] / "results"
OUT.mkdir(exist_ok=True)
MAT = {m.name: m for m in MATERIALS}
ABS_AMP, ABS_REFL, _ = shell_factors(MAT["ABS"])
RHO = 0.5                      # hidden block reflectivity (motors are 1.0)
SIGNAL_AMP = ABS_AMP * RHO     # payload amplitude after crossing the ABS shell twice
SNRS = np.arange(-60, 1, 5)    # per-sample SNR of a unit point scatterer (dB)
NPOS = len(scene.PAYLOAD_POSITIONS)


def log(*a):
    print(*a, flush=True)


def dump(name, obj):
    (OUT / f"sweep_{name}.json").write_text(json.dumps(obj, indent=2))


def load(name):
    p = OUT / f"sweep_{name}.json"
    return json.loads(p.read_text()) if p.exists() else None


def cnoise(rng, shape):
    return (rng.normal(size=shape) + 1j * rng.normal(size=shape)) / np.sqrt(2)


def pd_curves(sig, noi, snrs):
    """Pd at 5% false alarm from precomputed signal / noise images (coherent change detection).

    sig: (K, NPOS, npx) payload images (noise-free).  noi: (K, M, npx) noise-only images.
    Every noise draw is used both as an H0 sample and (with a payload added) as an H1
    sample. The false-alarm threshold applied to draw i is the 95th percentile of the
    H0 statistic over all *other* draws (leave-one-out), so no draw sets its own
    threshold and a lucky/unlucky split of the draws cannot bias the result.
    Returns pooled Pd per SNR and per-condition (K) Pd per SNR.
    """
    K, M, _ = noi.shape
    pos = np.arange(M) % NPOS
    flat = np.abs(noi).max(axis=2).ravel()            # H0 statistic at sigma = 1
    thr_unit = np.array([np.quantile(np.delete(flat, i), 0.95) for i in range(flat.size)]).reshape(K, M)
    pooled, per_k = [], []
    for snr in snrs:
        s = 10 ** (-snr / 20)
        det = np.empty((K, M), bool)
        for k in range(K):
            x = SIGNAL_AMP * sig[k][pos] + s * noi[k]
            det[k] = np.abs(x).max(axis=1) > s * thr_unit[k]
        pooled.append(float(det.mean()))
        per_k.append(det.mean(axis=1).tolist())
    return pooled, per_k


# --------------------------------------------------------------------- S1 + S2
def s1(quick):
    log("S1: snapshot ring, Pd vs ring orientation")
    grid = interior_grid("coherent")
    mask = scene.interior_mask(grid.X, grid.Y)
    K, M = (3, 8) if quick else (12, 40)
    offsets = np.arange(K) * (90.0 / K)
    rng = np.random.default_rng(10)
    res = dict(offsets_deg=offsets.tolist(), snr_db=SNRS.tolist(), n_noise=M, results={})
    for N in ([4, 6] if quick else [4, 5, 6, 8, 12]):
        t0 = time.time()
        cfg = RadarConfig(n_nodes=N)
        npx = int(mask.sum())
        sig = np.empty((K, NPOS, npx), np.complex64)
        noi = np.empty((K, M, npx), np.complex64)
        for k, off in enumerate(offsets):
            nodes = cfg.node_positions(np.deg2rad(off))
            for j, p in enumerate(scene.PAYLOAD_POSITIONS):
                sig[k, j] = backproject(cfg, grid, simulate(cfg, *scene.payload(p), nodes), nodes)[mask]
            for m in range(M):
                shp = (len(cfg.pairs()), cfg.n_freq)
                noi[k, m] = backproject(cfg, grid, cnoise(rng, shp), nodes)[mask]
        pooled, per_k = pd_curves(sig, noi, SNRS)
        res["results"][str(N)] = dict(pooled=pooled, per_offset=per_k, n_pairs=len(cfg.pairs()))
        i20 = int(np.where(SNRS == -20)[0][0])
        log(f"   N={N}: pooled Pd at -20 dB = {pooled[i20]:.2f}, "
            f"per-offset min/max = {min(per_k[i20]):.2f}/{max(per_k[i20]):.2f}  ({time.time() - t0:.0f} s)")
    dump("s1", res)


def s2(quick):
    log("S2: orbit sweep (6 drones, monostatic), Pd vs SNR")
    grid = interior_grid("coherent")
    mask = scene.interior_mask(grid.X, grid.Y)
    M = 12 if quick else 120
    rng = np.random.default_rng(20)
    ap = SweepAperture(RadarConfig(n_nodes=6), grid, step_deg=0.5)
    npx = int(mask.sum())
    t0 = time.time()
    sig = np.empty((1, NPOS, npx), np.complex64)
    for j, p in enumerate(scene.PAYLOAD_POSITIONS):
        sig[0, j] = ap.backproject(ap.simulate(*scene.payload(p)))[mask]
    noi = np.empty((1, M, npx), np.complex64)
    for m in range(M):
        noi[0, m] = ap.backproject(ap.noise(rng))[mask]
    pooled, _ = pd_curves(sig, noi, SNRS)
    noise_max = np.abs(noi[0]).max(axis=1).tolist()     # max |noise image| per draw at sigma = 1
    sig_max = np.abs(sig[0]).max(axis=1).tolist()
    log("   pooled Pd:", dict(zip(SNRS.tolist(), np.round(pooled, 2).tolist())), f"({time.time() - t0:.0f} s)")
    dump("s2", dict(snr_db=SNRS.tolist(), pd=pooled, n_looks=ap.n_looks, n_noise=M,
                    noise_max=noise_max, payload_max=sig_max,
                    note="full-circle coverage -> orientation invariant, so one orientation"))


# --------------------------------------------------------------------------- S3
def s3(name, plan, trials, quick, seed=33):
    """plan: list of (n_drones, [position errors in mm]). Noise-free, isolates position error."""
    grid = interior_grid("coherent")
    mask = scene.interior_mask(grid.X, grid.Y)
    rng = np.random.default_rng(seed)
    Pe, Ae = scene.exterior(ABS_REFL)
    Pi, Ai = scene.interior_nominal()
    Pall, Aall = np.vstack([Pe, Pi]), np.concatenate([Ae, ABS_AMP * Ai])
    res = {}
    ref = {}
    for N, errs in plan:
        ap = SweepAperture(RadarConfig(n_nodes=N), grid, step_deg=0.5)
        if "c" not in ref:   # nominal reference is identical for every N (full-circle sweep)
            ref["c"], ref["i"] = ap.backproject(ap.simulate(Pall, Aall), incoherent=True)
        for err in errs:
            t0 = time.time()
            h0c, h0i, h1c, h1i = [], [], [], []
            for _ in range(trials):
                nodes = ap.perturbed_nodes(rng, err * 1e-3)
                D = ap.simulate(Pall, Aall, nodes)
                Dp = SIGNAL_AMP * ap.simulate(*scene.payload(scene.PAYLOAD_POSITIONS[rng.integers(NPOS)]), nodes)
                c0, i0 = ap.backproject(D, incoherent=True)       # imager assumes nominal positions
                c1, i1 = ap.backproject(D + Dp, incoherent=True)
                h0c.append(np.abs(c0 - ref["c"])[mask].max()); h1c.append(np.abs(c1 - ref["c"])[mask].max())
                h0i.append((i0 - ref["i"])[mask].max());       h1i.append((i1 - ref["i"])[mask].max())
            pdc = float(np.mean(np.array(h1c) > np.quantile(h0c, 0.95)))
            pdi = float(np.mean(np.array(h1i) > np.quantile(h0i, 0.95)))
            res[f"N{N}_err{err}"] = dict(n_drones=N, err_mm=err, pd_coherent=pdc, pd_incoherent=pdi, trials=trials)
            log(f"   N={N:2d} err={err:5.2f} mm: Pd coherent {pdc:.2f}, incoherent {pdi:.2f}  ({time.time() - t0:.0f} s)")
            dump(name, res)   # save as we go


# ------------------------------------------------------------------------ plots
def plots():
    s1d, s2d = load("s1"), load("s2")
    if s1d:
        offs = s1d["offsets_deg"]
        i20 = s1d["snr_db"].index(-20)
        fig, ax = plt.subplots(figsize=(7.5, 4.2))
        for N, r in s1d["results"].items():
            ax.plot(offs, r["per_offset"][i20], "o-", label=f"{N} drones (pooled {r['pooled'][i20]:.2f})")
        ax.set(xlabel="ring rotation relative to the target body (deg)", ylabel="Pd at 5% false alarm",
               title="Snapshot ring: detection depends on how the ring lines up (SNR -20 dB/sample)", ylim=(0, 1.05))
        ax.title.set_fontsize(10); ax.legend(fontsize=8); ax.grid(alpha=.3)
        fig.savefig(OUT / "s1_pd_vs_ring_orientation.png", dpi=130, bbox_inches="tight"); plt.close(fig)
        log("wrote s1_pd_vs_ring_orientation.png")
    if s1d or s2d:
        fig, ax = plt.subplots(figsize=(7.5, 4.2))
        for N, r in (s1d["results"] if s1d else {}).items():
            ax.plot(s1d["snr_db"], r["pooled"], "--", lw=1.2, label=f"snapshot, {N} drones")
        if s2d:
            ax.plot(s2d["snr_db"], s2d["pd"], "k-o", lw=2.2, label="orbit sweep, 6 drones")
        ax.set(xlabel="per-sample SNR of a unit point (dB)", ylabel="Pd at 5% false alarm",
               title="Orbit sweep vs snapshot ring (ABS shell, block reflectivity 0.5, coherent)", ylim=(0, 1.05))
        ax.title.set_fontsize(10); ax.legend(fontsize=8); ax.grid(alpha=.3)
        fig.savefig(OUT / "s2_sweep_vs_snapshot.png", dpi=130, bbox_inches="tight"); plt.close(fig)
        log("wrote s2_sweep_vs_snapshot.png")
    s3d = {}
    for part in ("s3a", "s3b", "s3c"):
        s3d.update(load(part) or {})
    if s3d:
        fig, ax = plt.subplots(figsize=(7.5, 4.2))
        for N, mk in [(4, "s"), (6, "o"), (12, "^")]:
            rows = sorted([v for v in s3d.values() if v["n_drones"] == N], key=lambda v: v["err_mm"])
            if not rows:
                continue
            e = [r["err_mm"] for r in rows]
            ax.plot(e, [r["pd_coherent"] for r in rows], mk + "-", color="C0", alpha=.35 + .1 * (N == 6),
                    label=f"coherent, {N} drones")
            ax.plot(e, [r["pd_incoherent"] for r in rows], mk + "-", color="C1", alpha=.35 + .1 * (N == 6),
                    label=f"incoherent, {N} drones")
        ax.set_xscale("log")
        ax.axvline(3.9 / 10, color="grey", ls=":"); ax.text(0.4, 0.04, " λ/10", fontsize=8)
        ax.set(xlabel="error in each drone's known position, constant per drone, 1σ per axis (mm)",
               ylabel="Pd at 5% false alarm", title="Orbit sweep: position-knowledge error (ABS shell, noise-free)",
               ylim=(0, 1.05))
        ax.title.set_fontsize(10); ax.legend(fontsize=7, ncol=2); ax.grid(alpha=.3, which="both")
        fig.savefig(OUT / "s3_sweep_position_error.png", dpi=130, bbox_inches="tight"); plt.close(fig)
        log("wrote s3_sweep_position_error.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", nargs="*", default=["s1", "s2", "s3a", "s3b", "plots"])
    args = ap.parse_args()
    t0 = time.time()
    trials = 4 if args.quick else 30
    if "s1" in args.only: s1(args.quick)
    if "s2" in args.only: s2(args.quick)
    if "s3a" in args.only:
        log("S3a: sweep, position error, N=6")
        s3("s3a", [(6, [0.25] if args.quick else [0.1, 0.25, 0.5, 1.0, 2.0])], trials, args.quick)
    if "s3b" in args.only:
        log("S3b: sweep, position error, N=4 and N=12")
        s3("s3b", [(4, [0.25]), (12, [0.25])] if args.quick else [(4, [0.25, 1.0]), (12, [0.25, 1.0])], trials, args.quick)
    if "s3c" in args.only:
        log("S3c: sweep, large position errors, N=6 (brackets where the incoherent mode fails)")
        s3("s3c", [(6, [5.0] if args.quick else [5.0, 10.0, 20.0])], trials, args.quick, seed=34)
    if "plots" in args.only: plots()
    log(f"done in {time.time() - t0:.0f} s")
