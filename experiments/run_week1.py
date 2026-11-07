"""Week 1 feasibility experiments. Run from the repo root:

    python experiments/run_week1.py            # full run (~10-20 min on a laptop)
    python experiments/run_week1.py --quick    # fewer trials, for a smoke test

Outputs go to results/: PNG figures + summary.json
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mdts import scene
from mdts.materials import MATERIALS, materials_table, shell_factors, slab_tr
from mdts.radar import RadarConfig, ImageGrid, simulate, backproject, image_looks
from mdts.detect import SceneData, run_detection

OUT = Path(__file__).resolve().parents[1] / "results"
OUT.mkdir(exist_ok=True)
MAT = {m.name: m for m in MATERIALS}
ABS_AMP, ABS_REFL, _ = shell_factors(MAT["ABS"])


def save(fig, name):
    fig.savefig(OUT / name, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", name)


# ---------------------------------------------------------------- E1 materials
def e1_materials():
    print("E1 materials")
    rows = materials_table()
    fig, ax = plt.subplots(figsize=(7, 4))
    th = np.linspace(0.3e-3, 5e-3, 300)
    for name in ["Polypropylene (PP)", "ABS", "PEI", "PBT + glass fibre"]:
        m = MAT[name]
        T = [10 * np.log10(abs(slab_tr(m.eps_r, m.tan_d, t, 77e9)[0]) ** 2) for t in th]
        ax.plot(th * 1e3, T, label=name)
    for name, ls in [("PBT + carbon-fibre filler", "--"), ("CFRP laminate (assumed)", ":")]:
        ax.axhline(MAT[name].fixed_T_dB, ls=ls, color="k", lw=1, label=name)
    ax.set(xlabel="shell thickness (mm)", ylabel="one-way power transmission (dB)",
           title="How much 77 GHz gets through a drone shell", ylim=(-35, 1))
    ax.legend(fontsize=8, loc="lower left"); ax.grid(alpha=.3)
    save(fig, "e1_shell_transmission.png")
    return rows


# ---------------------------------------------------------------- E2 images
def block_avg(img, k):
    h, w = (img.shape[0] // k) * k, (img.shape[1] // k) * k
    return img[:h, :w].reshape(h // k, k, w // k, k).mean(axis=(1, 3))


def component_images(cfg, grid, aperture, step_deg):
    """Complex images of each scene part (unit weights); combine linearly later."""
    parts = dict(shell=scene.shell(1.0), frame=scene.frame(), interior=scene.interior_nominal(),
                 payload=scene.payload(scene.PAYLOAD_POSITIONS[0]))
    out = {}
    for k, (P, A) in parts.items():
        if aperture == "snapshot":
            out[k] = backproject(cfg, grid, simulate(cfg, P, A), mode="coherent")
        else:
            out[k] = image_looks(cfg, grid, P, A, cfg.orbit_looks(step_deg=step_deg))
    return out


def compose(c, mat_name, rho):
    amp, refl, _ = shell_factors(MAT[mat_name])
    return refl * c["shell"] + c["frame"] + amp * (c["interior"] + rho * c["payload"])


def e2_gallery(step_deg):
    print("E2 image gallery")
    grid = ImageGrid(half_width=0.21, pixel=0.0006)
    ext = [-0.21, 0.21, -0.21, 0.21]
    K = 4  # display block-average (0.6 mm -> 2.4 mm) to avoid moire

    # point-spread functions
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    for r, mode in enumerate(["coherent", "incoherent"]):
        for c, n in enumerate([6, 12, 36]):
            cfg = RadarConfig(n_nodes=n)
            I = block_avg(np.abs(backproject(cfg, grid, simulate(cfg, np.array([[0.0, 0.0]]), np.array([1.0])), mode=mode)), K)
            axs[r, c].imshow(20 * np.log10(I / I.max() + 1e-6), extent=ext, origin="lower", vmin=-30, vmax=0, cmap="magma")
            axs[r, c].set_title(f"{mode}, {n} drones ({len(cfg.pairs())} pairs)", fontsize=9)
    fig.suptitle("Point-spread function: image of a single point (dB)")
    save(fig, "e2_point_spread.png")

    apertures = [("6 drones, one snapshot", RadarConfig(n_nodes=6), "snapshot"),
                 ("36 drones, one snapshot", RadarConfig(n_nodes=36), "snapshot"),
                 (f"6 drones, each sweeps 60° (step {step_deg}°, monostatic)", RadarConfig(n_nodes=6), "sweep")]
    comps = []
    for title, cfg, ap in apertures:
        t = time.time()
        comps.append((title, component_images(cfg, grid, ap, step_deg)))
        print(f"   {title}: {time.time() - t:.0f} s")

    for mat in ["ABS", "PBT + carbon-fibre filler", "Aluminium / metal"]:
        fig, axs = plt.subplots(3, 3, figsize=(12, 12))
        for r, (title, c) in enumerate(comps):
            a, b = np.abs(compose(c, mat, 0)), np.abs(compose(c, mat, 0.5))
            d = np.abs(compose(c, mat, 0.5) - compose(c, mat, 0))
            vmax = block_avg(b, K).max()
            for col, (img, lab) in enumerate([(a, "normal drone"), (b, "with 3 cm block"), (d, "difference")]):
                im = block_avg(img, K)
                axs[r, col].imshow(im / (vmax if col < 2 else max(im.max(), 1e-12)), extent=ext,
                                   origin="lower", cmap="inferno", norm=PowerNorm(0.5, vmin=0, vmax=1))
                axs[r, col].set_title(f"{title}\n{lab}", fontsize=8)
                axs[r, col].add_patch(plt.Rectangle((-scene.BODY_HALF,) * 2, 2 * scene.BODY_HALF, 2 * scene.BODY_HALF,
                                                    fill=False, ec="cyan", lw=.6, ls="--"))
                px, py = scene.PAYLOAD_POSITIONS[0]
                if col:
                    axs[r, col].add_patch(plt.Rectangle((px - .015, py - .015), .03, .03, fill=False, ec="lime", lw=.6))
        fig.suptitle(f"Shell: {mat}. Coherent images, 3 m orbit, 77 GHz / 4 GHz BW, noise-free.\n"
                     "cyan = body shell, green = where the hidden block is (brightness: square-root scale)")
        save(fig, "e2_gallery_" + mat.split()[0].lower().replace("/", "") + ".png")


# ---------------------------------------------------------------- E3-E6 detection
def e3_shell(trials):
    print("E3 Pd vs shell transmission")
    cfg = RadarConfig(n_nodes=6)
    T_dB = np.array([0, -3, -6, -10, -15, -20, -25, -30])
    res = {}
    for mode in ["coherent", "incoherent"]:
        sd = SceneData.build(cfg, 0.5)
        for rho in [1.0, 0.3]:
            key = f"{mode}, block reflectivity {rho}"
            res[key] = [run_detection(cfg, 10 ** (t / 10), 0.5, rho, snr_db=-10, mode=mode,
                                      n_trials=trials, sd_nominal=sd)[0] for t in T_dB]
            print("   ", key, res[key])
    fig, ax = plt.subplots(figsize=(7, 4))
    for k, v in res.items():
        ax.plot(T_dB, v, "o-", label=k)
    for name, x in [("plastics", -1), ("PBT+CF", -15), ("CFRP?", -30)]:
        ax.axvline(x, color="grey", lw=.8, ls=":"); ax.text(x, 1.03, name, fontsize=8, ha="center")
    ax.set(xlabel="one-way shell transmission (dB)", ylabel="Pd at 5% false alarm",
           title="Can we detect a hidden 3 cm block? (6 drones, SNR -10 dB/sample)", ylim=(0, 1.1))
    ax.invert_xaxis(); ax.legend(fontsize=8); ax.grid(alpha=.3)
    save(fig, "e3_pd_vs_shell.png")
    return dict(T_dB=T_dB.tolist(), pd=res)


def e4_snr(trials):
    print("E4 Pd vs SNR")
    snr = np.array([-35, -30, -25, -20, -15, -10, -5, 0])
    res = {}
    for n in [5, 6, 8]:
        cfg = RadarConfig(n_nodes=n)
        sd = SceneData.build(cfg, ABS_REFL)
        res[f"{n} drones"] = [run_detection(cfg, ABS_AMP, ABS_REFL, 0.5, s, "coherent", trials, sd_nominal=sd)[0] for s in snr]
        print("   ", n, res[f"{n} drones"])
    fig, ax = plt.subplots(figsize=(7, 4))
    for k, v in res.items():
        ax.plot(snr, v, "o-", label=k)
    ax.set(xlabel="per-sample SNR of a unit point (dB)", ylabel="Pd at 5% false alarm",
           title="Detection vs SNR (ABS shell, block reflectivity 0.5, coherent)", ylim=(0, 1.05))
    ax.legend(); ax.grid(alpha=.3)
    save(fig, "e4_pd_vs_snr.png")
    return dict(snr_db=snr.tolist(), pd=res)


def e5_nodes(trials):
    print("E5 Pd vs number of drones")
    ns = [3, 4, 5, 6, 8, 12]
    res = {}
    for mode in ["coherent", "incoherent"]:
        res[mode] = []
        for n in ns:
            cfg = RadarConfig(n_nodes=n)
            res[mode].append(run_detection(cfg, ABS_AMP, ABS_REFL, 0.5, -20, mode, trials)[0])
        print("   ", mode, res[mode])
    fig, ax = plt.subplots(figsize=(7, 4))
    for k, v in res.items():
        ax.plot(ns, v, "o-", label=k)
    ax.set(xlabel="number of drones in the ring", ylabel="Pd at 5% false alarm",
           title="Detection vs swarm size (ABS shell, SNR -20 dB/sample)", ylim=(0, 1.05))
    ax.legend(); ax.grid(alpha=.3)
    save(fig, "e5_pd_vs_drones.png")
    return dict(n_drones=ns, pd=res)


def e6_position(trials):
    print("E6 Pd vs position-knowledge error")
    errs_mm = np.array([0, 0.1, 0.25, 0.5, 1, 2, 5, 10, 20])
    cfg = RadarConfig(n_nodes=6)
    res = {}
    for mode in ["coherent", "incoherent"]:
        res[mode] = [run_detection(cfg, ABS_AMP, ABS_REFL, 0.5, 0, mode, trials, pos_err_m=e * 1e-3)[0] for e in errs_mm]
        print("   ", mode, res[mode])
    fig, ax = plt.subplots(figsize=(7, 4))
    for k, v in res.items():
        ax.semilogx(np.maximum(errs_mm, 0.05), v, "o-", label=k)
    ax.axvline(3.9 / 10, color="grey", ls=":"); ax.text(0.39, 0.05, " λ/10", fontsize=8)
    ax.set(xlabel="error in each drone's known position, 1σ per axis (mm)  [0 plotted at 0.05]",
           ylabel="Pd at 5% false alarm", title="How well must each drone know where it is? (6 drones, ABS, high SNR)",
           ylim=(0, 1.05))
    ax.legend(); ax.grid(alpha=.3, which="both")
    save(fig, "e6_pd_vs_position_error.png")
    return dict(pos_err_mm=errs_mm.tolist(), pd=res)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", nargs="*", default=None, help="subset of: e1 e2 e3 e4 e5 e6")
    ap.add_argument("--trials", type=int, default=None)
    args = ap.parse_args()
    trials = args.trials or (30 if args.quick else 60)
    run = lambda name: args.only is None or name in args.only
    t0 = time.time()
    summary = dict(config=dict(f_center_GHz=77, bandwidth_GHz=4, n_freq=64, orbit_radius_m=3,
                               block_size_cm=3, trials_per_hypothesis=trials, pfa=0.05))
    if run("e1"): summary["E1_materials"] = e1_materials()
    if run("e2"): e2_gallery(step_deg=0.5 if args.quick else 0.2)
    if run("e3"): summary["E3_shell"] = e3_shell(trials)
    if run("e4"): summary["E4_snr"] = e4_snr(trials)
    if run("e5"): summary["E5_nodes"] = e5_nodes(trials)
    if run("e6"): summary["E6_position"] = e6_position(max(30, trials // 2))
    # merge with any previous summary so partial runs accumulate
    path = OUT / "summary.json"
    old = json.loads(path.read_text()) if path.exists() else {}
    old.update(summary)
    path.write_text(json.dumps(old, indent=2))
    print(f"done in {time.time() - t0:.0f} s")
