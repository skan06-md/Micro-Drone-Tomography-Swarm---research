"""Anomaly detection Monte Carlo: is there an unexpected block inside the shell?

Detector (deliberately simple, a baseline to beat later):
  1. Know what a *normal* drone of this model looks like -> noise-free reference image.
  2. Statistic = max over the interior of the change from the reference:
     coherent mode   -> |I_test - I_ref|   (complex / coherent change detection)
     incoherent mode -> I_test - I_ref     (magnitude images)
  3. Threshold set for a 5% false-alarm rate using payload-free trials.
  Pd = fraction of payload trials above threshold.

The data are linear in the scatterers, so per radar config we simulate
exterior / interior / payload data once and combine them with weights:
  data = D_ext + T * (D_int + rho * D_pay) + noise
"""
from dataclasses import dataclass
import numpy as np

from . import scene
from .radar import RadarConfig, ImageGrid, simulate, backproject


def interior_grid(mode):
    return ImageGrid(half_width=scene.BODY_HALF + 0.005, pixel=0.00075 if mode == "coherent" else 0.002)


@dataclass
class SceneData:
    cfg: RadarConfig
    D_ext: np.ndarray
    D_int: np.ndarray
    D_pay: list

    @classmethod
    def build(cls, cfg, shell_reflect_amp, nodes=None):
        e = scene.exterior(shell_reflect_amp)
        i = scene.interior_nominal()
        return cls(cfg, simulate(cfg, *e, nodes), simulate(cfg, *i, nodes),
                   [simulate(cfg, *scene.payload(p), nodes) for p in scene.PAYLOAD_POSITIONS])


def run_detection(cfg, shell_T_amp, shell_reflect_amp, rho, snr_db, mode="coherent",
                  n_trials=100, pos_err_m=0.0, rng=None, sd_nominal=None):
    """Return (Pd at Pfa=5%, H0 stats, H1 stats)."""
    rng = np.random.default_rng(0) if rng is None else rng
    grid = interior_grid(mode)
    mask = scene.interior_mask(grid.X, grid.Y)
    sd0 = sd_nominal or SceneData.build(cfg, shell_reflect_amp)
    ref = backproject(cfg, grid, sd0.D_ext + shell_T_amp * sd0.D_int, mode=mode)
    sigma = 10 ** (-snr_db / 20)
    nominal_nodes = cfg.node_positions()

    def one(with_payload):
        if pos_err_m > 0:   # true positions differ from what the imager assumes
            true_nodes = nominal_nodes + rng.normal(0, pos_err_m, nominal_nodes.shape)
            sd = SceneData.build(cfg, shell_reflect_amp, true_nodes)
        else:
            sd = sd0
        d = sd.D_ext + shell_T_amp * sd.D_int
        if with_payload:
            d = d + shell_T_amp * rho * sd.D_pay[rng.integers(len(sd.D_pay))]
        noise = sigma * (rng.normal(size=d.shape) + 1j * rng.normal(size=d.shape)) / np.sqrt(2)
        img = backproject(cfg, grid, d + noise, mode=mode)
        # coherent: complex change detection |I - I_ref|; incoherent: magnitude increase
        diff = np.abs(img - ref) if mode == "coherent" else img - ref
        return float(np.max(diff[mask]))

    h0 = np.array([one(False) for _ in range(n_trials)])
    h1 = np.array([one(True) for _ in range(n_trials)])
    thr = np.quantile(h0, 0.95)
    return float(np.mean(h1 > thr)), h0, h1
