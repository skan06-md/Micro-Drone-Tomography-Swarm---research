"""Multistatic stepped-frequency radar on a circle of nodes + backprojection.

Signal model (Born approximation, isotropic point scatterers, no multipath):
    s_ij(f) = sum_k a_k * exp(-j 2 pi f (|Tx_i - p_k| + |p_k - Rx_j|) / c)

Image formation (delay-and-sum backprojection):
    I(x) = sum_pairs sum_f s_ij(f) * exp(+j 2 pi f (|Tx_i - x| + |x - Rx_j|) / c)
"""
from dataclasses import dataclass, field
import numpy as np

C = 299_792_458.0


@dataclass
class RadarConfig:
    n_nodes: int = 6
    radius: float = 3.0          # orbit radius (m)
    f_center: float = 77e9
    bandwidth: float = 4e9
    n_freq: int = 64
    include_monostatic: bool = True
    max_bistatic_deg: float = 120.0  # drop near-opposite pairs: there the Tx direct
                                     # path arrives at the same delay as the echo

    @property
    def freqs(self):
        return self.f_center + np.linspace(-0.5, 0.5, self.n_freq) * self.bandwidth

    def node_positions(self, offset_rad=0.0):
        th = offset_rad + 2 * np.pi * np.arange(self.n_nodes) / self.n_nodes
        return self.radius * np.c_[np.cos(th), np.sin(th)]

    def pairs(self):
        n = self.n_nodes
        out = []
        for i in range(n):
            for j in range(i if self.include_monostatic else i + 1, n):
                sep = abs(i - j) * 360.0 / n
                if min(sep, 360 - sep) <= self.max_bistatic_deg + 1e-9:
                    out.append((i, j))
        return out

    def orbit_looks(self, arc_deg=None, step_deg=0.2):
        """Node positions at each time step while the ring rotates.

        By default each drone sweeps its own slot (360/N degrees), so together
        the ring covers the full circle in one pass -- circular SAR by a swarm.
        """
        arc = 360.0 / self.n_nodes if arc_deg is None else arc_deg
        offs = np.deg2rad(np.arange(0, arc, step_deg))
        return [self.node_positions(o) for o in offs]


@dataclass
class ImageGrid:
    half_width: float = 0.22
    pixel: float = 0.003
    X: np.ndarray = field(init=False)
    Y: np.ndarray = field(init=False)

    def __post_init__(self):
        ax = np.arange(-self.half_width, self.half_width + 1e-9, self.pixel)
        self.X, self.Y = np.meshgrid(ax, ax)

    @property
    def points(self):
        return np.c_[self.X.ravel(), self.Y.ravel()]


def simulate(cfg: RadarConfig, pts, amps, nodes=None):
    """Return data array (n_pairs, n_freq) for scatterers pts (K,2) with amps (K,)."""
    nodes = cfg.node_positions() if nodes is None else nodes
    k = 2 * np.pi * cfg.freqs / C
    dist = np.linalg.norm(nodes[:, None, :] - pts[None, :, :], axis=2)   # (N, K)
    out = np.empty((len(cfg.pairs()), cfg.n_freq), complex)
    for p, (i, j) in enumerate(cfg.pairs()):
        path = dist[i] + dist[j]                                       # (K,)
        out[p] = np.exp(-1j * np.outer(k, path)) @ amps
    return out


def range_profiles(cfg: RadarConfig, data, upsample=16):
    """IFFT each pair's frequency data -> baseband profile vs bistatic path length."""
    f = cfg.freqs
    df = f[1] - f[0]
    n_fft = cfg.n_freq * upsample
    prof = np.fft.ifft(data, n=n_fft, axis=1) * n_fft          # sum_n s_n e^{+j2pi n df tau}
    path_axis = np.arange(n_fft) * C / (df * n_fft)            # path length for each bin
    period = C / df                                            # unambiguous path length
    return prof, path_axis, period


def backproject(cfg: RadarConfig, grid: ImageGrid, data, nodes=None, mode="coherent"):
    """Backprojection image on the grid, normalised so a unit point peaks at ~1.

    mode="coherent":   sum_pairs profile(path) * carrier phase  -> ~lambda/4 resolution,
                       needs sub-mm knowledge of node positions.
    mode="incoherent": sum_pairs |profile(path)|                -> ~range-resolution
                       (few cm), tolerant to cm-level position errors.
    """
    nodes = cfg.node_positions() if nodes is None else nodes
    prof, path_axis, period = range_profiles(cfg, data)
    f0 = cfg.freqs[0]
    P = grid.points
    dist = np.linalg.norm(nodes[:, None, :] - P[None, :, :], axis=2)      # (N, Npix)
    dpath = path_axis[1]
    img = np.zeros(len(P), complex if mode == "coherent" else float)
    for p, (i, j) in enumerate(cfg.pairs()):
        path = dist[i] + dist[j]
        u = np.mod(path, period) / dpath
        i0 = np.floor(u).astype(int)
        w = u - i0
        i0 %= prof.shape[1]
        i1 = (i0 + 1) % prof.shape[1]
        val = (1 - w) * prof[p, i0] + w * prof[p, i1]
        if mode == "coherent":
            img += val * np.exp(1j * 2 * np.pi * f0 * path / C)
        else:
            img += np.abs(val)
    img /= len(cfg.pairs()) * cfg.n_freq
    return img.reshape(grid.X.shape)


def image_looks(cfg: RadarConfig, grid: ImageGrid, pts, amps, looks, monostatic_only=True,
                mode="coherent"):
    """Noise-free image summed over many looks (time steps of an orbiting ring)."""
    sub = RadarConfig(**{**cfg.__dict__, "max_bistatic_deg": 0.0 if monostatic_only else cfg.max_bistatic_deg})
    img = 0
    for nodes in looks:
        img = img + backproject(sub, grid, simulate(sub, pts, amps, nodes), nodes, mode)
    return img / len(looks)
