"""Orbit-sweep aperture: the ring rotates and every drone images as it flies.

With N drones each sweeping its own 360/N-degree slot, the swarm covers the
full circle in one pass (circular SAR flown by a swarm). Monostatic looks only:
each drone listens to its own pulse, so no inter-drone phase sync is needed for
this mode; what matters instead is how well each drone knows its own position.

Because the full circle is covered, the aperture looks the same from every
direction, so the target's orientation is modelled as an angular offset of the
look angles (rotating the target by +phi == rotating the aperture by -phi).
"""
import numpy as np

from .radar import RadarConfig, ImageGrid, C


class SweepAperture:
    def __init__(self, cfg: RadarConfig, grid: ImageGrid, step_deg=0.5, offset_deg=0.0):
        self.cfg, self.grid = cfg, grid
        self.angles = np.deg2rad(offset_deg + np.arange(0, 360, step_deg))
        self.nodes = cfg.radius * np.c_[np.cos(self.angles), np.sin(self.angles)]
        L = len(self.angles)
        self.drone_of_look = (np.arange(L) * cfg.n_nodes) // L   # which drone flew each look
        f = cfg.freqs
        self.k = 2 * np.pi * f / C
        self.f0 = f[0]
        self.n_fft = cfg.n_freq * 16
        df = f[1] - f[0]
        self.period = C / df
        self.dpath = self.period / self.n_fft

    @property
    def n_looks(self):
        return len(self.angles)

    def perturbed_nodes(self, rng, sigma_m):
        """True positions when each drone has a constant unknown offset (per axis, 1 sigma)."""
        bias = rng.normal(0, sigma_m, (self.cfg.n_nodes, 2))
        return self.nodes + bias[self.drone_of_look]

    def simulate(self, pts, amps, nodes=None, chunk=24):
        """Monostatic data (n_looks, n_freq) for point scatterers."""
        nodes = self.nodes if nodes is None else nodes
        out = np.empty((len(nodes), self.cfg.n_freq), np.complex128)
        a = amps.astype(np.complex64)
        for s in range(0, len(nodes), chunk):
            d = 2 * np.linalg.norm(nodes[s:s + chunk, None, :] - pts[None, :, :], axis=2)
            ph = np.mod(d[:, None, :] * self.k[None, :, None], 2 * np.pi).astype(np.float32)
            out[s:s + chunk] = np.exp(-1j * ph) @ a
        return out

    def noise(self, rng):
        """Unit-variance complex white noise with the shape of one sweep's data."""
        shp = (self.n_looks, self.cfg.n_freq)
        return (rng.normal(size=shp) + 1j * rng.normal(size=shp)) / np.sqrt(2)

    def backproject(self, data, nodes=None, incoherent=False):
        """Image using the positions the imager *believes* (default: nominal).

        Returns the coherent complex image, or (coherent, incoherent) if asked.
        A unit point scatterer peaks at about 1 in both.
        """
        nodes = self.nodes if nodes is None else nodes
        prof = np.fft.ifft(data, n=self.n_fft, axis=1) * self.n_fft
        P = self.grid.points
        coh = np.zeros(len(P), np.complex128)
        inc = np.zeros(len(P)) if incoherent else None
        kc = 2 * np.pi * self.f0 / C
        for l in range(len(nodes)):
            path = 2 * np.hypot(P[:, 0] - nodes[l, 0], P[:, 1] - nodes[l, 1])
            u = np.mod(path, self.period) / self.dpath
            i0 = u.astype(np.int64)
            w = u - i0
            i0 %= self.n_fft
            i1 = (i0 + 1) % self.n_fft
            val = (1 - w) * prof[l, i0] + w * prof[l, i1]
            coh += val * np.exp(1j * kc * path)
            if incoherent:
                inc += np.abs(val)
        norm = len(nodes) * self.cfg.n_freq
        shape = self.grid.X.shape
        if incoherent:
            return coh.reshape(shape) / norm, inc.reshape(shape) / norm
        return coh.reshape(shape) / norm


def rotate(pts, deg):
    t = np.deg2rad(deg)
    R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    return pts @ R.T
