"""Shell-material penetration model at mmWave.

Normal-incidence transmission/reflection of a single homogeneous dielectric
slab in air (transfer-matrix method). This answers the first feasibility
question: how much signal gets *through* a drone's shell and back out.

Values marked `assumed` are typical-order guesses, not measured data; replace
them with datasheet / measured values when you have them. See
docs/week1_findings.md for sources.
"""
from dataclasses import dataclass
import numpy as np

C = 299_792_458.0


@dataclass
class Material:
    name: str
    eps_r: float            # relative permittivity (real part)
    tan_d: float            # loss tangent
    source: str             # where the numbers come from
    fixed_T_dB: float | None = None  # override: one-way power transmission (dB)


# Dielectric shells (slab model applies)
MATERIALS = [
    Material("Polypropylene (PP)", 2.2, 0.0005, "eps: radar-cover patent table @76.5 GHz; tan_d assumed"),
    Material("ABS", 2.7, 0.007, "eps: radar-cover patent table @76.5 GHz; tan_d assumed (ABS is a lossy plastic)"),
    Material("Polycarbonate (PC)", 3.0, 0.006, "eps: radar-cover patent table; tan_d assumed"),
    Material("PEI", 3.1, 0.003, "Avient PREPERM bulletin @77 GHz"),
    Material("PBT + glass fibre", 3.8, 0.03, "Avient PREPERM bulletin @77 GHz (eps 3.5-4.2)"),
    # Conductive shells: slab dielectric model does not apply -> fixed values
    Material("PBT + carbon-fibre filler", 0, 0, "patent: <3% transmission 75-110 GHz", fixed_T_dB=-15.0),
    Material("CFRP laminate (assumed)", 0, 0, "assumed; real laminates are typically worse", fixed_T_dB=-30.0),
    Material("Aluminium / metal", 0, 0, "conductor: effectively opaque", fixed_T_dB=-200.0),
]


def slab_tr(eps_r: float, tan_d: float, thickness_m: float, freq_hz: float):
    """Complex amplitude (t, r) for a slab in air at normal incidence."""
    lam0 = C / freq_hz
    n = np.sqrt(eps_r * (1 - 1j * tan_d))      # e^{+jwt} convention -> Im(n) < 0 = lossy
    delta = 2 * np.pi * n * thickness_m / lam0
    m11 = np.cos(delta)
    m12 = 1j * np.sin(delta) / n
    m21 = 1j * n * np.sin(delta)
    m22 = np.cos(delta)
    den = m11 + m12 + m21 + m22
    t = 2 / den
    r = (m11 + m12 - m21 - m22) / den
    return t, r


def shell_factors(mat: Material, thickness_m: float = 2e-3, freq_hz: float = 77e9):
    """Return (interior_amp_factor, shell_reflect_amp, one_way_T_dB).

    interior_amp_factor = |t|^2 : an echo from inside the shell crosses it
    twice (in and out), so its amplitude is multiplied by |t| twice.
    """
    if mat.fixed_T_dB is not None:
        T_pow = 10 ** (mat.fixed_T_dB / 10)
        t_amp = np.sqrt(T_pow)
        r_amp = np.sqrt(max(0.0, 1 - T_pow)) if mat.fixed_T_dB < -100 else 0.8
        return t_amp ** 2, r_amp, mat.fixed_T_dB
    t, r = slab_tr(mat.eps_r, mat.tan_d, thickness_m, freq_hz)
    T_pow = abs(t) ** 2
    return abs(t) ** 2, abs(r), 10 * np.log10(T_pow)


def materials_table(thickness_m=2e-3, freq_hz=77e9):
    rows = []
    for m in MATERIALS:
        amp, refl, T_dB = shell_factors(m, thickness_m, freq_hz)
        rows.append(dict(material=m.name, one_way_T_dB=round(float(T_dB), 2),
                         two_way_interior_loss_dB=round(float(2 * T_dB), 2),
                         shell_reflection_amp=round(float(refl), 3), source=m.source))
    return rows
