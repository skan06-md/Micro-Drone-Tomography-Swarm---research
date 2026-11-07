"""2D top-down model of a target quadcopter as point scatterers.

Coordinates in metres, target centred at the origin, the swarm orbits in the
same plane. Scatterers are grouped so images can be formed per group and
combined linearly later (image = exterior + T * (interior + rho * payload)).

Amplitudes are in arbitrary units *per centimetre* of edge, so a shape's
total echo doesn't depend on how finely it is sampled.
"""
import numpy as np

SPACING = 2e-3  # 2 mm between scatterer points along edges


def _segment(p0, p1, amp_per_cm):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(2, int(np.linalg.norm(p1 - p0) / SPACING))
    s = np.linspace(0, 1, n, endpoint=False)[:, None]
    pts = p0 + s * (p1 - p0)
    amps = np.full(n, amp_per_cm * SPACING / 1e-2)
    return pts, amps


def rect_outline(cx, cy, w, h, amp_per_cm):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    P, A = zip(*[_segment(corners[i], corners[i + 1], amp_per_cm) for i in range(4)])
    return np.vstack(P), np.concatenate(A)


def circle_outline(cx, cy, r, amp_per_cm):
    n = max(8, int(2 * np.pi * r / SPACING))
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.c_[cx + r * np.cos(th), cy + r * np.sin(th)]
    return pts, np.full(n, amp_per_cm * SPACING / 1e-2)


BODY_HALF = 0.075  # body shell is a 15 cm x 15 cm box


def shell(shell_reflect_amp: float = 1.0):
    return rect_outline(0, 0, 2 * BODY_HALF, 2 * BODY_HALF, shell_reflect_amp)


def frame():
    """Arms + motors."""
    parts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            corner = (sx * BODY_HALF, sy * BODY_HALF)
            motor = (sx * 0.18, sy * 0.18)
            parts.append(_segment(corner, motor, 0.3))          # arm tube
            parts.append(circle_outline(*motor, 0.014, 1.0))    # metal motor bell
    P, A = zip(*parts)
    return np.vstack(P), np.concatenate(A)


def exterior(shell_reflect_amp: float):
    """Shell + arms + motors (everything outside the shell)."""
    (P1, A1), (P2, A2) = shell(shell_reflect_amp), frame()
    return np.vstack([P1, P2]), np.concatenate([A1, A2])


def interior_nominal():
    """What a normal drone has inside: LiPo battery (foil = strong) + flight controller."""
    parts = [rect_outline(0.0, -0.035, 0.09, 0.035, 0.8),   # battery
             rect_outline(-0.035, 0.035, 0.035, 0.035, 0.3)]  # flight controller PCB
    P, A = zip(*parts)
    return np.vstack(P), np.concatenate(A)


# Free interior spots where an unexpected block could sit (not overlapping battery/PCB)
PAYLOAD_POSITIONS = [(0.03, 0.035), (0.035, 0.02), (0.02, 0.045), (0.04, 0.04), (0.0, 0.04), (0.03, 0.005)]
PAYLOAD_SIZE = 0.03  # 3 cm block


def payload(pos, amp_per_cm=1.0):
    return rect_outline(pos[0], pos[1], PAYLOAD_SIZE, PAYLOAD_SIZE, amp_per_cm)


def interior_mask(X, Y, margin=0.008):
    lim = BODY_HALF - margin
    return (np.abs(X) < lim) & (np.abs(Y) < lim)
