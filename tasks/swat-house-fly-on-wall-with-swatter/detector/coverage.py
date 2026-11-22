"""Where the 13-sensor detector fires, compared with the simulator's 15 cm rule.

    python coverage.py

Frame: the fly's centre is the origin; u points straight out of the wall, v along the wall (the robot's +y, to the left as the
robot sees it), w up. Millimetres. Each VL53L1X sees a 27 degree cone (13.5 degrees half-angle) from its window and reports the
nearest target in it; the sketch fires when any sensor reads below its threshold. The layout and thresholds here are the ones
in detector.ino and WIRING.md.

For a target brought straight in towards the fly from a direction (elevation from the wall's normal, azimuth around it), the
script prints the distance from the fly at which the detector first fires: for a point (a fingertip), and for a 100 x 100 mm card
held face-on to the fly (the calibration card; the swatter's head is 127 x 119 mm). The simulator's rule is 150 mm in every
direction.
"""

import numpy as np

HALF = np.radians(13.5)
TABLE_W = -140.0  # the table top, below the fly


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def tilted(elev_deg, az):
    e = np.radians(elev_deg)
    return unit([np.cos(e), np.sin(e) * np.cos(az), np.sin(e) * np.sin(az)])


# S0 straight out; S1-S4 tilted 45 degrees towards +v, +w, -v, -w (22 mm from the fly); S5-S8 tilted 70 degrees towards the
# diagonals +v+w, -v+w, -v-w, +v-w (36 mm from the fly); S9-S12 tilted 75 degrees towards +v, +w, -v, -w (40 mm from the fly)
SENSORS = [("S0", np.array([0.0, 9.0, -9.0]), unit([1, 0, 0]))]  # (name, window position, axis)
for i in range(4):
    a = i * np.pi / 2
    SENSORS.append((f"S{1 + i}", np.array([0.0, 22 * np.cos(a), 22 * np.sin(a)]), tilted(45, a)))
for i in range(4):
    a = np.pi / 4 + i * np.pi / 2
    SENSORS.append((f"S{5 + i}", np.array([0.0, 36 * np.cos(a), 36 * np.sin(a)]), tilted(70, a)))
for i in range(4):  # S9-S12: tilted 75 degrees towards +v, +w, -v, -w (40 mm from the fly), for approaches along the wall
    a = i * np.pi / 2
    SENSORS.append((f"S{9 + i}", np.array([0.0, 40 * np.cos(a), 40 * np.sin(a)]), tilted(75, a)))


def on_axis_threshold(p, d, dist=150.0):
    """The range along the sensor's axis at which a target is `dist` from the fly."""
    b = float(p @ d)
    return -b + np.sqrt(b * b - (p @ p - dist * dist))


def table_range(p, d):
    """The nearest the table top comes to a sensor's window inside its cone (mm), or infinity."""
    a = unit(np.cross(d, [0, 1, 0]) if abs(d[1]) < 0.9 else np.cross(d, [0, 0, 1]))
    b = np.cross(d, a)
    best = np.inf
    for t in np.linspace(0, HALF, 15):
        for ph in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            ray = np.cos(t) * d + np.sin(t) * (np.cos(ph) * a + np.sin(ph) * b)
            if ray[2] < -1e-6:
                best = min(best, (TABLE_W - p[2]) / ray[2])
    return best


TABLE_MARGIN = 15.0  # mm: a downward-looking sensor's threshold stays this far short of the table top
THRESH = {n: min(on_axis_threshold(p, d), table_range(p, d) - TABLE_MARGIN) for n, p, d in SENSORS}


def fires(points):
    for n, p, d in SENSORS:
        v = points - p
        r = np.linalg.norm(v, axis=-1)
        ok = (r < THRESH[n]) & ((v @ d) > r * np.cos(HALF))
        if ok.any():
            return True
    return False


def card(center, normal, size=100.0, n=21):
    a = unit(np.cross(normal, [0, 0, 1]) if abs(normal[2]) < 0.9 else np.cross(normal, [0, 1, 0]))
    b = np.cross(normal, a)
    s = np.linspace(-size / 2, size / 2, n)
    g = np.array([center + x * a + y * b for x in s for y in s])
    return g[g[:, 0] >= 0]  # the wall stops a card


def trigger(direction, kind):
    for D in np.arange(220.0, 0.0, -1.0):
        if kind == "point":
            pts = (D * direction)[None]
        else:  # the card's nearest point D from the fly, face-on to it
            pts = card(D * direction, -direction)
        pts = pts[pts[:, 2] >= TABLE_W]
        if len(pts) and fires(pts):
            return D
    return 0.0


if __name__ == "__main__":
    print("sensor  window (u, v, w) mm    axis elevation  axis azimuth  table in cone mm  threshold mm")
    for n, p, d in SENSORS:
        az = np.degrees(np.arctan2(d[2], d[1])) % 360 if d[0] < 0.999 else 0.0
        print(f"{n:6s}  {p[0]:5.0f} {p[1]:5.0f} {p[2]:5.0f}        {np.degrees(np.arccos(d[0])):5.0f}        {az:5.0f}"
              f"        {table_range(p, d):8.0f}      {THRESH[n]:5.0f}")
    print("\nfirst firing distance from the fly (mm), point / 100 mm card; the simulator: 150 everywhere")
    azs = [("+v (side, robot's left)", 0.0), ("+v+w diagonal", np.pi / 4), ("+w (from above)", np.pi / 2),
           ("-v+w diagonal", 3 * np.pi / 4), ("-v (side, robot's right)", np.pi), ("-v-w diagonal", 5 * np.pi / 4),
           ("-w (from below)", 3 * np.pi / 2), ("+v-w diagonal", 7 * np.pi / 4)]
    elevs = [0, 20, 30, 45, 60, 70, 80, 85]
    print(f"{'approach azimuth':26s}" + "".join(f"{e:>10d}deg" for e in elevs))
    for name, a in azs:
        row = []
        for e in elevs:
            dvec = tilted(e, a)
            row.append(f"{trigger(dvec, 'point'):5.0f}/{trigger(dvec, 'card'):<5.0f}")
        print(f"{name:26s}" + "".join(f"{r:>13s}" for r in row))
    print("\nthe swatter's head (127 x 119 mm) held flat, parallel to the wall, slid in along it towards the fly")
    gy, gz = np.meshgrid(np.linspace(-63.5, 63.5, 26), np.linspace(-59.5, 59.5, 25))
    for u in (5, 10, 30, 60):
        out = []
        for name, sv, sw in (("+v", 1, 0), ("-v", -1, 0), ("+w", 0, 1), ("-w", 0, -1)):
            for D in np.arange(220.0, 0.0, -1.0):
                v = sv * (D + 63.5) + gy.ravel() if sv else gy.ravel()
                w = sw * (D + 59.5) + gz.ravel() if sw else gz.ravel()
                pts = np.c_[np.full(v.size, float(u)), v, w]
                pts = pts[pts[:, 2] >= TABLE_W]
                if len(pts) and fires(pts):
                    break
            out.append(f"from {name}: {np.hypot(D, u):4.0f}")
        print(f"  {u:3d} mm off the wall, nearest point from the fly when it fires (mm): " + ", ".join(out))
