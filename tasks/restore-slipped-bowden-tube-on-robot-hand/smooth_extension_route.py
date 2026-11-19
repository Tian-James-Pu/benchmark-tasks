# SPDX-License-Identifier: CERN-OHL-S-2.0
"""Smooth initial tube route between source-defined straight guide sections.

The Boolean cutter is clearance geometry, not a tube shape. Retain its end
locations and guide tangents, then use a cubic Hermite curve through the open
middle span. This is an assumed initial shape, not a hardware measurement.
The 66.5624 mm polygonal tube has 0.1 mm axial clearance at both shoulders.
Source and corresponding editable CAD: see extension_route.py and NOTICE.
"""
import numpy as np

from extension_route import POINTS as CLEARANCE_ROUTE

POINTS = CLEARANCE_ROUTE.copy()
START, END = 6, 21
a, b = CLEARANCE_ROUTE[START], CLEARANCE_ROUTE[END]
height = b[2]-a[2]
slope_a = (a[:2]-CLEARANCE_ROUTE[START-1, :2])/(a[2]-CLEARANCE_ROUTE[START-1, 2])
slope_b = (CLEARANCE_ROUTE[END+1, :2]-b[:2])/(CLEARANCE_ROUTE[END+1, 2]-b[2])
for i in range(START, END+1):
    t = (POINTS[i, 2]-a[2])/height
    POINTS[i, :2] = ((2*t**3-3*t**2+1)*a[:2]
                    + (t**3-2*t**2+t)*height*slope_a
                    + (-2*t**3+3*t**2)*b[:2]
                    + (t**3-t**2)*height*slope_b)
POINTS[0] += .0001*(POINTS[1]-POINTS[0])/np.linalg.norm(POINTS[1]-POINTS[0])
POINTS[-1] -= .0001*(POINTS[-1]-POINTS[-2])/np.linalg.norm(POINTS[-1]-POINTS[-2])
