# Floor fixture and removable cover

This is a specified construction for the custom fixture represented in `task.py`. Dimensions are in millimetres;
they are design dimensions, not measurements of a built specimen. The cover's 1.05 kg mass and material friction
are simulation estimates. No physical load or displacement test has been performed on this slotted cover.

## Materials and coordinates

Use structural plywood: APA C-C Exterior, Group 1, nominal 3/4 inch for the cover and nominal 23/32 inch for the
deck. The model rounds these thicknesses to 19 and 18 mm respectively. Record the panel grade stamp, actual
thickness and assembled cover mass before a physical trial; a materially different assembly requires matching
the simulation and repeating validation. The cover's face grain runs along world x. Use straight, dry No. 2 or
better softwood, planed to the dimensions below, for the sleepers and cleats. These specifications identify
materials; they do not assign a load rating to the slotted assembly.

World x points forward, y left, and z up. All frame members stand 64 mm high on the slab. Their x/y bounds below
use the same world origin as the task. The plywood deck lies on top, so its walking surface is 82 mm above the
slab. The 200 mm square opening has bounds x = 200–400 and y = −100–100.

## Frame cut list

| Part | Quantity | Cut dimensions | x bounds | y bounds |
| --- | --- | --- | --- | --- |
| Near perimeter sleeper | 1 | 620 × 38 × 64 | 100–138 | −170–450 |
| Far perimeter sleeper | 1 | 620 × 38 × 64 | 462–500 | −170–450 |
| Right perimeter sleeper | 1 | 324 × 38 × 64 | 138–462 | −170–−132 |
| Left perimeter sleeper | 1 | 324 × 38 × 64 | 138–462 | 412–450 |
| Near opening blocking | 1 | 270 × 38 × 64 | 162–200 | −132–138 |
| Far opening blocking | 1 | 270 × 38 × 64 | 400–438 | −132–138 |
| Right opening blocking | 1 | 200 × 32 × 64 | 200–400 | −132–−100 |
| Left opening blocking | 1 | 200 × 38 × 64 | 200–400 | 100–138 |
| Intermediate sleeper | 1 | 324 × 38 × 64 | 138–462 | 271–309 |

The simulator uses overlapping fixed boxes at the right side of the opening. The 270 mm lengths and 32 mm width
above eliminate the physical overlap while preserving their combined solid shape exactly. Do not cut all opening
blocking to 276 × 38 mm and force the pieces together.

Cut the deck as one 400 × 620 mm panel with the opening centred 200 mm from the x edge and 170 mm from the
negative-y edge. Its footprint is x = 100–500, y = −170–450. The simulator's four adjacent plywood boxes represent
this same panel area. Fasten the deck down with countersunk 4 × 50 mm wood screws along each sleeper and blocking
centreline, at no more than 100 mm spacing and at least 20 mm from each member end. Predrill to suit the chosen
wood and screw. The deck ties the members into a fixed fixture; it is not a free-standing structural floor.
Clamp the completed frame to the trial table outside the robot's swept area, with the frame and basket positions
matching the setup image. Fixture clamps must neither obstruct the task nor provide a robot support point.

## Cover cut list and assembly

Cut a 300 × 300 × 19 mm plate. Cut the centred slot 110 mm along x and 36 mm along y; smooth its edges without
enlarging the opening. The slot is a custom handling feature. Its dimensions alone do not establish a safe
walking-surface cover or its load capacity.

Cut two 180 × 38 × 19 mm cleats. In cover-centred coordinates, their bounds are x = −90–90, with y = 56–94 for
one and −94–−56 for the other. Their upper faces touch the plate's underside. Plane a 6 × 6 mm, 45-degree bevel
along each cleat's lower outside longitudinal edge. The remaining straight outside wall is 13 mm high. Do not
bevel the upper locating edges or the cleat ends.

Attach each cleat with three countersunk 4 × 35 mm wood screws driven from the plate's top at x = −60, 0, 60 mm
and y = +75 or −75 mm. The heads finish flush with the plate. The specified 35 mm screw fits within the nominal
38 mm combined wood thickness; check the actual assembled thickness and ensure no point projects below a cleat.
These six screws are a construction specification, not a verified fastener-capacity calculation. The simulator
represents the assembled cover as one rigid body and does not model screw deformation or failure.

Paint the upper face safety orange, stencil HOLE in 50 mm letters beyond the slot's far end as shown in the setup
image, and record the finished mass. When centred, the cleats have 10 mm clearance at each x end and 6 mm at each
outer y face. The plate has 50 mm overlap beyond each edge of the opening. Mark its seated outline on the deck.

## Evidence limits for the intended use

The Construction Safety Council's [Fall Protection Hazard Awareness Guide](https://www.elcosh.org/document/1680/d000593/Fall%20Protection%20Hazard%20Awareness%20Guide.html)
describes cleats as a possible way to secure covers, with additional restraint where uplift can occur.
[OSHA 1926.502(i)](https://www.osha.gov/laws-regs/regulations/standardnumber/1926/1926.502)
also requires adequate strength for the imposed load and prevention of accidental displacement. Neither source
rates this particular slotted and bevelled construction.

The outstanding physical evidence is the selected panel/fastener assembly's load capacity and its resistance to
the applicable displacement and uplift loads. The hand slot, grain orientation and actual support layout must
be included in that assessment. An uncut panel's rating, or a rigid-body simulation that does not deform, cannot
by itself establish the modified cover's capacity. This document specifies the trial fixture; it does not
certify it for use as an occupied-site protective cover.
