// Cookie replica for real trials: the same profile as the simulated cookie (task.py, COOKIE_PROFILE).
// 57 mm across, 10 mm thick, underside flat to a 49 mm circle, then rounded up to the rim.
// Print in PETG, flat side down on a smooth bed, 0.2 mm layers, 2 perimeters, 3 bottom and top layers, gyroid infill.
// Start at 35 % infill and adjust until each replica weighs 16 +/- 1 g on a kitchen scale (solid would be about 28 g).
// Volume: 22.1 cm3. Print six: four for the trial and two spares.

$fn = 120;
profile_mm = [[0, 0], [24.5, 0], [26.8, 0.7], [28.1, 2.0], [28.5, 4.0], [27.9, 6.5], [25.5, 8.5], [18.0, 9.6], [0, 10.0]];

rotate_extrude() polygon(profile_mm);
