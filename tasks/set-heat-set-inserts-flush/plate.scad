// The sensor plate for this task: 40 x 80 x 12 mm, four blind 4.0 mm holes 7.0 mm deep on a 24 x 60 mm pattern,
// for CNC Kitchen M3 standard heat-set inserts (5.7 mm long; hole 4.0 mm, about 1 mm deeper than the insert).
// Print flat in PETG: 0.2 mm layers, 4 perimeters, 30 % infill (about 40 g, 25 min). Export with: openscad -o plate.stl plate.scad
$fn = 64;
difference() {
    translate([-20, -40, 0]) cube([40, 80, 12]);
    for (x = [-12, 12]) for (y = [-30, 30])
        translate([x, y, 12 - 7]) cylinder(d = 4.0, h = 7.01);
}
