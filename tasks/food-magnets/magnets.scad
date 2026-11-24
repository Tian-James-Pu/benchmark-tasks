// Printable models of the six magnets (review TK-9NQBPY note: "Provide printable magnet models or drawings").
// OpenSCAD 2021.01 or later. Pick a part and render (F6), then export STL:
//   openscad -D 'part="cookie"' -o cookie.stl magnets.scad
// Parts: strawberry, cookie, donut, plain (the note's magnet), round, square (the two ordinary magnets), all (preview).
// Units mm. Frame: x across (the board's y), y up (the board's z), z out of the board (thickness); back face at z = 0.
// Print back face down, PLA, 0.2 mm layers, 4 perimeters, 40 % infill (the masses in task.json are estimates for that).
// Each magnet gets one Master Magnetics 07002 ceramic disc (12.7 dia x 4.75) pressed and glued into its back pocket,
// flush with the back face. Sizes are params.json's (the simulation uses the same outlines, geometry.py).

part = "all";

POCKET_D = 12.9;        // 07002 disc 12.7 + 0.2 clearance (params.json food.donut.pocket_dia)
POCKET_H = 4.85;        // disc 4.75 + 0.1, flush back face
$fn = 96;

module pocket(x = 0, y = 0) {
    translate([x, y, -0.01]) cylinder(d = POCKET_D, h = POCKET_H + 0.01);
}

// strawberry: 30 wide x 38 tall x 12 thick; outline = geometry.outline('strawberry'): an ellipse narrowed to 0.55 of its
// width at the tip (bottom), full width at the top; front domed (12 at the middle, 6 at the rim); 3 seed dimples
SW = 30; SH = 38; ST = 12;
function _sy(t) = cos(t) * (0.55 + 0.45 * (sin(t) + 1) / 2);
_SYMAX = max([for (t = [0 : 360 / 96 : 359.99]) abs(_sy(t))]);
function straw_outline() = [for (t = [0 : 360 / 96 : 359.99]) [_sy(t) / _SYMAX * SW / 2, sin(t) * SH / 2]];

module strawberry() {
    color([0.80, 0.10, 0.12]) difference() {
        union() {
            linear_extrude(height = ST / 2) polygon(straw_outline());
            // dome: the outline extruded on to the full thickness while shrinking to 40 %
            translate([0, 0, ST / 2]) linear_extrude(height = ST / 2, scale = 0.4) polygon(straw_outline());
        }
        pocket(0, 0);
        for (p = [[-6, 4], [6, 4], [0, -6]])          // seed dimples, 2.4 dia, on the dome
            translate([p[0], p[1], ST * 0.86]) sphere(d = 2.4);
    }
}

// cookie: 40 dia x 8, tan, 6 dark chip bumps (5 dia, 1.5 proud) on its front; print the chips in a second colour
// (filament change at the 8.0 mm layer) or as separate parts glued on
CD = 40; CT = 8;
module cookie() {
    difference() {
        color([0.80, 0.60, 0.36]) cylinder(d = CD, h = CT);
        pocket(0, 0);
    }
    color([0.30, 0.18, 0.10]) for (a = [0 : 60 : 359]) {
        r = (a % 120 == 0) ? 11 : 6;
        translate([r * cos(a + 20), r * sin(a + 20), CT]) scale([1, 1, 0.6]) sphere(d = 5);
    }
}

// doughnut: 40 OD, 6 hole, 12 thick, rounded edges; tan body with a pink icing top 3 thick (colour change at 9.0 mm);
// the hole leaves no room for the disc at the centre, so the pocket sits 11.5 off-centre (params food.donut)
DO = 40; DI = 6; DT = 12; ICING = 3;
module donut_body(h0, h1) {
    // rounded section between heights h0 and h1 of a 12 thick ring (edge radius 4)
    intersection() {
        rotate_extrude() hull() {
            translate([DI / 2 + 4, 4]) circle(r = 4);
            translate([DO / 2 - 4, 4]) circle(r = 4);
            translate([DI / 2 + 4, DT - 4]) circle(r = 4);
            translate([DO / 2 - 4, DT - 4]) circle(r = 4);
        }
        translate([-DO, -DO, h0]) cube([2 * DO, 2 * DO, h1 - h0]);
    }
}
module donut() {
    difference() {
        union() {
            color([0.80, 0.60, 0.36]) donut_body(0, DT - ICING);
            color([0.93, 0.52, 0.68]) donut_body(DT - ICING, DT);
        }
        pocket(11.5, 0);
    }
}

// the note's plain magnet A: 40 dia x 8, matte black, no surface detail
module plain() { color([0.08, 0.08, 0.08]) difference() { cylinder(d = 40, h = 8); pocket(0, 0); } }

// ordinary magnets (r5): a plain 30 dia x 8 disc and a plain 30 x 30 x 8 square (corner radius 3), matte black
module round_magnet() { color([0.08, 0.08, 0.08]) difference() { cylinder(d = 30, h = 8); pocket(0, 0); } }
module square_magnet() {
    color([0.08, 0.08, 0.08]) difference() {
        linear_extrude(height = 8) offset(r = 3) square([30 - 6, 30 - 6], center = true);
        pocket(0, 0);
    }
}

if (part == "strawberry") strawberry();
else if (part == "cookie") cookie();
else if (part == "donut") donut();
else if (part == "plain") plain();
else if (part == "round") round_magnet();
else if (part == "square") square_magnet();
else {
    translate([-60, 0, 0]) strawberry();
    translate([0, 0, 0]) cookie();
    translate([60, 0, 0]) donut();
    translate([-60, -60, 0]) plain();
    translate([0, -60, 0]) round_magnet();
    translate([60, -60, 0]) square_magnet();
}
