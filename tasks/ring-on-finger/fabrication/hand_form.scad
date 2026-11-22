// Hand form core + ring-finger skin mould for ring-on-finger. Units mm. Frame: hand plane = y-z, palm faces -x,
// z = height above the plate top minus 10 (z 0 = table), ring finger axis at x = y = 0, web at z = 130.
$fn = 64;
web = 130;
U = [-3, 0, 10, 18, 28, 36, 44, 52, 60, 64.5];
Dk = [19.0, 17.0, 17.0, 17.3, 18.0, 17.0, 16.0, 16.4, 15.6, 15.0];
// crest = PIP crest diameter; 18.0 is the task's skin, 17.9 is the fallback mould insert.
module ring_finger_skin(crest = 18.0) {   // revolved outer profile, u from web
  Dc = [for (i = [0:len(Dk)-1]) i == 4 ? crest : Dk[i]];
  translate([0, 0, web]) rotate_extrude() polygon(concat([[0, U[0]]], [for (i = [0:len(U)-1]) [Dc[i]/2, U[i]]],
    [for (a = [0:10:90]) [7.5*cos(a), 64.5 + 7.5*sin(a)]], [[0, 72]]));
}
module bone() { translate([0, 0, web - 10]) cylinder(h = 10 + 64.5, d = 9.6); translate([0, 0, web + 64.5]) sphere(d = 9.6); }
module finger(by, bz, tilt, L, d) { translate([0, by, bz]) rotate([-tilt, 0, 0]) { cylinder(h = L - d/2, d = d); translate([0, 0, L - d/2]) sphere(d = d); } }
module core() {
  translate([-14, -52, 35]) cube([28, 80, 95]);                 // palm, y -52..28, z 35..130
  difference() { translate([-16, -27.5 - 12, 10]) cube([32, 55, 25]);   // wrist stub on the plate
    for (y = [-30, 6]) translate([0, y, 9]) cylinder(h = 13, d = 6.4); }  // M5 heat-set insert bores, 12 deep
  finger(-20, web - 3, -12, 78, 17);                            // middle (tilt toward -y), base 3 mm into the palm
  finger(-38, web, -22, 70, 16.5);                              // index
  finger(18, web - 8, 12, 58, 14.5);                            // little
  finger(-45, 70, -45, 60, 20);                                 // thumb
  bone();
}
module ring_mould(crest = 18.0) { difference() { translate([-16, -16, web - 5]) cube([32, 32, 82]); ring_finger_skin(crest); } } // split at x = 0; v2 insert: ring_mould(17.9)
// Base plate drawing and the reset template.
module plate() {                    // 150 x 150 x 10 mild steel, centred 20 mm behind the ring finger (x +20)
  difference() { translate([20 - 75, -75, 0]) cube([150, 150, 10]);
    for (y = [-30, 6]) translate([0, y, -1]) { cylinder(h = 12, d = 5.5); cylinder(h = 3.7, d1 = 12.4, d2 = 5.0); } } // M5 countersunk from below
}
module template() {                 // 2D, print 1:1 on A3 (420 x 297) and tape to the table at the rig's fixed marks
  // plate outline at the nominal pose, 1 mm ticks to +/-15 mm along its front (-x) and right (-y) edges;
  // ring circle (d 21.55, the ring's OD) at (-100, +160) from the ring-finger axis, 1 mm ticks
  // to +/-20 mm in x and y. Draws: ../draws.csv.
  difference() { translate([-55, -75]) square([150, 150]); translate([-54.5, -74.5]) square([149, 149]); }
  for (i = [-15:15]) { translate([-55 + i, -78]) square([0.3, i % 5 == 0 ? 6 : 3]); translate([-61, -75 + i]) square([i % 5 == 0 ? 6 : 3, 0.3]); }
  translate([-100, 160]) difference() { circle(d = 21.55); circle(d = 20.95); }
  for (i = [-20:20]) { translate([-100 + i, 135]) square([0.3, i % 5 == 0 ? 6 : 3]); translate([-125, 160 + i]) square([i % 5 == 0 ? 6 : 3, 0.3]); }
}
// Push-test lever clamp, PLA. Lower jaw 70 x 24 x 8 mm with a hinge post (two cheeks) and a push
// handle; upper jaw 62 x 24 x 8 mm with a tongue on a 4 mm steel pin at x -6. Each jaw holds one spare YAM gripper pad in a
// 20 x 20 x 3 mm recess centred at x 40; the jaws' pad faces are one ring OD (21.55 mm) apart. The upper jaw's 3 mm hook hole
// is at x 40, directly above the pad centre, so the hung mass acts on the pad's line: lever ratio 1:1 (params.json
// gates.G1_lever_ratio, G1_lever_arm_mm). Hung mass 612 g including the upper jaw's own weight -> 6.0 N.
module lever_clamp(od = 21.55) {
  top = 8 + od; pin_z = top + 4;
  difference() {
    union() { translate([-10, -12, 0]) cube([70, 24, 8]);
      for (y = [-12, 6]) translate([-10, y, 0]) cube([8, 6, pin_z + 5]);          // hinge cheeks
      translate([-50, -6, 0]) cube([40, 12, 8]); }                                // push handle
    translate([30, -10, 5]) cube([20, 20, 3.01]);                                 // lower pad recess
    translate([-6, -13, pin_z]) rotate([-90, 0, 0]) cylinder(h = 26, d = 4.2);    // pin bore
  }
  difference() {
    union() { translate([-2, -12, top]) cube([62, 24, 8]); translate([-10, -5.5, top]) cube([8, 11, 8]); } // jaw + tongue
    translate([30, -10, top - 0.01]) cube([20, 20, 3.01]);                        // upper pad recess (bottom face)
    translate([40, 0, top - 1]) cylinder(h = 10, d = 3);                          // hook hole above the pad centre
    translate([-6, -7, pin_z]) rotate([-90, 0, 0]) cylinder(h = 14, d = 4.2);     // pin bore
  }
}

core();
%ring_finger_skin();
// plate();      // export separately for the plate drawing
// template();   // export separately (2D) for the reset template
// lever_clamp(); // push-test fixture, export separately
