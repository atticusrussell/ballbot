% scale(1000) import("webcam_body.stl");

// Sketch PureShapes 24
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, -5.45438235443059], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 24.000000;
translate([0, 0, -thickness]) {
  translate([39.867013, 16.844005, 0]) {
    rotate([0, 0, -180.0]) {
      cube([79.734025, 33.688010, thickness]);
    }
  }
}
}

// Sketch PureShapes 17
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 16.5], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 17.000000;
translate([0, 0, -thickness]) {
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=10.000000,h=thickness);
  }
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=6.000000,h=thickness);
  }
}
}
