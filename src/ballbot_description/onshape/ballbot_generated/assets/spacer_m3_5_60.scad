% scale(1000) import("spacer_m3_5_60.stl");

// Sketch PureShapes 60
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 60.000000;
translate([0, 0, -thickness]) {
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=2.500000,h=thickness);
  }
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=2.500000,h=thickness);
  }
}
}
