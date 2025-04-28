% scale(1000) import("board_ipt.stl");

// Sketch PureShapes 2
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 2.000000;
translate([0, 0, -thickness]) {
  translate([-5.500000, 10.250000, 0]) {
    cylinder(r=1.500000,h=thickness);
  }
  translate([-5.500000, -10.250000, 0]) {
    cylinder(r=1.500000,h=thickness);
  }
  translate([6.000000, -11.430000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, -8.890000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, -6.350000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, -3.810000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, -1.270000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, 1.270000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, 3.810000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, 6.350000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, 8.890000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
  translate([6.000000, 11.430000, 0]) {
    cylinder(r=0.500000,h=thickness);
  }
}
}
