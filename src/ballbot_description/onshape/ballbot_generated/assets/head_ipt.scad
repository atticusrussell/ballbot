% scale(1000) import("head_ipt.stl");

// Sketch PureShapes 20
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 20.000000;
translate([0, 0, -thickness]) {
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=33.612261,h=thickness);
  }
  translate([-0.000000, 0.000000, 0]) {
    cylinder(r=34.612042,h=thickness);
  }
}
}
