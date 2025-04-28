% scale(1000) import("webcam_sensor.stl");

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
