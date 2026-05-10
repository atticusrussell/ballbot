% scale(1000) import("chassis_plate.stl");

// Sketch PureShapes 2
multmatrix([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 2.000000;
translate([0, 0, -thickness]) {
  translate([-123.478922, -70.300639, 0]) {
    rotate([0, 0, 0.0]) {
      cube([242.000000, 140.695881, thickness]);
    }
  }
}
}
