R_side_grip = 0.015;
h_side_grip = 0.01;
R_hub = 0.3;
H_hub = 1.1;


translate([0, 0, -H_hub/2])
cylinder(h = H_hub, r1 = R_hub, r2 = R_hub, center = true, $fn=6);

translate([0, 0, -H_hub/2])
cylinder(h = 0.02*H_hub, r1 = 1.2*R_hub, r2 = 1.2*R_hub, center = true, $fn=6);

