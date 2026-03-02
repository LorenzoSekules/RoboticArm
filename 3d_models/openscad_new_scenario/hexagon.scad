module hexagon_tile(R, H=0.03, R_side_grip=0.015, h_side_grip=0.03, h_center_grip=0, r_center_grip=0){
//translate([0,sin(60)*R+h_side_grip/2,0])    
// sin(60)*R+0.05/2 em y
union(){
//difference(){    
cylinder(h = H, r1 = R, r2 = R, center = true, $fn=6);
//translate([0,0,-H/2])
//cylinder(h = H, r1 = R-0.05, r2 = R-0.05, center = true, $fn=6);
//}
for (i = [1:6]){
    rotate([0,0,(i-1)*60])
    translate([0,cos(30)*R+h_side_grip/2,0])
    rotate([90,0,0])
    cylinder(h = h_side_grip, r1 = 0.8*R_side_grip, r2 = R_side_grip, center = true, $fn=4); 
}
translate([0,0,-(h_center_grip+H)/2])
cylinder(h = h_center_grip, r1 = r_center_grip, r2 = r_center_grip, center = true, $fn=6); 
}
}

hexagon_tile(R=1);
