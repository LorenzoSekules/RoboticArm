module hexagon_tile(R, h_center_grip=0,R_grips_center=0){

H = 0.03;
R_grips = H/2;
h_grips = 0.03;

//translate([0,sin(60)*R+h_grips/2,0])    
// sin(60)*R+0.05/2 em y
union(){
//difference(){    
cylinder(h = H, r1 = R, r2 = R, center = true, $fn=6);
//translate([0,0,-H/2])
//cylinder(h = H, r1 = R-0.05, r2 = R-0.05, center = true, $fn=6);
//}
for (i = [1:6]){
    rotate([0,0,(i-1)*60])
    translate([0,cos(30)*R,0])
    rotate([90,0,0])
    cylinder(h = h_grips, r1 = 0.8*R_grips, r2 = R_grips, center = true, $fn=4); 
}
translate([0,0,-(h_center_grip+H)/2])
cylinder(h = h_center_grip, r1 = R_grips_center, r2 = R_grips_center, center = true, $fn=6); 
}
}

hexagon_tile(R=1);
