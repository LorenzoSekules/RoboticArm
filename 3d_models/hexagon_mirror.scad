use <hexagon.scad>

R = 0.5;
translate([0,0,0.05+0.03/2])
//rotate([0,0,150])
union(){
hexagon_tile(R, h_center_grip=0.05,R_grips_center=0.1);
for (i = [1:4]) {
    rotate([0,0,(i-1)*60])
    translate([0,cos(30)*2*R+0.03-1e-2,0])
    hexagon_tile(R); 
}

}