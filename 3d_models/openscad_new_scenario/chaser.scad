use <hexagon_mirror.scad>
use <solar_panel.scad>
use <hexagon.scad>

R_side_grip = 0.015;
h_side_grip = 0.01;

module target(R_hub = 0.25, H_hub = 1){
hexagon_mirror(R=0.5, h_center_grip=0.25, r_center_grip=0.3,R_side_grip = 0.015,h_side_grip = 0.01);

translate([0, 0, -H_hub/2])
cylinder(h = H_hub, r1 = R_hub, r2 = R_hub, center = true, $fn=6);

translate([0, 0, -H_hub/2])
cylinder(h = 0.02*H_hub, r1 = 1.2*R_hub, r2 = 1.2*R_hub, center = true, $fn=6);

for(i=[1:3])
    rotate([0,0,(i-1)*120])
        translate([0,R_hub*cos(30),-H_hub+0.05])
            rotate([0, 180, 0])
        solar_panel(sl=0.6, sw=0.35, sh=0.01, conl=0.01, conh=0.005, hubcl=0.02, hubcw=0.06, hubch=0.008, Nl=3, Nw=2);
}

module chaser(w_hub, h_hub, l_hub){
translate([0, 0, -l_hub/2])
//    difference(){
        cube([w_hub, h_hub, l_hub], center=true);
//        translate([-0.1, 0, -0.3])
//            cube([1.1*w_hub, 0.9*h_hub, 0.3*l_hub], center=true);
//    }
    
//for(i=[1:2])
//    rotate([0,0,(i-1)*180])
//        translate([0,h_hub/2,-l_hub+0.3])
//            rotate([0, 90, 0])
//        solar_panel(sl=0.4, sw=0.3, sh=0.01, conl=0.01, conh=0.005, hubcl=0.02, hubcw=0.06, hubch=0.008, Nl=3, Nw=2);

l_hub_back = 0.05;
translate([0.2*w_hub, 0, -l_hub-l_hub_back/2])
cube([1*w_hub, h_hub, l_hub_back], center=true);
translate([0.65*w_hub, 0, -l_hub+l_hub_back/2-w_hub])
cube([0.15*w_hub, h_hub, l_hub_back], center=true);
rotate([0,90,0])
translate([+w_hub/2+l_hub,0,0.7*w_hub])
cube([1*w_hub, h_hub, l_hub_back], center=true);

for (i = [1:1]){
        translate([-0.1/2+0.032, 0, -l_hub_chaser-0.1-(i-1)*0.1+0.1])
        rotate([0,0,30])
        hexagon_tile(H=0.03, R=0.5,R_side_grip = 0.015,
h_side_grip = 0.01);
        }
        
for (i = [1:7]){       
        translate([0.5-0.1/2, 0, -l_hub_chaser-0.1-(i-1)*0.07+0.1])
        rotate([0,90,0])
        cylinder(h = 0.05, r1 = 0.01, r2 = 0.01, center = true, $fn=12);
        }
}

l_hub_chaser=0.950;
R_target = 0.3;
H_target = 1.1;
//target(R_hub=R_target, H_hub=H_target);

// rotate([0,0,60])
//translate([0, 0.4+cos(30)*R_target, -H_target*0.5])
//   rotate([90,90,0]) {
    rotate([0,90,0]) 
    translate([0,0,l_hub_chaser/2])
        chaser(w_hub = 0.794-0.1, h_hub = 0.970-0.1, l_hub=l_hub_chaser-0.1);      
//   }