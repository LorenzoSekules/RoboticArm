use <hexagon_truss.scad>

module hexagon_mirror(r_hex, h_hex, th){

union(){
    
hexagon_truss(r_hex, h_hex, th);
    
//for (i= [-1:1]) 
//    translate([i*2*r,0,0])
//                hexagon_truss(r, h, th, thw); 
//for (i= [-1, 1]) 
//    translate([i*r,2*r*cos(30),0])
//                hexagon_truss(r, h, th, thw); 
//for (i= [1]) 
//    translate([i*r,-2*r*cos(30),0])
//                hexagon_truss(r, h, th, thw); 
    
// first layer    
for (i = [2, 3, 4, 5, 6]) {
    rotate([0,0,i*60+30])
        translate([0,2*r_hex-1e-4,0])
            rotate([0,0,0])
                rotate([0,0,30])
                    hexagon_truss(r_hex, h_hex, th); 
}


// second layer    
//for (i = [1, 2, 3, 4, 5, 6]) {
//    rotate([0,0,i*60+30])
//        translate([0,4*r,0])
//            rotate([0,0,30])
//                hexagon_truss(r, h, th, thw); 
//}
//for (i = [1, 2, 3, 4, 5, 6]) {
//    rotate([0,0,i*60+0])
//        translate([0,4*cos(30)*r,0])
//            rotate([0,0,0])
//                hexagon_truss(r, h, th, thw); 
//}

}
}

h_hex = 0.15;
translate([0, 0, h_hex/2])
   color([0.8, 0.8, 0.8]) hexagon_mirror(r_hex=0.5, h_hex=h_hex, th=0.025);