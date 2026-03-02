module hexagon_truss(r_hex, h_hex, th){    
    r = r_hex*0.99;   
    lh = r_hex - r;
     h = h_hex - 4*lh;
    
    difference(){    
        cylinder(h = h, r1 = r, r2 = r, center = true, $fn=6);
    triangle_l = r/sqrt(3)-th;
    displacement = r*cos(30)-(r+th)/sqrt(3)/2;
    for (i = [0:5])
        rotate([0,0,i*60+30])
                translate([displacement,0,0])
                rotate([0,0,60]){                
                cylinder(h = h+1e-3, r1 = triangle_l, r2 = triangle_l, center = true, $fn=3);
            }
//        }
    for (i = [0:5])
        rotate([0,0,i*60+30])
            translate([r*cos(30)-th/2,0,0])
                rotate([0,0,90])
                    cube([r-2*th, th+1e-3, h-th],center=true);
                
    for (i = [0:5])
        rotate([0,0,i*60])
            translate([(r-th/2)/2,0,0])
                cube([r-3*th, th, h-th],center=true);
    

}
    for (i = [0:5])
        rotate([0,0,i*60])
            translate([r-lh,0,0])
                rotate([0,90,0])
                cylinder(h = 4*lh, r1 = 3.5*lh, r2 = 3*lh, center = true, $fn=6);
    
    translate([0,0,h/2])
        cylinder(h = 4*lh, r1 = 3.5*lh, r2 = 3*lh, center = true, $fn=6);
    translate([0,0,-h/2])
        rotate([180,0,0])
            cylinder(h = 4*lh, r1 = 3.5*lh, r2 = 3*lh, center = true, $fn=6);
}

r_hex=0.5;
rotate([180, 0, 0])
translate([0,0,r_hex])
    rotate([90,30,0])
        color([0.8, 0.8, 0.8])   hexagon_truss(r_hex=r_hex, h_hex=0.15, th=0.025);
