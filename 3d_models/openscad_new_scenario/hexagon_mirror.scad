module hexagon_mirror(r_hex, h_hex, th){    
    r = r_hex*0.99;
    lh = r_hex - r;
    h = h_hex - 4*lh;
    
  difference(){
    cylinder(h = h, r1 = r, r2 = r, center = true, $fn=6);
    translate([0,0,h/2])
        cylinder(h = h-th/2, r1 = r-th, r2 = r-th, center = true, $fn=6);
  }
    translate([0,0,-h/2])
        rotate([180,0,0])
            cylinder(h = 4*lh, r1 = 5*lh, r2 = 4*lh, center = true, $fn=6);
}

h_hex=0.05;
th=0.025;
translate([0,0,h_hex/2])
    hexagon_mirror(r_hex=0.5, h_hex=h_hex, th=0.025);
