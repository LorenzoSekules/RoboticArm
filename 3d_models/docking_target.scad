union(){
  
cylinder(h=0.15,r=0.2,center=true,$fn=50);

for (i = [1:6]){

    rotate([0,0,(i-1)*60])
    translate([0,cos(30)*(0.15),0.15/2])
    cylinder(h=0.2,r=0.03, center = true, $fn=50); 
}
    
} 
