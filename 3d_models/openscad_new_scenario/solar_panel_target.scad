module solar_panel(l=0.25, w=0.15, h=0.01, hubl=0.05, hubw=0.05, hubh=0.01){

union(){
translate([0,0,l/2+hubl-1e-6])
    cube([w,h,l],center=true);
translate([0,0,hubl/2])    
    cube([hubw,hubh,hubl],center=true);
}    
  
    
//translate([0,hubcw/2,0])
//  union(){
//    cube([hubcl,hubcw,hubch],center=true);
//  translate([-sl/2,0,-sh/2]){
//    translate([sl*0.2, 0, -(conh-sh)/2])
//      cube([conl,(sw+ssep)*4-ssep,conh]);
//    translate([sl*0.8-conl, 0, -(conh-sh)/2])
//      cube([conl,(sw+ssep)*4-ssep,conh]);
//    for(i=[0:3]){
//      translate([0, (sw+ssep)*i, 0]){
//        cube([sl,sw,sh]);
//        translate([sep, sep, 0])
//          for(i=[0:2])
//            for(j=[0:1])
//              translate([i*(cl+sep), j*(cw+sep), 0])
//                cube([cl,cw,ch]);
//        }
//    }
//  }
//  }
//}
}
// rotate([90,0,0])
    solar_panel(l=0.6, w=0.35, h=0.02, hubl=0.02, hubw=0.06, hubh=0.01);