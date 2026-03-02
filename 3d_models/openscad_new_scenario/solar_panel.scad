
module solar_panel(sl=0.25, sw=0.15, sh=0.01, conl=0.1, conh=0.01, hubcl=0.05, hubcw=0.05, hubch=0.01, N=4){
ssep = 0.01;
sep = 0.01;

//cl = (sl-(Nl+1)*sep)/Nl;
//cw = (sw-(Nw+1)*sep)/Nw;
//ch = sh*1.1;

translate([0,hubcw/2,0])
  union(){
    cube([hubcl,hubcw,hubch],center=true);
  translate([-sl/2,0,-sh/2]){
    translate([sl*0.2, 0, -(conh-sh)/2])
      cube([conl,(sw+ssep)*N-ssep,conh]);
    translate([sl*0.8-conl, 0, -(conh-sh)/2])
      cube([conl,(sw+ssep)*N-ssep,conh]);
    for(i=[0:N-1]){
      translate([0, (sw+ssep)*i, 0]){
        cube([sl,sw,sh]);
//        translate([sep, sep, 0])
//          for(i=[0:2])
//            for(j=[0:1])
//              translate([i*(cl+sep), j*(cw+sep), 0])
//                cube([cl,cw,ch]);
        }
    }
  }
  }
}
//rotate([90,0,0])
   color([0.3,0.3,1]) solar_panel(sl=0.6, sw=0.35, sh=0.01, conl=0.01, conh=0.005, hubcl=0.02, hubcw=0.06, hubch=0.008, N=4);