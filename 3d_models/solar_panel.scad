sl = 0.25;
sw = 0.15;
sh = 0.015;
ssep = 0.01;
Nl = 3;
Nw = 2;
sep = 0.01;

cl = (sl-(Nl+1)*sep)/Nl;
cw = (sw-(Nw+1)*sep)/Nw;
ch = sh*1.1;
echo(cl);
echo(cw);

conl = 0.1;
conh = 0.01;

hubcl = 0.05;
hubcw = 0.05;
hubch = 0.01;
translate([0,hubcw/2,0])
  union(){
    cube([hubcl,hubcw,hubch],center=true);
  translate([-sl/2,0,-sh/2]){
    translate([sl*0.2, 0, -(conh-sh)/2])
      cube([conl,(sw+ssep)*4-ssep,conh]);
    translate([sl*0.8-conl, 0, -(conh-sh)/2])
      cube([conl,(sw+ssep)*4-ssep,conh]);
    for(i=[0:3]){
      translate([0, (sw+ssep)*i, 0]){
        cube([sl,sw,sh]);
        translate([sep, sep, 0])
          for(i=[0:2])
            for(j=[0:1])
              translate([i*(cl+sep), j*(cw+sep), 0])
                cube([cl,cw,ch]);
        }
    }
  }
  }