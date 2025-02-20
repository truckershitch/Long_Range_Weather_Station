/*
Weather Station Deck Cap Mounting Flange
For 1.25 inch Schedule 40 PVC Pipe

Created August 30, 2024
Modified August 30, 2024
*/

// Pipe or Rod Mounting Flange
// (c) 2013, Christopher "ScribbleJ" Jansen


include <custom.scad>

// Minimum Thickness of Object
min_thick = 5;
// Diameter of Pipe/Rod
pipe_d    = 1.660 * 25.4 + 1; // 1 1/4" sch 40 PVC





// Diameter of Flange
flange_d  = 100;
// Height of Object
height    = 45;

// Number of Setscrew Holes
num_setscrews = 3;
// Number of Flange Bolts
num_bolts = 4;

// Diameter of Flange Bolt
bolt_d    = 3;
// Diameter of Flange Bolthead
bolthead_d = 6;

// Diameter of "Setscrew" Bolt
setscrew_d = 3;
// Diameter of "Setscrew" Bolthead
setscrewhead_d = 6;

/* [Hidden] */

$fs = 0.2;

s = flange_d - pipe_d - min_thick*2;

difference()
{
  union()
  {
    cylinder(r=pipe_d/2 + min_thick, h=height);
    // cylinder(r=flange_d/2, h=s/2 + min_thick);
    // translate([0, 0, s/4 + min_thick/2])
      extruded_rounded_rect(size=[flange_d, flange_d, s/2 + min_thick], radius=20, center=true);
  }

  // Pipehole
  translate([0,0,-1]) cylinder(r=pipe_d/2, h=height+2);

  // Bolts
  // if(num_bolts > 0)
  // {
  //   for(x=[0:360/num_bolts:num_bolts*(360/num_bolts)])
  //   {
  //     rotate([0,0,x]) 
  //     {
  //       translate([pipe_d/2 + min_thick + s/4,0,-1]) 
  //       {
  //         cylinder(r=bolt_d/2, h=height);
  //         translate([0,0,min_thick + 1]) cylinder(r=bolthead_d/2, h=height);
  //       }
  //     }
  //   }
  // }
// }
    x_dist = 64;
    y_dist = 79;
    color("red")
    for(x=[-1 , 1], y=[-1 , 1]) {
        translate([x * x_dist/2, y * y_dist/2, -1]) {
            cylinder(d=bolt_d, h=height);

            translate([0, 0, min_thick - 1]) cylinder(d=bolthead_d, h=height);
        }
    }


  // Chamfer
  // translate([0,0,min_thick+s/2]) rotate_extrude() translate([pipe_d/2+min_thick+s/2,0,0]) circle(r=s/2);
  translate([0,0,min_thick+s/2]) rotate_extrude() translate([pipe_d/2+min_thick+s/2,0,0]) sq_circle();

  // Setscrews
 if(num_setscrews > 0)
 {
   for(x=[0:360/num_setscrews:num_setscrews*(360/num_setscrews)])
   {
     translate([0,0,min_thick+s/2]) rotate([0,0,(360/(num_setscrews*2))+x]) rotate([0,90,0])
     {
       cylinder(r=setscrew_d/2, h=flange_d/2);
       translate([0,0,pipe_d/2 + min_thick]) cylinder(r=setscrewhead_d/2, h=flange_d/2);
     }
   }
 }


}

module sq_circle() {
  circle(r=s/2);
  color("blue")
  translate([s/4, 0])
  square(size=[s/2, s], center=true);
}

// sq_circle();