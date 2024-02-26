/*
Anemometer Pipe Cap for Hall Sensor
Modeled for https://www.thingiverse.com/thing:5997239

Created November 28, 2023
Modified February 23, 2024

1/11/24: Increased OD of lip rim for easier removal
2/23/24:
  - Increased slot gap thickness 1.2 -> 4 - can clamp on wires now
  - Changed theta from 330 to 315
*/

pipe_od = 20; // stay inside pipe od
cap_od = 16.3; // ID of amazon pipe

cap_bott_h = 8;
cap_top_d = 3; // diameter of half circle to rotate_extrude on top lip
theta = 315;

// slot for hall sensor
slot_w = 4;  // notch is half this distance from ID of pipe
slot_h = cap_bott_h;

$fs = 0.1;

/*
rotate half circle around top lip, 330 degrees
cut small slot for hall sensor
*/

module half_circle() {
    difference() {
        circle(d=cap_top_d);
        translate([0, -cap_top_d/2])
            square(size=[cap_top_d, cap_top_d], center=true);
    }
}

module plug() {
    difference() {
        cylinder(d=cap_od, h=cap_bott_h);

        color("red")
        rotate(theta)
        translate([0, 0, cap_bott_h/2])
        rotate_extrude(angle=360 - theta)
        translate([cap_od/2, 0])
        square(size=[slot_w, slot_h], center=true);
    }
}

module plug_top() {
    rotate_extrude(angle=theta)
    translate([cap_od/2.1, 0, 0])
    half_circle();
}


module draw_plug() {
    plug();
    translate([0, 0, cap_bott_h])
        plug_top();
}

draw_plug();