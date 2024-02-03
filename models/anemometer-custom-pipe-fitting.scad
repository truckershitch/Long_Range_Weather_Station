/*
Anemometer custom fitting

Created November 27, 2023
Modified November 28, 2023
*/

$fs = 0.1;

wall_t = 1.5;

pipe_id = 20.55;
pipe_od = pipe_id + wall_t * 2;
pipe_h = 25;

cone1_od1 = pipe_od;
cone1_od2 = 18.2;
cone_h = 3.5;

top_od = cone1_od2;
top_id = 13.25;
top_h = 6.5;

ring_h = 3.5;
ring_d = top_id;
scr_hole_d = 10;


difference() {
    cylinder(d=pipe_od, h=pipe_h);
    cylinder(d=pipe_id, h=pipe_h);
}

translate([0, 0, pipe_h])
difference() {
    cylinder(d1=cone1_od1, d2=cone1_od2, h=cone_h);
    cylinder(d1=pipe_id, d2=10, h=cone_h);
}

translate([0, 0, pipe_h + cone_h]) {
    difference() {
        cylinder(d=top_od, h=top_h);
        cylinder(d=top_id, h=top_h);
    }
}

translate([0, 0, pipe_h]) {
    difference() {
        cylinder(d=ring_d, h=ring_h);
        cylinder(d=scr_hole_d, h=ring_h);
    }
}

translate([0, 0, pipe_h - cone_h])
    difference() {
        cylinder(d=pipe_id, h=cone_h);
        cylinder(d1=pipe_id, d2=scr_hole_d, h=cone_h);
    }

