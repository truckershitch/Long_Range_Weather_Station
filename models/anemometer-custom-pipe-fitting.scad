/*
Anemometer custom pipe fitting
Remixed from https://www.thingiverse.com/thing:5997239

Created November 27, 2023
Modified February 25, 2024

2/14/2024 - reduced cone_h 3.5 -> 2.7
2/25/2024 - increased bott_pipe_h 25 -> 30
*/

$fs = 0.1;

wall_t = 1.5;

pipe_id = 20.55;
pipe_od = pipe_id + wall_t * 2;
bott_pipe_h = 30;

cone1_od1 = pipe_od;
cone1_od2 = 18.2;
cone_h = 2.7;

top_od = cone1_od2;
top_id = 13.25;
top_h = 6.5;

ring_h = 3.5;
ring_d = top_id;
scr_hole_d = 10;

err = 0.01; // OpenSCAD visual rendering error
err_trans = [0, 0, -err]; // common translation vector for viewing purposes

// bottom
difference() {
    cylinder(d=pipe_od, h=bott_pipe_h);
    translate(err_trans)
    cylinder(d=pipe_id, h=bott_pipe_h + err * 2);
}

// cone
translate([0, 0, bott_pipe_h])
difference() {
    cylinder(d1=cone1_od1, d2=cone1_od2, h=cone_h);
    translate(err_trans)
    cylinder(d1=pipe_id, d2=10, h=cone_h + err * 2);
}

// outer ring
translate([0, 0, bott_pipe_h + cone_h]) {
    difference() {
        cylinder(d=top_od, h=top_h);
        translate(err_trans)
        cylinder(d=top_id, h=top_h + err * 2);
    }
}

// top inner ring
translate([0, 0, bott_pipe_h]) {
    difference() {
        cylinder(d=ring_d, h=ring_h);
        translate(err_trans)
        cylinder(d=scr_hole_d, h=ring_h + err * 2);
    }
}

// middle inner ring
translate([0, 0, bott_pipe_h - cone_h])
    difference() {
        cylinder(d=pipe_id, h=cone_h);
        translate(err_trans)
        cylinder(d1=pipe_id, d2=scr_hole_d, h=cone_h + err * 2);
    }

