/*
Long Range Weather Station
Cover with Extended Legs

Created September 9, 2024
Modified September 27, 2024

2024-09-27 Changes
leg broke during storm.  Increasing diameter by 3mm
*/

ext_h = 2.4; // leg extension height

scr_d = 3.25; // 2 in stl model
// scr_d = 0.16 * 25.4; // #8 screw
scr_off = 3; // from stl model
scr_x_dist = 64;
scr_y_dist = 79;

leg_d = 10 + 3; // from stl model
leg_h = 21.5; // from stl model
leg_s = 7; // square leg side length

grill_w = 70;
grill_l = 85;
grill_h = 2;
lip_t = 2;

slot_l = 45;
slot_w = 3.67; // from model
slot_off = 3.327; // from model
slot_inc = 7.5;

notch_off = 20;
notch_w = 3;
notch_l = 16;

$fs = 0.2;

module grill() {
    difference() {
        cube(size=[grill_w, grill_l, grill_h]);
    
        for(y = [0 : 10]) {
            translate([(grill_w - slot_l)/2, slot_off + y * slot_inc, 0])
                cube(size=[slot_l, slot_w, grill_h]);
            translate([(grill_w - slot_l)/2, slot_off + y * slot_inc + slot_w/2, 0])
                cylinder(d=slot_w, h=grill_h);
            translate([grill_w - (grill_w - slot_l)/2, slot_off + y * slot_inc + slot_w/2, 0])
                cylinder(d=slot_w, h=grill_h);
        }
    }
}

module screw_holes() {
    for(x = [0, 1], y = [0, 1]) {
        translate([scr_off + x * scr_x_dist, scr_off + y * scr_y_dist, 0])
        cylinder(d=scr_d, h=leg_h  + ext_h);
    }
}

module leg() {
    linear_extrude(leg_h + ext_h)
    difference() {
        circle(d=leg_d);

        // circle(d=scr_d); // not really needed -- just for checking alignment
        translate([-scr_off - leg_d/2, 0])
            square(size=[leg_d, leg_d], center=true);
        translate([0, -scr_off - leg_d/2])
            square(size=[leg_d, leg_d], center=true);
    }
}

module legs() {
    r = [[0, 270], [90, 180]];

    for (x = [0, 1], y = [0, 1]) {
        translate([scr_off + x * scr_x_dist, scr_off + y * scr_y_dist, 0])
        rotate(r[x][y])
            leg();
    }
}

module notch() {
    cube(size=[notch_w, notch_l, grill_h + ext_h]);
}

module lip() {
    translate([0, 0, grill_h])
        cube(size=[grill_w, lip_t, ext_h]);
    translate([0, grill_l - lip_t, grill_h])
        cube(size=[grill_w, lip_t, ext_h]);
    translate([0, 0, grill_h])
        cube(size=[lip_t, grill_l, ext_h]);
    translate([grill_w - lip_t, 0, grill_h])
        cube(size=[lip_t, grill_l, ext_h]);
}

module draw_cover() {
    difference() {
        union() {
            grill();
            legs();
            lip();
        }
        screw_holes();

        translate([grill_w - notch_w, notch_off, 0])
        notch();

        translate([0, grill_l - notch_off - notch_l, 0])
        notch();
    }
}

draw_cover();

