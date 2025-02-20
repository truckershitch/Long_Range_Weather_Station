/*
French Weather Station 125mm Pipe Join Fix
Remixed for https://www.thingiverse.com/thing:5990046

Saving filament and time by coupling old print to newer print

Created March 26, 2024
Modified March 26, 2024
*/

skinny_od = 118.2 + 0.6; // same as funnel fitting OD, just tighter fit
skinny_id = skinny_od - 4; // can be thin here
wide_od = 125;
wide_id = 118.2 + 0.45; // should fit outside funnel fitting OD

wide_h = 3;
taper_h = 3;
skinny_h = 8;

$fs = 0.2;

module pipe_fix() {
    // skinny end
    translate([0, 0, 0])
    difference() {
        cylinder(d=skinny_od, h=skinny_h);
        cylinder(d=skinny_id, h=skinny_h);
    }

    // taper
    translate([0, 0, skinny_h])
    difference() {
        cylinder(d1=skinny_od, d2=wide_od, h=taper_h);
        cylinder(d1=skinny_id, d2=wide_id, h=taper_h);
    }

    // wide end
    translate([0, 0, skinny_h + taper_h])
    difference() {
        cylinder(d=wide_od, h=wide_h);
        cylinder(d=wide_id, h=wide_h);
    }

    // taper
    translate([0, 0, skinny_h + taper_h + wide_h])
    difference() {
        cylinder(d1=wide_od, d2=skinny_od, h=taper_h);
        cylinder(d1=wide_id, d2=skinny_id, h=taper_h);
    }

    // skinny end
    translate([0, 0, skinny_h + taper_h + wide_h + taper_h])
    difference() {
        cylinder(d=skinny_od, h=skinny_h);
        cylinder(d=skinny_id, h=skinny_h);
    }
}

test_h = 2;
module wide_test_ring() {
    // wide
    difference() {
        cylinder(d=wide_od, h=test_h);
        cylinder(d=wide_id, h=test_h);
    }
}

module skinny_test_ring() {
    // skinny
    difference() {
        cylinder(d=skinny_od, h=test_h * 2);
        cylinder(d=skinny_id, h=test_h * 2);
    }
}

// wide_test_ring();
// skinny_test_ring();

pipe_fix();
