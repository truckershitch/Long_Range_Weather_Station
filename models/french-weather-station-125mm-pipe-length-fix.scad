/*
French Weather Station 125mm Pipe Length Fix

Created December 13, 2023
Modified December 13, 2023

Needed to increase 150mm high by 15mm
*/

skinny_od = 118.2 + 0.6; // same as funnel fitting OD, just tighter fit
wide_id = 118.2 + 0.45; // should fit outside funnel fitting OD
skinny_id = skinny_od - 4; // can be thin here
wide_od = 125;

wide_h = 15; // effective increase in height -- needed for OG 150 pipe height
taper_h = 5;
skinny_h = 10;

$fs = 0.2;

module pipe_fix() {
    // wide end
    difference() {
        cylinder(d=wide_od, h=wide_h);
        cylinder(d=wide_id, h=wide_h);
    }

    // taper
    translate([0, 0, wide_h])
    difference() {
        cylinder(d1=wide_od, d2=skinny_od, h=taper_h);
        cylinder(d1=wide_id, d2=skinny_id, h=taper_h);
    }

    // skinny end
    translate([0, 0, wide_h + taper_h])
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
