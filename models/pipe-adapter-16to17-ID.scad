/*
Pipe adapter 16mm to 17mm ID

Created November 24, 2023
Modified November 26, 2023
*/

adapt_t = 1.5;

ws_id = 17.45; // weather station ID
ws_od = ws_id + 2 * adapt_t;
ws_h = 21;

ring_h = 2;

pipe_od = 16.4; // amazon pipe ID
pipe_id = pipe_od - 2 * adapt_t;
pipe_h = 15 + ring_h;


$fs = 0.1;

// weather station connection
difference() {
    cylinder(d=ws_od, h=ws_h);
    cylinder(d=ws_id, h=ws_h);
}

translate([0, 0, ws_h]) {
    // amazon pipe connection
    difference() {
        cylinder(d=pipe_od, h=pipe_h);
        cylinder(d=pipe_id, h=pipe_h);
    }

    // ring
    difference() {
        cylinder(d=ws_od, h=2*adapt_t);
        cylinder(d=pipe_id, h=2*adapt_t);
    }
}