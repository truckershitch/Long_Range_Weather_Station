/*
3/4" PVC to Weather Station Mount Adapter

Created April 1, 2024
Modified April 1, 2024
*/

adapt_od = 50;
adapt_id = 1.05 * 25.4 + 0.5;

adapt_h = 30;
top_h = 2;

difference() {
    cylinder(d=adapt_od, h=adapt_h);
    translate([0, 0, -top_h])
        cylinder(d=adapt_id, h=adapt_h - top_h);
}   