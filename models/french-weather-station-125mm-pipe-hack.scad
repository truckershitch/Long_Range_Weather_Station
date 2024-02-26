/*
Connecting 125mm Pipe Hack
for Weather Station
Remixed for https://www.thingiverse.com/thing:5990046
Metric PVC pipes are hard to find in the USA

Created October 13, 2023
Modified December 13, 2023
*/

// see https://epco-plastics.com/news/post/inch-and-metric-pipe-dimensions

pvc_125_id = 118.8;
pvc_125_od = 125; // OD of metric pipe
nozzle_size = 0.6;

ht = 165;  // OG 150 + 15 = 165 12/13/2023 (then won't need length fix)

id = pvc_125_id + 0.15;
od =  pvc_125_id + floor((pvc_125_od - pvc_125_id) / nozzle_size) * nozzle_size; // make wall size nultiple of nozzle size (0.6)
echo("OD", od);

hole_d = 2;
hole_z_off = 5 + 11;

$fs = 0.2;

difference() {
    cylinder(d=od, h=ht);
    cylinder(d=id, h=ht);

    translate([0, od/2, hole_z_off])
    rotate([90, 0, 0])
    cylinder(d=hole_d, h=od);

    translate([od/2, 0, hole_z_off])
    rotate([0, 270, 0])
    cylinder(d=hole_d, h=od);
}
