/*
Connecting 125mm Pipe Hack - Shorty
for Weather Station
Remixed for https://www.thingiverse.com/thing:5990046
Metric PVC pipes are hard to find in the USA

Created October 13, 2023
Modified October 31, 2024
*/

// see https://epco-plastics.com/news/post/inch-and-metric-pipe-dimensions

pvc_125_id = 118.8;
pvc_125_od = 125; // OD of metric pipe
nozzle_size = 0.6;

ht = 70;

id = pvc_125_id + 0.15;
od = pvc_125_id + floor((pvc_125_od - pvc_125_id) / nozzle_size) * nozzle_size; // make wall size nultiple of nozzle size (0.6)
echo("OD", od);

hole_d = 2;
hole_z_off = 5 + 11;

base_to_pipe_ctr = 26;
base_pipe_d = 20.683 + 2;
pipe_cut_cyl_h = od + 5;

skinny_od = 118.2 + 0.6; // same as funnel fitting OD, just tighter fit
skinny_id = skinny_od - 4; // can be thin here

taper_h = 4;
skinny_h = 15;

$fs = 0.2;

difference() {
    cylinder(d=od, h=ht);
    cylinder(d=id, h=ht);

    color("purple")
    translate([0, -pipe_cut_cyl_h/2, base_to_pipe_ctr])
    rotate([270, 0, 0])
    cylinder(d=base_pipe_d, h=pipe_cut_cyl_h);
}

translate([0, 0, ht])
difference() {
    cylinder(d1=od, d2=skinny_od, h=taper_h);

    cylinder(d1=id, d2=skinny_id, h=taper_h);
}

translate([0, 0, ht + taper_h])
    difference() {
        cylinder(d=skinny_od, h=skinny_h);

        cylinder(d=skinny_id, h=skinny_h);
    }