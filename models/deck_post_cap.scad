/*
Deck Post Cap
For mounting Long Range Weather Station
to 4 x 4 inch deck pole (standard US size)
Remixed / New Mount for https://www.thingiverse.com/thing:5990046

Created October 16, 2023
Modified November 23, 2023

Notes 12/13/23:
    - rot_cap_theta set to my deck "South"
    - used compass app on my phone to find South direction
    - easier to calibrate
*/

bott_cap_lw = 102.3; // length/width of deck post, bottom l/w of cap
bott_cap_h = 30; // inside cap height

top_t = 4; // cap top thickness
wall_t = 4; // wall thickness
cap_lw = bott_cap_lw + wall_t * 4; // top cap length/width

hole_d = 3.15; // mount hole diameter
x_dist = 64; // x hole spacing
y_dist = 79; // y hole spacing

rot_cap_lw = 83; // rotating part to set correct north/south angle
rot_cap_theta = -39; // rot cap rotation -- was 30
x_off = (rot_cap_lw - x_dist)/2;
y_off = (rot_cap_lw - y_dist)/2;
rradius = 1; // common rectangle rounded radius

bracket_t = 6; // bracket thickness
arm_cyl_d = 15; // bracket arm cylinder diameter
center_cyl_d = 35; // bracket center cylinder diameter

$fs = 0.2; // precision of curves

module rounded_rect(size=[1, 1], radius=0.5, center=false) {
	// If single value, convert to [x, y] vector
	size = (size[0] == undef) ? [size, size] : size;

	translate = (center == false) ?
		[radius, radius] :
		[radius - size[0]/2, radius - size[1]/2];

	translate(v = translate)
	minkowski() {
		square(size = [
            size[0] - radius * 2,
			size[1] - radius * 2,
		]);
		circle(r = radius);
	}
}

module extruded_rounded_rect(size=[1, 1, 1], radius=0.5, center=false) {
    translate([0, 0, -size.z/2])
    linear_extrude(size.z)
        rounded_rect(size=[size.x, size.y], radius=radius, center=center);
}

module cap_bottom() {
    translate([0, 0, bott_cap_h/2])
        difference() {
            extruded_rounded_rect(size=[bott_cap_lw + wall_t * 2, bott_cap_lw + wall_t * 2, bott_cap_h], radius=rradius, center=true);
            extruded_rounded_rect(size=[bott_cap_lw, bott_cap_lw, bott_cap_h], radius=rradius, center=true);
        }
}

module cap_top() {
    translate([0, 0, top_t/2 + bott_cap_h])
        extruded_rounded_rect(size=[cap_lw, cap_lw, top_t], radius=rradius, center=true);
}

module rot_cap_cube() {
    translate([0, 0, top_t/2])
        cube(size=[rot_cap_lw, rot_cap_lw, top_t], center=true);
}

module draw_rot_cap(rot) {
    translate([0, 0, bott_cap_h])
    rotate(rot) {
        difference() {
            rot_cap_cube();
            
            // mount holes
            color("red")
            for(x=[-1 , 1], y=[-1 , 1]) {
                translate([x * x_dist/2, y * y_dist/2, 0])
                    cylinder(d=hole_d, h=top_t);
            }
        }
    }
}

module draw_cap(rot_mount) {
    rot = rot_mount ? rot_cap_theta : 0;

    rotate([180, 0, 0]) { // flip over x axis for better print orientation
        cap_bottom();
        
        difference() {
            cap_top();

            // cut out rotating part -- MAY BE OBSOLETE b/c of bracket
            // rotated mount holes may not fit inside of the cap!
            translate([0, 0,  bott_cap_h])
            rotate(rot)
               rot_cap_cube();
        }
        
        color("orange")
        draw_rot_cap(rot);
    }
}

module bracket_arms(hole_rot) {
    difference() {
        union() {
            // arm 1
            hull() {
                translate([x_dist/2, y_dist/2, 0])
                    cylinder(d=arm_cyl_d, h=bracket_t);
                translate([-x_dist/2, -y_dist/2, 0])
                    cylinder(d=arm_cyl_d, h=bracket_t);
            }
            // arm 2
            hull() {
                translate([-x_dist/2, y_dist/2, 0])
                    cylinder(d=arm_cyl_d, h=bracket_t);
                translate([x_dist/2, -y_dist/2, 0])
                    cylinder(d=arm_cyl_d, h=bracket_t);
            }
            // hub
            cylinder(d=center_cyl_d, h=bracket_t);
        }

        // connecting holes in center
        color("green")
        rotate(hole_rot)
        for(x=[-1, 1], y=[-1, 1]) {
            translate([x * 0.4 * arm_cyl_d, y * 0.4 * arm_cyl_d])
                cylinder(d=hole_d, h=bracket_t);
        }

        // mounting holes
        for(x=[-1, 1], y=[-1, 1]) {
            translate([x * x_dist/2, y * y_dist/2, 0])
                cylinder(d=hole_d, h=bracket_t);
        }
    }
}

module draw_brackets() {
    // bracket_arms(hole_rot=0);
    translate([x_dist * 1.5, 0, 0])
    // rotate(rot_cap_theta)
        bracket_arms(hole_rot=-rot_cap_theta);
}

draw_cap(rot_mount=false);
// draw_brackets();