/*
WS Pole Mount - Paint Roller End of Pole

Created March 22, 2024
Modified March 25, 2024
*/

include <tsmthread4.scad>

/*
  DMAJ=19.85 mm is just right
  PITCH=5 mm is just right
  A=29, H1=0.5/2, H2=0.5/2 from examples
*/

dmaj = 19.85 + $OD_COMP;
pitch = 5;
a = 29;
h1 = 0.5/2;
h2 = 0.5/2;
l = 35;

adapt_d = 49.9; // 49.8 loose
adapt_h = 30;

module test_fit() {
    difference() {
        cylinder(d=adapt_d, h=adapt_h/2);
        comp_thread(DMAJ=dmaj, L=l, PITCH=pitch, A=a, H1=h1, H2=h2, in=true);
    }
}

module station_foot_paint_roller_adapter() {
    // station foot print = 50.6 mm diameter, 30 mm high
    difference() {
        cylinder(d=adapt_d, h=adapt_h);
        comp_thread(DMAJ=dmaj, L=l, PITCH=pitch, A=a, H1=h1, H2=h2, in=true);
    }
}

station_foot_paint_roller_adapter();