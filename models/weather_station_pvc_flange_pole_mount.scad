/*
Weather Station PVC Flange Thread Mount

Created March 21, 2024
Modified March 24, 2024

Uses tsmthread 0.4 library
https://www.thingiverse.com/thing:3391213
and http://burningsmell.org/3d/lib/tsmthread/

See https://www.ultimatewasher.com/dimensionschart.htm
for NPT Specifications

See https://www.thomsonlinear.com/en/support/tips/what-is-thread-pitch
for thread pitch
*/

include <tsmthread4.scad>

// Flange pattern as described by this URL:
// https://www.helixlinear.com/Product/PowerAC-38-2-RA-wBronze-Nut/
//
//      /---\   <--------
//      |   |           |
// ---> |>>>|           |
// |    |   |           |
// |    |   \---\ <--   |
// |    |       |   |   |
// F*2  |       |   C   H
// |    |       |   |   |
// |    |   /---/ <--   |
// |    |   |           |
// ---> |>>>|           |
//      |   |           |
//      \---/   <--------
//      ^   ^   ^
//      |-G-|-B-|
//
//      B is height of stem
//      C is diameter of stem
//      F is the radius of the screw pattern
//      F2 is the diameter of the screw holes.
//      G is the thickness of the base
//      H is width of base.
//
//      holes is the pattern of holes to drill.  default is 4 holes, 1 every 90 degrees.
//      when open is true, the holes are sots extending to the edge.
//      off is how much to bevel.

/*
  DMAJ = 1.917" little tight, 1.918 just right
  PITCH = 0.147" from calipers, just right
  A=29, H1=0.5/2, H2=0.5/2 from examples
*/

dmaj = 1.918;
pitch = 0.147;
a = 29;
h1 = 0.5/2;
h2 = 0.5/2;

module ws_pole_flange() {
    imperial() { // units are in INCHES not mm
        difference() {
            flange(B=0.75, G=0.25, F=3, F2=3/25.4, C=2.65+$OD_COMP, H=3.5+$OD_COMP, holes=[0, 90, 180, 270]);

            comp_thread(DMAJ=dmaj + $OD_COMP, L=pitch * 10, PITCH=pitch, A=a, H1=h1, H2=h2, in=true);
        }
    }
}

module test_fit_flange() {
    imperial() { // units are in INCHES not mm
        difference() {
            flange(B=0, G=0.5, F=3, F2=0.16, C=2.65+$OD_COMP, H=2.15+$OD_COMP, holes=[]);

            comp_thread(DMAJ=dmaj + $OD_COMP, L=pitch * 10, PITCH=pitch, A=a, H1=h1, H2=h2, in=true);
        }
    }
}

// test_fit_flange();
ws_pole_flange();