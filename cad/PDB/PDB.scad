/*  Copyright (C) 2020  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Power Distribution Board Panel
// Contains LCD, Power and Manual switches.
include <Common.scad>

//difference() {
    rotate([180, 0, 0])
        PDB();
//    cube([120, 120, (sideDepth-7.0)*2], center=true);
//    translate([0, 0, sideDepth+120/2+0.01])
//        cube([120, 120, 120], center=true);
//}