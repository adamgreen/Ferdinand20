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
// Deadman button to be placed inside of remote control top to press deadman switch.
include <Common.scad>


rotate([0.0, 180.0, 0.0]) scale([bodyWidthScale, bodyLengthScale, 1.0])
    Button();



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
