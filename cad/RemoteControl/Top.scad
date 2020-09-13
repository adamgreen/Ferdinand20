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
// Case to contain the electronics for the remote control.
include <Common.scad>


rotate([0, 180.0, 0]) {
    // Draw the top of the remote control.
    RemoteControlTop();
    // Insert the deadman button in front of controller for size check.
    %scale([bodyWidthScale, bodyLengthScale, 1.0])
        translate([0, 0, buttonZPos]) 
            rotate([0, 0, 180]) 
                Button();
    // Insert mock PCB inside of controller for size check.
    %translate([0, 0, pcbZ]) PcbWithJoystick();
}