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


// Animate in and out by setting anim to 0.0-1.0-0.0
anim = $t <= 0.5 ? $t*2 : (1.0-$t)*2;

// Draw the top of the remote control.
%RemoteControlTop();
// Insert mock PCB inside of controller for size check.
translate([0, 0, pcbZ]) PcbWithJoystick();

// Animate  the deadman button in and out of the front of the controller.
%translate([0, 0.5*anim, 0])
    scale([bodyWidthScale, bodyLengthScale, 1.0])
        translate([0, 0, buttonZPos]) 
            rotate([0, 0, 180]) 
                Button();

// Animate the handle up and down.
%translate([0, 0, -20.0*anim])
    rotate([rampAngle, 0, 0])
        translate([0, 0, -(batteryPackHeight + tan(rampAngle)*batteryPackWidth/2 + 2 * heightClearance)])
            rotate([0, 0, 90]) {
                RemoteControlHandle();
                translate([0, 0, handleHeight-rampHeight-handleIndent]) 
                    rotate([0, rampAngle, 0])
                        DrawTopOval();
                // Insert the battery pack into the middle of the pack for fitment check.
                translate([0, 0, heightClearance])BatteryPack();
        }
