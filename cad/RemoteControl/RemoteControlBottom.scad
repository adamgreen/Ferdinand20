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

// Dimensions of the 2xAAA battery pack that goes inside the handle.
batteryPackWidth = 25.9;
batteryPackThickness = 15.64;
batteryPackHeight = 63.2;

// Width and thickness clearance around battery pack.
sideClearance = 5;
// Height clearance around battery pack.
heightClearance = 5;

// Diameter of rings around handle to act as grip between fingers.
fingerRingDiameter = 3;

// Distance from top for each finger ring.
fingerRingHeights = [21.5, 21.5+17];

// Bottom flange dimensions.
flangeHeight = 5;
flangeWidth = 5;
flangePoints = [[0, 0], [0, flangeHeight], [flangeWidth, 0]];

// How angled should the handle be with respect to the top of the remote.
rampAngle = 30;


// Calculated values.
handleDiameter = sqrt(pow(batteryPackWidth, 2) + pow(batteryPackThickness, 2));
handleRadius = handleDiameter/2;
totalDiameter = handleDiameter+sideClearance*2;
totalRadius = totalDiameter / 2;
rampWidth = totalDiameter+0.02;
rampHeight = sin(rampAngle) * rampWidth;
rampPoints = [[0,rampHeight], [rampWidth, rampHeight], [rampWidth, 0]];
handleHeight = batteryPackHeight + rampHeight + 2 * heightClearance;


// Draw the handle and use scale to make it into an oval.
scale([1.0, 0.75, 1.0]) union() {
    // The main cylinder with finger grip rings and slanted top.
    difference() {
        union() {
            // The main cylinder of the handle.
            cylinder(d=totalDiameter, h=handleHeight);
            // The rings that form the grip and separate the fingers holding the remote.
            for(i=[0:len(fingerRingHeights)-1])
                DrawFingerRing(handleHeight-fingerRingHeights[i]);
        }
        // Mask off the top to slant it.
        TopMask();
    }
    // The bottom flare.
    DrawFlange();
}
translate([0, 0, heightClearance])BatteryPack();



// Draws the ramp at the top of the handle to apply an angle between it and the main
// remote body.
module TopMask() {
    translate([-rampWidth/2, rampWidth/2, handleHeight-rampHeight+0.01]) 
        rotate([90, 0, 0]) 
            linear_extrude(height=rampWidth) 
                polygon(points=rampPoints);
}

// Draw a ring around the handle to fall between fingers.
module DrawFingerRing(height) {
    translate([0, 0, height]) 
        rotate_extrude(angle=360) 
            translate([handleRadius+sideClearance, 0, 0]) 
                circle(d=fingerRingDiameter);
}

// Draw the flange for the bottom of the handle.
module DrawFlange() {
    rotate_extrude(angle=360) 
        translate([totalRadius-0.01, 0, 0]) 
            polygon(points=flangePoints);
}

// Draw the rough shape of the 2x AAA battery pack.
module BatteryPack() {
    translate([0, 0, batteryPackHeight/2]) 
        cube([batteryPackWidth, batteryPackThickness, batteryPackHeight], center=true);
}



$fs = 1;
$fa = 3;
