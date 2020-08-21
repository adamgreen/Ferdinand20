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
use <Sparkfun_9032.scad>


// Parameters
pcbDiameter = 54;
pcbClearance = 3;

bodyHeight = 27;
bodyLowerHeight = 9;
bodyEdgeRadius = 2;
bodySlantStart = 9;

buttonAngle = 45;
buttonEdgeRadius = 0.5;
buttonThickness = 3;
buttonHeight = 15;
buttonZPos = 16;
buttonSlant = 1;



union() {
    scale([0.9, 1.1, 1.0]) {
        rotate([0, 0, 180]) Button();
        MainBody();
    }
    translate([0, 0, 6]) PcbWithJoystick();
}



// Calculated values
topDiameter = pcbDiameter + 2 * pcbClearance;
topRadius = topDiameter / 2;
buttonEdgeDistance = topRadius+buttonThickness-buttonEdgeRadius;
buttonEdgeAngle = asin(buttonEdgeRadius/buttonEdgeDistance);
// Increase the circumference by buttonSlant. circumference = radians * radius.
// insideRadians * insideRadius = outsideRadians * outsideRadius + 2 * buttonSlant.
buttonInsideAngle = rad2deg((deg2rad(buttonAngle) * buttonEdgeDistance + 2 * buttonSlant) / topRadius);
bodySlantLength = 2 * sqrt(pow(topRadius, 2) - pow(bodySlantStart, 2));

function deg2rad(a) = a * PI / 180;
function rad2deg(a) = a * 180 / PI;



// Draws the main body oval that houses the electronics.
module MainBody() {
    // Uses a hull stretched over rings to provide a rounded edge oval.
    // Includes a round bar behind the joystick to start a slant down to the bottom of
    // the body to give more room for the palm of the hand.
    hull() {
        // Rounded edge for the bottom oval.
        RoundedBodyEdge(bodyEdgeRadius);
        // UNDONE: I don't think this is needed to make room for the PCB.
        // Rounded edge for oval that designates the bottom of the slant.
        *RoundedBodyEdge(bodyLowerHeight - bodyEdgeRadius);
        // Rounded edge for the top oval with the front removed to make room for the slant.
        difference() {
            RoundedBodyEdge(bodyHeight - bodyEdgeRadius);
            translate([0, topDiameter/2 + bodySlantStart, bodyHeight - bodyEdgeRadius]) 
                cube([topDiameter, topDiameter, 2*bodyEdgeRadius], center=true);
        }
        // The rounded edge that goes across the top of the body to start the slant.
        // Built it 2 pieces so that a nice union of the two top rings occurs.
        intersection() {
            TopOfBodySlant(bodySlantLength);
            RoundedBodyEdge(bodyHeight - bodyEdgeRadius);
        }
        TopOfBodySlant(bodySlantLength-2*bodyEdgeRadius);
    }
}

// Draws a rounded edge oval at the specified height.
module RoundedBodyEdge(height) {
    translate([0, 0, height])
        rotate_extrude(angle=360) 
            translate([topRadius-bodyEdgeRadius, 0, 0]) 
                circle(r=bodyEdgeRadius);
}

// Draws the rounded edge that goes across the top of the body to start the slant.
// Caller provides the desired width of this top edge so that it can be built in 2
// pieces.
module TopOfBodySlant(width) {
    translate([0, bodySlantStart, bodyHeight - bodyEdgeRadius]) 
        rotate([0, 90, 0]) 
            cylinder(r=bodyEdgeRadius, h=width, center=true);
}

// Draws the deadman switch button.
module Button() {
    difference() {
        hull() {
            ButtonCorners();
            ButtonEdges();
            ButtonInside();
        }
        // Finalize by carving out the inner concave surface of the button that is 
        // left after the hull operation.
        translate([0, 0, buttonZPos]) cylinder(r=topRadius, h=buttonHeight+2*buttonSlant+0.1, center=true);
    }
}

// Finds the corner of the rounded edges by intersecting full horizontal/vertical edges.
module ButtonCorners() {
    intersection() {
        for(i =[-1:2:1])
            HorizontalButtonEdge(buttonAngle, i);
        for (i = [-1:2:1])
            VerticalButtonEdge(0, i);
    }
}
// Draws the button edges, but not the whole length. 
// Instead go a bit shorter to meetup with corners.
module ButtonEdges() {
    for(i =[-1:2:1])
        HorizontalButtonEdge(buttonAngle-2*buttonEdgeRadius, i);
    for (i = [-1:2:1])
        VerticalButtonEdge(buttonEdgeRadius, i);
}

// Draw the inside curved surface of the button.
module ButtonInside() {
    rotate([0, 0, 90-buttonInsideAngle/2]) translate([0, 0, buttonZPos]) 
        rotate_extrude(angle=buttonInsideAngle) 
            translate([topRadius, 0, 0]) square([0.1, buttonHeight+2*buttonSlant], center=true);
}

// Draws the horizontal button edge. 
// Caller specifies degrees it should span and whether it should go on top or bottom.
module HorizontalButtonEdge(a, topOrBottom) {
    rotate([0, 0, (90-a/2)])
        translate([0, 0, buttonZPos+topOrBottom*(buttonHeight/2-buttonEdgeRadius)]) 
            rotate_extrude(angle=a) 
            translate([buttonEdgeDistance, 0, 0]) 
                circle(r=buttonEdgeRadius);
}

// Draws the vertical button edge. 
// Caller specifies clearance on ends and whether it should draw left or right edge.
module VerticalButtonEdge(clearance, leftOrRight) {
    translate([leftOrRight*sin(buttonAngle/2-buttonEdgeAngle)*buttonEdgeDistance, 
               cos(buttonAngle/2-buttonEdgeAngle)*buttonEdgeDistance, 
               buttonZPos-buttonHeight/2+clearance]) 
        cylinder(r=buttonEdgeRadius, h=buttonHeight-2*clearance);

}



// Draws the PCB with thumb joystick. Centered on X/Y origin, bottom of PCB at Z=0.
module PcbWithJoystick() {
    // Center the PCB.
    translate([-33/2, -43/2, 0]) {
        union() {
            // Place joystick on PCB as found in PCB layout.
            translate([9.46+16/2, 7+16/2, 1.6])
                joystick();
            difference() {
                // PCB, Place lower left corner at origin to make it easier to match KiCAD part placements.
                translate([33/2, 43/2, 0]) 
                    roundedRect(cornerRadius=7.62, width=33, length=43, height=1.6);
                // Remove 4 x 3.2mm corner mounting holes.
                translate([71.882-68.834, 117.856-111.76, -0.1]) 
                    cylinder(r=3.2/2, h=1.6+0.2);
                translate([98.552-68.834, 117.856-111.76, -0.1]) 
                    cylinder(r=3.2/2, h=1.6+0.2);
                translate([71.882-68.834, 148.844-111.76, -0.1]) 
                    cylinder(r=3.2/2, h=1.6+0.2);
                translate([98.552-68.834, 148.844-111.76, -0.1]) 
                    cylinder(r=3.2/2, h=1.6+0.2);
            }
            // Right angle deadman switch.
            translate([14+7.0/2, 10.1, 0]) rotate([0, 180, 0]) rightAngleSwitch(); 
        }
    }
}

module roundedRect(cornerRadius, width, length, height) {
    hull() {
        translate([width/2-cornerRadius, length/2-cornerRadius, height/2]) 
            cylinder(r=cornerRadius, h=height, center=true);
        translate([-(width/2-cornerRadius), -(length/2-cornerRadius), height/2]) 
            cylinder(r=cornerRadius, h=height, center=true);
        translate([-(width/2-cornerRadius), length/2-cornerRadius, height/2]) 
            cylinder(r=cornerRadius, h=height, center=true);
        translate([width/2-cornerRadius, -(length/2-cornerRadius), height/2]) 
            cylinder(r=cornerRadius, h=height, center=true);
    }
}

module rightAngleSwitch() {
    translate([-6.0/2, 0, 0]) union() {
        cube([6.0, 3.7, 7.0]);
        translate([6.0/2, 0, 4.0]) 
            rotate([90, 0, 0]) 
                cylinder(d=3.4, h=1.3);
    }
}



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
