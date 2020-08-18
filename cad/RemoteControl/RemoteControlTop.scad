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
            translate([15.5, 15.5, -3.2])
                // Center Sparkfun joystick drawing.
                joystick();
            // PCB
            cube([33, 43, 1.6]);
        }
    }
}

// Draws a rough version of the Sparkfun joystick.
module joystick() {
    // 9032.stl contains a non-manifold version created by Sparkfun.
    //translate([-26.6/2, -26.6/2, 0]) import("9032.stl");
    union() {
        hull() {
            difference() {
                translate([0, 0, 18.9]) sphere(d=32);
                cube([40, 40, 64], center=true);
            }
            translate([0, 0, 30]) cylinder(d=20.5, h=0.1);
        }
        translate([0, 0, 24]) cylinder(d=11.3, h=6);
        difference() {
            translate([0, 0, 10.8]) sphere(d=29);
            cube([40, 40, 32], center=true);
        }
        translate([0, 0, 10.5]) cube([15.5, 15.9, 12], center=true);
    }
}



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
