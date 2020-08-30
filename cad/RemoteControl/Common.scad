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
// Parameters and modules used to produce multiple parts of the remote control enclosure.


// Parameters
pcbDiameter = 54;
pcbClearance = 3;

bodyWidthScale = 0.9;
bodyLengthScale = 1.1;

buttonAngle = 30;
buttonEdgeRadius = 0.5;
buttonThickness = 3;
buttonHeight = 11;
buttonZPos = 18;
buttonSlant = 1;
buttonEnclosureDepth = 9.5;
buttonBumperWidth = 1.0;
buttonBumperOffset = 0.0;
buttonBumperThickness = 0.5;
buttonArmRadius = 2.0;
buttonArmHeight = 8.4;
buttonArmLength = 10.9;
buttonArmXOffset = -1.0;
buttonArmYOffset = 1.5;
buttonRidgeRadius = 1.0;
buttonRidgeStart = 1.0;


// Calculated values
topDiameter = pcbDiameter + 2 * pcbClearance;
topRadius = topDiameter / 2;

buttonEdgeDistance = topRadius+buttonThickness-buttonEdgeRadius;
buttonEdgeAngle = asin(buttonEdgeRadius/buttonEdgeDistance);
// Increase the circumference by buttonSlant. circumference = radians * radius.
// insideRadians * insideRadius = outsideRadians * outsideRadius + 2 * buttonSlant.
buttonInsideAngle = rad2deg((deg2rad(buttonAngle) * buttonEdgeDistance + 2 * buttonSlant) / topRadius);

buttonInsideWidth = deg2rad(buttonInsideAngle)*topRadius-2*buttonEdgeRadius;
buttonInsideHeight = buttonHeight+2*buttonSlant;
buttonInsideRadiusDiff = cos(buttonInsideAngle/2)*topRadius;
buttonInsideRadius = buttonInsideRadiusDiff-buttonEnclosureDepth/2;

buttonBumperLocation = (topRadius-buttonInsideRadiusDiff)+buttonEnclosureDepth-buttonBumperOffset;

function deg2rad(a) = a * PI / 180;
function rad2deg(a) = a * 180 / PI;



// Draws the deadman switch button.
module Button() {
    hull() {
        ButtonCorners();
        ButtonEdges();
        ButtonInside();
        ButtonThroughEnclosure();
    }
    ButtonBumper();
    translate([buttonArmXOffset, 0, 0]) ButtonArm();
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
    rotate([0, 0, 90-buttonInsideAngle/2]) translate([0, 0, 0]) 
        rotate_extrude(angle=buttonInsideAngle) 
            translate([topRadius, 0, 0]) square([0.1, buttonHeight+2*buttonSlant], center=true);
}

// Draw the piece of the button which passes through the enclosure.
module ButtonThroughEnclosure() {
    translate([0, buttonInsideRadius, 0])
        cube([buttonInsideWidth, buttonEnclosureDepth, buttonInsideHeight], center=true);
}

// Draw the rounded bumper around the inside of the button to hold it in.
module ButtonBumper() {
    translate([0, topRadius-buttonBumperLocation-buttonBumperThickness/2, -buttonBumperWidth/2])
        rotate([90, 0, 0]) 
            translate([0, 0, -buttonBumperThickness/2]) 
                RoundedRect(2.0, buttonInsideWidth+buttonBumperWidth*2, buttonInsideHeight+buttonBumperWidth, buttonBumperThickness);
}

// Draw the arm that goes from the deadman button to the right angled switch.
module ButtonArm() {
    rotate([0, 180, 0]) {
        translate([0, buttonInsideRadius-buttonEnclosureDepth/2+buttonArmRadius+buttonArmYOffset, buttonInsideHeight/2-0.001]) {
            intersection() {
                cylinder(r=buttonArmRadius, h=buttonArmHeight);
                translate([0, buttonArmRadius, buttonArmHeight-buttonArmRadius]) 
                    rotate([90, 0, 0]) cylinder(r=buttonArmRadius, h=buttonArmLength);
            }
            cylinder(r=buttonArmRadius, h=buttonArmHeight-buttonArmRadius);
            translate([0, 0, buttonArmHeight-buttonArmRadius]) 
                rotate([90, 0, 0]) cylinder(r=buttonArmRadius, h=buttonArmLength);
        }
    }
}

// This is the solid that should be removed from the bottom of the button guide to allow the arm to slide.
module ButtonArmSlot() {
    translate([-buttonArmRadius+buttonArmXOffset, buttonInsideRadius-buttonEnclosureDepth/2, -buttonInsideHeight/2-buttonArmHeight+0.001])
        cube([buttonArmRadius*2, buttonArmYOffset+buttonArmRadius, buttonArmHeight]);
}

// Draw a ridge on the top of the button to fit into a groove of the remote control.
// This stabilizes the button from side to side rotation.
module ButtonRidge() {
    translate([0, buttonInsideRadius-buttonEnclosureDepth/2, buttonInsideHeight/2]) 
        rotate([-90, 0, 0]) 
            cylinder(r=buttonRidgeRadius, h=topRadius-buttonInsideRadius+buttonEnclosureDepth/2-buttonRidgeStart);
}

// Draws the horizontal button edge. 
// Caller specifies degrees it should span and whether it should go on top or bottom.
module HorizontalButtonEdge(a, topOrBottom) {
    rotate([0, 0, (90-a/2)])
        translate([0, 0, topOrBottom*(buttonHeight/2-buttonEdgeRadius)]) 
            rotate_extrude(angle=a) 
            translate([buttonEdgeDistance, 0, 0]) 
                circle(r=buttonEdgeRadius);
}

// Draws the vertical button edge. 
// Caller specifies clearance on ends and whether it should draw left or right edge.
module VerticalButtonEdge(clearance, leftOrRight) {
    translate([leftOrRight*sin(buttonAngle/2-buttonEdgeAngle)*buttonEdgeDistance, 
               cos(buttonAngle/2-buttonEdgeAngle)*buttonEdgeDistance, 
               -buttonHeight/2+clearance]) 
        cylinder(r=buttonEdgeRadius, h=buttonHeight-2*clearance);

}

// Draws a rounded rect with the requested dimensions.
module RoundedRect(cornerRadius, width, length, height) {
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
