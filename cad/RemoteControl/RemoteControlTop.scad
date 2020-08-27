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
include <Common.scad>


// Parameters
bodyHeight = 27;
bodyLowerHeight = 9;
bodyEdgeRadius = 2;
bodySlantStart = 9;

buttonHoleTolerance = 1.03;

pcbStandOffOD = 6.0;
pcbStandOffID = 2.5;
pcbStandOffHeight = 100.0; // Will be truncated to fit inside body.
pcbStandOffHoleHeight = 8.0;
pcbStandOffX = 26.67;
pcbStandOffY = 30.988;
pcbZ = 9.0;
pcbThickness = 1.6;

joystickHole = 25.0;


// Calculated values
bodySlantLength = 2 * sqrt(pow(topRadius, 2) - pow(bodySlantStart, 2));


difference() {
    union() {
        scale([bodyWidthScale, bodyLengthScale, 1.0]) {
            difference() {
                union() {
                    difference() {
                        MainBody();
                        // Hollow it out and leave ~2mm of shell.
                        translate([0, 0, -2.0]) scale([0.93, 0.93, 1.0]) MainBody();
                    }
                    // Create some extra support inside of the shell to support the deadman button.
                    ButtonSupport();
                }
                // Make hole for button to pass through.
                // NOTE: Not leaving any tolerance on the sides. Sand down the final button to make it
                //       fit but stay tight enough that it can't tilt off axis and jam when pressed.
                translate([0, 0.01, buttonZPos]) rotate([0, 0, 180]) {
                    scale([1.0, 1.0, buttonHoleTolerance]) {
                        Button();
                        ButtonArmSlot();
                    }
                }

            }
            %translate([0, 0, buttonZPos]) rotate([0, 0, 180]) Button();
        }
        
        // Standoffs from top to which the PCB will be mounted.
        intersection() {
            // Just keep the parts of the standoff which fall inside of the main body.
            MainBody();
            union() {
                translate([pcbStandOffX/2, pcbStandOffY/2, pcbZ+pcbThickness]) 
                    standOff(od=pcbStandOffOD, id=pcbStandOffID, h=pcbStandOffHeight, holeH=pcbStandOffHoleHeight);
                translate([-pcbStandOffX/2, -pcbStandOffY/2, pcbZ+pcbThickness])
                    standOff(od=pcbStandOffOD, id=pcbStandOffID, h=pcbStandOffHeight, holeH=pcbStandOffHoleHeight);
                translate([-pcbStandOffX/2, pcbStandOffY/2, pcbZ+pcbThickness]) 
                    standOff(od=pcbStandOffOD, id=pcbStandOffID, h=pcbStandOffHeight, holeH=pcbStandOffHoleHeight);
            }
        }
        // One of the standoffs needs to have notch cut out of it to make room for joystick rotation.
        difference() {
            intersection() {
                MainBody();
                translate([pcbStandOffX/2, -pcbStandOffY/2, pcbZ+pcbThickness])
                    standOff(od=pcbStandOffOD, id=pcbStandOffID, h=pcbStandOffHeight, holeH=pcbStandOffHoleHeight);
            }
            // Approximate the rotation of the joystick with an enlarged sphere.
            translate([(9.46+16/2)-33/2, (7+16/2)-43/2, 20.0]) sphere(d=28.0);
        }
    }
    // Hole on top for joystick.
    translate([(9.46+16/2)-33/2, (7+16/2)-43/2, bodyHeight]) 
        cylinder(d=joystickHole, h=5, center=true);
    
    // UNDONE: Just print the front of the device for button testing.
    translate([0, 38.0, 0]) cube([100, 100, 100], center=true);
    translate([0, 0, -40]) cube([100, 100, 100], center=true);
}
// Insert mock PCB inside of controller for size check.
%translate([0, 0, pcbZ]) PcbWithJoystick();



// Draws the main body oval that houses the electronics.
module MainBody() {
    // Uses a hull stretched over rings to provide a rounded edge oval.
    // Includes a round bar behind the joystick to start a slant down to the bottom of
    // the body to give more room for the palm of the hand.
    hull() {
        // Rounded edge for the bottom oval.
        RoundedBodyEdge(bodyEdgeRadius);
        // Rounded edge for the top oval with the front removed to make room for the slant.
        difference() {
            RoundedBodyEdge(bodyHeight - bodyEdgeRadius);
            translate([0, topDiameter/2 + bodySlantStart, bodyHeight - bodyEdgeRadius]) 
                cube([topDiameter, topDiameter, 2*bodyEdgeRadius], center=true);
        }
        // The rounded edge that goes across the top of the body to start the slant.
        // Built in 2 pieces so that a nice union of the two top rings occurs.
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

// Draws a standoff with the desired inner and outer diameters.
module standOff(od, id, h, holeH) {
    difference() {
        cylinder(d=od, h=h);
        translate([0, 0, -0.1]) cylinder(d=id, h=holeH+0.2);
    }
}

module ButtonSupport() {
    translate([0, 0, buttonZPos]) {
        difference() {
            cylinder(r=topRadius, h=buttonHeight+2*buttonSlant+2.0, center=true);
            translate([0, buttonBumperLocation, 0]) 
                cube([topDiameter, topDiameter, buttonHeight+2*buttonSlant+2.0+0.2], center=true);
        }
    }
}


// Draws the PCB with thumb joystick. Centered on X/Y origin, bottom of PCB at Z=0.
module PcbWithJoystick() {
    // Center the PCB.
    translate([-33/2, -43/2, 0]) {
        union() {
            // Place joystick on PCB as found in PCB layout.
            translate([9.46+16/2, 7+16/2, pcbThickness])
                joystick();
            difference() {
                // PCB, Place lower left corner at origin to make it easier to match KiCAD part placements.
                translate([33/2, 43/2, 0]) 
                    RoundedRect(cornerRadius=7.62, width=33, length=43, height=1.6);
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
