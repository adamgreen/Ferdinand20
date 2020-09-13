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
use <Sparkfun_9032.scad>

// **********
// Parameters
// **********
// Dimensions of the PCB to be fit within the top of the remote control.
pcbWidth = 33.0;
pcbLength = 43.0;
pcbThickness = 1.6;
// The top of the remote control is actually an oval created by scaling with these values.
bodyWidthScale = 0.9;
bodyLengthScale = 1.1;
// The amount of free space that should exist around the PCB within the enclosure.
pcbClearance = 3.0;

// The total height of the remote control top. Doesn't include the handle.
bodyHeight = 27.0;
// The radius of the rounded edges used on the remote control top so that there are no sharp edges.
bodyEdgeRadius = 2.0;
// Distance from center of top where the palm slant should start.
bodySlantStart = 9.0;

// How much to scale up the deadman button before using it to cutout a hole for it in the remote control top.
buttonHoleTolerance = 1.03;

// The diameter of the hole at the top of the remote control through which the joystick protrudes.
joystickHoleDiameter = 25.0;

// Dimensions of the lip placed around the large hole in the bottom of the remote control top.
// The handle will have a matching piece to mate with this lip.
// The thickness of the oval at the top of handle which mates with the remote top.
handleLipThickness = 2.0;
// The amount of lip that the handle should come into contact with.
handleLipWidth = 2.0;
// The lip starts this far from the outer diameter of the remote control top.
handleLipStart = 1.25;
// The lip isn't just an overhang. It slops up from the interior sides of the top at this angle.
handleLipSlopeAngle = 45.0;

// Outer diameter of the 4 PCB standoffs in the remote control top.
pcbStandOffOD = 6.0;
// Inner diameter of the 4 PCB standoffs in the remote control top.
// Currently set to be tapped for M3 screw.
pcbStandOffID = 2.5;
// Large value used when it doesn't matter exactly how long something is since it will 
// be booleaned with something else to restrict object to proper length.
longLength = 100.0;
// How far the M3 bolt can be screwed into the standoff.
pcbStandOffHoleHeight = 8.0;
// The PCB is centered in the remote control top. 
// Its standoffs are offset by these amounts in the X and Y axis.
pcbStandOffX = 26.67;
pcbStandOffY = 30.988;
// How far the bottom of the PCB should be from the bottom of the remote control top.
pcbZ = 9.0;

// The deadman button subtends this angle on the outer surface of the enclosure.
// Must be small enough that the resulting button will pass between the PCB standoffs when pressed.
buttonAngle = 30.0;
// The rounded edge between the flat front of the button and the angled edges.
buttonEdgeRadius = 0.5;
// The thickness of the angled part of the button that extends outside of the enclosure.
buttonThickness = 3.0;
// The angled part of the button is this much larger buttonThickness mm from the front surface of the button.
buttonSlant = 1.0;
// The height of the button. If it is too large, it won't fit above the PCB on the standoffs.
buttonHeight = 11.0;
// The button should be centered at this height from bottom of the remote control top.
buttonZPos = 18.0;
// How far should the button be guided inside of the enclosure.
// The simple square part of the button that goes inside the enclosure will be this length too.
buttonGuideDepth = 9.5;
// Dimensions of the flange on the inside of the button which stops the button from falling out of the enclosure.
buttonFlangeWidth = 1.0;
buttonFlangeOffset = 0.0;
buttonFlangeThickness = 0.5;
// Dimensions of the arm which extends out of the bottom of the button to make contact
// with the deadman switch on the PCB.
// NOTE: These dimensions were selected to have the arm line up properly with the
//       deadman switch on the PCB.
buttonArmRadius = 2.0;
buttonArmHeight = 8.4;
buttonArmLength = 10.9;
buttonArmXOffset = -1.0;
buttonArmYOffset = 1.5;

// Dimensions of the 2xAAA battery pack that goes inside the handle.
batteryPackWidth = 25.9;
batteryPackThickness = 15.64;
batteryPackHeight = 63.2;
// Width and thickness clearance around battery pack.
sideClearance = 5.0;
// Height clearance around battery pack.
heightClearance = 5.0;
// The remote control handle is actually an oval created by scaling with these values.
handleXScale = 1.0;
handleYScale = 0.75;

// Diameter of rings around handle to act as grip between fingers.
fingerRingDiameter = 3.0;

// Distance from top for each finger ring.
fingerRingHeights = [21.5, 21.5+17];

// Bottom flange dimensions.
flangeHeight = 5.0;
flangeWidth = 5.0;
flangePoints = [[0, 0], [0, flangeHeight], [flangeWidth, 0]];

// How angled should the handle be with respect to the top of the remote.
rampAngle = 30.0;
// The depth of the indent in the top oval used to mate with the main handle.
handleIndent = 0.25;

// The amount of extra negative space to leave for the battery pack to fit into the handle.
handleBatteryPackClearance = 0.25;
// The dimensions of the snap hook used to hold the battery pack in.
batteryHookWidth = 8.0;
batteryHookTotalHeight = 15.0;
batteryHookArmThickness = 2.0;
batteryHookThickness = batteryHookArmThickness + 2.5;
batteryHookHeight = 4.0;
// The desired clearance around the battery hook.
batteryHookClearance = 1.0;

// The thickness of the tongue between handle and top.
handleTongueThickness=handleLipThickness/2;
handleTongueDepth=1.0;
handleTongueAngle=20.0;

// Hole and standoff used to attach handle to remote control top with M3 screw.
// Outer diameter of the standoff in the remote control top.
mountHoleStandOffOD = 6.0;
// Inner diameter of the 4 standoff in the remote control top.
// Currently set to be tapped for M3 screw.
mountHoleStandOffID = 2.5;
// How far the M3 bolt can be screwed into the standoff.
mountHoleStandOffHoleHeight = 8.0;
// The mounting hole diameter.
mountHoleDiameter = 3.2;
// The mounting hole is offset by these amounts in the X axis.
mountHoleStandOffX = 26.0;




// *****************************************************************
// Top of Remote Control - Holds PCB, joystick, and deadman button.
// *****************************************************************
// Calculated Values
// -----------------
// The distance from one corner of the PCB to other corner.
pcbDiameter = sqrt(pcbWidth*pcbWidth + pcbLength*pcbLength);
// The top of the remote control diameter is based on the PCB dimensions + specified clearance amount.
topDiameter = pcbDiameter + 2 * pcbClearance;
topRadius = topDiameter / 2;
// How wide is the palm rest slant where it starts at the top of the remote control.
bodySlantLength = 2 * sqrt(pow(topRadius, 2) - pow(bodySlantStart, 2));
// How high up into the inside of the enclosure does the lip have to start based on the desired angle?
handleLipSlopeHeight = (handleLipWidth+handleLipStart)*tan(90.0-handleLipSlopeAngle);


// Draws the top of the remote control.
module RemoteControlTop() {
    difference() {
        union() {
            // Use scale to convert circles into desired oval shape.
            scale([bodyWidthScale, bodyLengthScale, 1.0]) {
                difference() {
                    union() {
                        difference() {
                            // The outside shape of the remote control top.
                            // Holes for things like joysticks and deadman button will be removed later.
                            MainBody();
                            // Hollow it out and leave ~2mm of shell.
                            translate([0, 0, -2.0]) scale([0.93, 0.93, 1.0]) MainBody();
                        }
                        // Add extra support inside of the shell to support the deadman button.
                        ButtonGuide();
                        // Add a lip on the bottom of the remote control into which the handle will mate.
                        intersection() {
                            // Use intersection to just keep parts which fall inside of the main body.
                            MainBody();
                            LipForHandle();
                        }
                    }
                    // Make hole for button and its arm to pass through.
                    // NOTE: Not leaving any tolerance on the sides. Sand down the final button to make it
                    //       fit but stay tight enough that it can't tilt off axis and jam when pressed.
                    translate([0, 0.01, buttonZPos]) rotate([0, 0, 180]) {
                        scale([1.0, 1.0, buttonHoleTolerance]) {
                            Button();
                            ButtonArmSlot();
                        }
                    }

                }
            }

            // Add standoffs from top to which the PCB will be mounted.
            // Also include the standoff for the mounting screw.
            intersection() {
                // Use intersection to just keep the parts of the standoff which fall inside of the main body.
                MainBody();
                union() {
                    // Add 3 of the PCB standoffs.
                    translate([pcbStandOffX/2, pcbStandOffY/2, pcbZ+pcbThickness]) 
                        StandOff(od=pcbStandOffOD, id=pcbStandOffID, h=longLength, holeH=pcbStandOffHoleHeight);
                    translate([-pcbStandOffX/2, -pcbStandOffY/2, pcbZ+pcbThickness])
                        StandOff(od=pcbStandOffOD, id=pcbStandOffID, h=longLength, holeH=pcbStandOffHoleHeight);
                    translate([-pcbStandOffX/2, pcbStandOffY/2, pcbZ+pcbThickness]) 
                        StandOff(od=pcbStandOffOD, id=pcbStandOffID, h=longLength, holeH=pcbStandOffHoleHeight);
                    translate([0, mountHoleStandOffX, handleLipThickness])
                        StandOff(od=mountHoleStandOffOD, id=mountHoleStandOffID, h=longLength, holeH=mountHoleStandOffHoleHeight);
                }
            }
            // The last standoff needs to have notch cut out of it to make room for joystick rotation.
            difference() {
                intersection() {
                    MainBody();
                    translate([pcbStandOffX/2, -pcbStandOffY/2, pcbZ+pcbThickness])
                        StandOff(od=pcbStandOffOD, id=pcbStandOffID, h=longLength, holeH=pcbStandOffHoleHeight);
                }
                // Approximate the rotation of the joystick with an enlarged sphere.
                translate([(9.46+16/2)-33/2, (7+16/2)-43/2, 20.0]) sphere(d=28.0);
            }
        }
        // Remove hole on top for joystick.
        translate([(9.46+16/2)-33/2, (7+16/2)-43/2, bodyHeight]) 
            cylinder(d=joystickHoleDiameter, h=5, center=true);
        // Remove slot to mate with handle tongue.
        translate([0, 0.01, (handleLipThickness-handleTongueThickness)/2]) 
            rotate([0, 0, 90.0]) 
                ArcedTongue();
        
        // UNDONE: Just print the front of the device for button testing.
        *translate([0, 38.0, 0]) cube([100, 100, 100], center=true);
        *translate([0, 0, -40]) cube([100, 100, 100], center=true);
    }
}

// Draws the main body oval with slanted palm rest that houses the electronics.
module MainBody() {
    // Uses a hull stretched over rings to provide a rounded edge oval.
    // Includes a round bar behind the joystick to start a slant down to the bottom of
    // the body to give more room for the palm of the hand.
    hull() {
        // Rounded edge for the bottom oval.
        RoundedBodyEdge(bodyEdgeRadius);
        // Rounded edge for the top oval with the front removed to make room for the palm restslant.
        difference() {
            RoundedBodyEdge(bodyHeight - bodyEdgeRadius);
            translate([0, topRadius + bodySlantStart, bodyHeight - bodyEdgeRadius]) 
                cube([topDiameter, topDiameter, 2*bodyEdgeRadius], center=true);
        }
        // The rounded edge that goes across the top of the body to start the palm rest slant.
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
// pieces to get a smoothe intersection.
module TopOfBodySlant(width) {
    translate([0, bodySlantStart, bodyHeight - bodyEdgeRadius]) 
        rotate([0, 90, 0]) 
            cylinder(r=bodyEdgeRadius, h=width, center=true);
}

// Draws a standoff with the desired inner and outer diameters.
module StandOff(od, id, h, holeH) {
    difference() {
        cylinder(d=od, h=h);
        translate([0, 0, -0.1]) cylinder(d=id, h=holeH+0.2);
    }
}

// Place more plastic on the inside of the top enclosure through which the deadman 
// button will travel. It acts as a guide to keep the button aligned during its travel.
module ButtonGuide() {
    translate([0, 0, buttonZPos]) {
        // The support is a button thick disc cut by a chord where the inside button flange will meet it.
        difference() {
            cylinder(r=topRadius, h=buttonHeight+2*buttonSlant+2.0, center=true);
            translate([0, buttonFlangeLocation, 0]) 
                cube([topDiameter, topDiameter, buttonHeight+2*buttonSlant+2.0+0.2], center=true);
        }
    }
}

// Create a lip on the bottom of the remote control.
module LipForHandle() {
    // Start with disc and remove material to just leave:
    // * A recessed lip into which the handle will mate.
    // * An outer ring that extends from the previously mentioned lip and the outside of the top enclosure.
    difference() {
        cylinder(r=topRadius-bodyEdgeRadius, h=handleLipSlopeHeight+handleLipThickness);
        translate([0, 0, -0.01]) 
            cylinder(r=topRadius-bodyEdgeRadius-handleLipStart, h=handleLipThickness+0.01);
        translate([0, 0, handleLipThickness-0.01]) 
            cylinder(r1=topRadius-bodyEdgeRadius-(handleLipStart+handleLipWidth), r2=topRadius-bodyEdgeRadius, h=handleLipSlopeHeight+0.02);
    }
}

// Draws the PCB with thumb joystick. Centered on X/Y origin, bottom of PCB at Z=0.
// Note: Numbers are hardcoded in this module since they are based on actual measurements and
//       aren't parameters to be tweaked.
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
                    RoundedRect(cornerRadius=7.62, width=pcbWidth, length=pcbLength, height=1.6);
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

// Draws a simplified version of right angle switch found on the PCB.
// Draws a vertical cube out of which extends a round button.
// Note: Numbers are hardcoded in this module since they are based on actual measurements and
//       aren't parameters to be tweaked.
module rightAngleSwitch() {
    translate([-6.0/2, 0, 0]) union() {
        cube([6.0, 3.7, 7.0]);
        translate([6.0/2, 0, 4.0]) 
            rotate([90, 0, 0]) 
                cylinder(d=3.4, h=1.3);
    }
}



// *********************************************************************************
// Deadman Button - Slides in and out of remote control top to press switch on PCB.
// *********************************************************************************
// Calculated Values
// -----------------
// The rounded outermost edges of the button are centered at this radius.
buttonEdgeDistance = topRadius+buttonThickness-buttonEdgeRadius;
// The angle subtended by the small rounded edges on the front of the buttons.
buttonEdgeAngle = asin(buttonEdgeRadius/buttonEdgeDistance);
// The angle (slightly larger than buttonAngle) subtended by the button where it enters the enclosure, after the slant.
// Increase the circumference by buttonSlant. circumference = radians * radius.
// insideRadians * insideRadius = outsideRadians * outsideRadius + 2 * buttonSlant.
buttonInsideAngle = rad2deg((deg2rad(buttonAngle) * buttonEdgeDistance + 2 * buttonSlant) / topRadius);
// The width of the button where it passes inside of the enclosure.
buttonInsideWidth = deg2rad(buttonInsideAngle)*topRadius-2*buttonEdgeRadius;
// The height of the button where it passes inside of the enclosure.
buttonInsideHeight = buttonHeight+2*buttonSlant;
// The minimum distance from axis to rounded front edge of button.
buttonInsideRadiusDiff = cos(buttonInsideAngle/2)*topRadius;
// The distance from the axis to translate the square part of the button that travels through the enclosure.
buttonInsideRadius = buttonInsideRadiusDiff-buttonGuideDepth/2;
// The distance from the outside of enclosure to translate the button flange so that it sits on the inside of the button.
buttonFlangeLocation = (topRadius-buttonInsideRadiusDiff)+buttonGuideDepth-buttonFlangeOffset;

// Functions to convert between degrees and radians.
function deg2rad(a) = a * PI / 180;
function rad2deg(a) = a * 180 / PI;


// Draws the deadman switch button, with its arm to press the dead man switch.
module Button() {
    hull() {
        ButtonCorners();
        ButtonEdges();
        ButtonInside();
        ButtonThroughEnclosure();
    }
    buttonFlange();
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
        cube([buttonInsideWidth, buttonGuideDepth, buttonInsideHeight], center=true);
}

// Draw the rounded flange around the inside of the button to hold it in.
module buttonFlange() {
    translate([0, topRadius-buttonFlangeLocation-buttonFlangeThickness/2, -buttonFlangeWidth/2])
        rotate([90, 0, 0]) 
            translate([0, 0, -buttonFlangeThickness/2]) 
                RoundedRect(2.0, buttonInsideWidth+buttonFlangeWidth*2, buttonInsideHeight+buttonFlangeWidth, buttonFlangeThickness);
}

// Draw the arm that goes from the deadman button to the right angled switch.
module ButtonArm() {
    rotate([0, 180, 0]) {
        translate([0, buttonInsideRadius-buttonGuideDepth/2+buttonArmRadius+buttonArmYOffset, buttonInsideHeight/2-0.001]) {
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
    translate([-buttonArmRadius+buttonArmXOffset, buttonInsideRadius-buttonGuideDepth/2, -buttonInsideHeight/2-buttonArmHeight+0.001])
        cube([buttonArmRadius*2, buttonArmYOffset+buttonArmRadius, buttonArmHeight]);
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



// ***********************************************
// Remote Control Handle - Contains battery back.
// ***********************************************
// Calculated Values.
// ------------------
// Diameter/Radius of the handle to match battery pack size.
handleDiameter = sqrt(pow(batteryPackWidth, 2) + pow(batteryPackThickness, 2));
handleRadius = handleDiameter/2;
// Diameter/Radius of the handle with clearance around the battery back.
totalDiameter = handleDiameter+sideClearance*2;
totalRadius = totalDiameter / 2;
// How much will the angled top cut off the top of the handle on the lower side?
rampHeight = tan(rampAngle) * totalRadius;
// Total handle height, including highest part of sloped top and clearance for battery pack.
handleHeight = batteryPackHeight + rampHeight + tan(rampAngle)*batteryPackWidth/2 + 2 * heightClearance;
// Top oval needs to be reduced by the specified clearance value, to match the mating
// indent in the remote control top.
topOvalRadius = topRadius - bodyEdgeRadius - handleLipStart;
// Dimensions of the battery hole with clearance.
batteryHoleWidth=batteryPackWidth+handleBatteryPackClearance;
batteryHoleThickness=batteryPackThickness+handleBatteryPackClearance;


// Draw the handle and use scale to make it into an oval.
module RemoteControlHandle() {
    let(hookHoleThickness=batteryHookThickness+batteryHookClearance,
        hookHoleWidth=batteryHookWidth+batteryHookClearance,
        hookArmHeight=batteryHookTotalHeight-batteryHookHeight,
        hookHoleLength=batteryHookTotalHeight+batteryHookClearance) 
    {
        difference() {
            union() {
                scale([handleXScale, handleYScale, 1.0]) union() {
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
                    DrawBottomFlange();
                }
            }
            // Carve out a hole for the battery pack.
            translate([0, 0, longLength/2+heightClearance]) 
                cube([batteryHoleWidth, batteryHoleThickness, longLength], center=true);
            // Carve out a hole for the battery clip.
            translate([-batteryHoleWidth/2-hookHoleThickness/2+0.01, 0, heightClearance+batteryPackHeight+hookHoleLength/2-hookArmHeight+0.01])
                cube([hookHoleThickness, hookHoleWidth, hookHoleLength], center=true);
            translate([-batteryHoleWidth/2-batteryHookArmThickness+0.01, 0, heightClearance+0.1])
                BatteryClip();
            translate([-batteryHoleWidth/2-batteryHookArmThickness/2+0.01, 0, heightClearance+longLength/2])
                cube([batteryHookArmThickness, batteryHookWidth, longLength], center=true);
            
        }
    }
}

// Draws the ramp at the top of the handle to apply an angle between it and the main
// remote body.
module TopMask() {
    translate([0, 0, handleHeight-rampHeight]) 
        rotate([0, rampAngle, 0])
            translate([0, 0, handleHeight/2]) cube([1.5*totalDiameter, 1.5*totalDiameter, handleHeight], center=true);
}

// Draw a ring around the handle to fall between fingers.
module DrawFingerRing(height) {
    translate([0, 0, height]) 
        rotate_extrude(angle=360) 
            translate([handleRadius+sideClearance, 0, 0]) 
                circle(d=fingerRingDiameter);
}

// Draw the flange for the bottom of the handle.
module DrawBottomFlange() {
    rotate_extrude(angle=360) 
        translate([totalRadius-0.01, 0, 0]) 
            polygon(points=flangePoints);
}

// Draws the too oval which mates with the top of the remote control.
module DrawTopOval() {
    difference() {
        // Draw the oval.
        scale([bodyLengthScale, bodyWidthScale, 1.0])
            cylinder(r=topOvalRadius, h=handleLipThickness);
        // Carve out indent for main handle to mate with.
        translate([0, 0, handleIndent])
            rotate([0, -rampAngle, 0])
                translate([0, 0, -(handleHeight-rampHeight)]) 
                    RemoteControlHandle();
        // Carve out a hole for the battery pack.
        rotate([0, -rampAngle, 0])
            cube([batteryHoleWidth, batteryHoleThickness, longLength], center=true);
        // Carve out hole for the M3 screw to mount to remote top.
        translate([mountHoleStandOffX, 0, -0.1])
            cylinder(d=mountHoleDiameter, h=handleLipThickness+0.2);
    }
    // Add tongue to mate with remote control top.
    translate([0, 0, (handleLipThickness-handleTongueThickness)/2])
        ArcedTongue();
}

// Draw the battery clip insert to be glued into the handle.
module BatteryClip() {
    SnapHook(width=batteryHookWidth, totalHeight=batteryPackHeight+batteryHookHeight, armThickness=batteryHookArmThickness, hookThickness=batteryHookThickness, hookHeight=batteryHookHeight);
    translate([0, -batteryHookWidth/2, -batteryHookArmThickness]) 
        cube([batteryHoleWidth+batteryHookArmThickness, batteryHookWidth, batteryHookArmThickness]);
}

// Draws a snap hook (clip) with the specified dimensions.
module SnapHook(width, totalHeight, armThickness, hookThickness, hookHeight) {
    // The arm to which the hook is attached.
    translate([0, -width/2, 0]) 
        cube([armThickness, width, totalHeight-hookHeight]);
    // The triangular hook at end of arm.
    translate([0, 0, totalHeight-hookHeight]) 
        rotate([90, 0, 0]) 
            linear_extrude(height=width, center=true) 
                polygon([[0, 0], [hookThickness, 0], [0, hookHeight]]);
}

// Draws an arced tongue. Positive used on handle oval and negative used on mating surface in remote top.
module ArcedTongue() {
    scale([bodyLengthScale, bodyWidthScale, 1.0])
        rotate([0, 0, 180.0-handleTongueAngle/2]) 
            translate([0, 0, handleTongueThickness/2]) 
                rotate_extrude(angle=handleTongueAngle) 
                    translate([topOvalRadius+0.0, 0]) 
                        square([handleTongueDepth, handleTongueThickness]);
}

// Draw the rough shape of the 2x AAA battery pack.
module BatteryPack() {
    translate([0, 0, batteryPackHeight/2]) 
        cube([batteryPackWidth, batteryPackThickness, batteryPackHeight], center=true);
}




// *************************************
// Sets the smoothness of arcs/circles.
// *************************************
$fs = 0.1;
$fa = 3;
