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

// ************
//  Parameters
// ************
// Dimensions of the PCB.
pcbWidth = 64.24;
pcbHeight = 50.86;

// Desired clearance around the PCB.
pcbClearance = 2.0;

// Dimensions for triangular internal braces.
braceRadius = 2;
// UNDONE: braceXOffset = 51/2 + 3 + braceRadius;

// Dimensions of round power switch.
powerSwitchDiameter = 16.2;
powerSwitchRadius = powerSwitchDiameter / 2;
powerSwitchClearanceDiameter = 22.0; // UNDONE: (20.0+21.7)/2;

// Dimensions of switch label text.
textSize = 4;
textThickness = 1;
textYDistanceFromCenter = 21;
textClearance = 2.0;

// Dimensions of panel face
panelCornerRadius = 2.5;
panelWidth = panelCornerRadius*4 + pcbWidth + pcbClearance*2 + powerSwitchClearanceDiameter;
panelHeight = panelCornerRadius*4 + max(pcbHeight + pcbClearance*2, 2*powerSwitchClearanceDiameter, 2*powerSwitchDiameter+2*textSize);
panelThickness = 4;

// Where the triangular internal brace should be located to fortify the switches.
braceXOffset = -panelWidth/2 + 2*panelCornerRadius + powerSwitchClearanceDiameter + braceRadius;

// Dimensions of viewable area of LCD.
lcdViewableWidth = 28.32;
lcdViewableHeight = 28.32;

// Dimensions of larger backlight surface of LCD.
lcdBacklightWidth = 31.62 + 0.2;
lcdBacklightHeight = 35.2 + 0.2;
lcdBacklightDepth = 2.0;
lcdBacklightUpOffset = 2.24 + 0.1;

// The hole for the LCD screen will be beveled.
lcdBevelAngle = 45.0;
lcdBevelDepth = panelThickness - lcdBacklightDepth;
lcdBevelDiff = tan(lcdBevelAngle) * lcdBevelDepth;
lcdBevelWidth = lcdViewableWidth + 2 * lcdBevelDiff;
lcdBevelHeight = lcdViewableHeight + 2 * lcdBevelDiff;

// Dimensions of standoffs for LCD.
standoffHeight = 1.6;
standoffRadius = 2.5;

// Dimensions & locations of LCD M2.5 screw mounting holes.
mountHoleDiameter = 2.5+0.2;
mountHoleRadius = mountHoleDiameter / 2;
mountHoleWidth = 36.83;
mountHoleHeight = 38.862;
// UNDONE: mountHoleBuryDepth = 4.5 - standoffHeight;

// Where to center the screen.
screenCenterDistanceFromRight = panelCornerRadius*2 + pcbClearance + pcbWidth/2;
screenCenterX = panelWidth / 2 - screenCenterDistanceFromRight;

// Dimensions of keyed round manual switch.
manualSwitchDiameter = 12;
manualSwitchRadius = manualSwitchDiameter/2;
manualSwitchKeyWidth = 1.7;
manualSwitchKeyHeight = 1.0;
// Dimensions of manual switch cover plate.
manualSwitchCoverRadius = 2.0;
manualSwitchCoverUpperWidth = 15.2;
manualSwitchCoverUpperHeight = 7.0;
manualSwitchCoverLowerWidth = 17.2;
manualSwitchCoverLowerHeight = 40.75 - manualSwitchCoverUpperHeight;
manualSwitchCoverThickness = 0.84;

// Dimensions for bottom of enclosure.
sideDepth = 40;
bottomLength = 65;
bottomHolesRadius = 3.5 / 2;
bottomHolesZOffset = sideDepth + bottomHolesRadius;
bottomHolesXOffset = 39.65 / 2;

// Dimensions of groove along top of sides for later mounting of top/back piece.
grooveDepth = panelCornerRadius;
grooveHeight = panelCornerRadius;
grooveLength = sideDepth - 5 * 2;
grooveTopOffset = 3 + grooveHeight/2;

// Place power switch in upper left corner.
powerSwitchX = -panelWidth/2 + panelCornerRadius*2 + powerSwitchClearanceDiameter/2;
powerSwitchY = panelHeight/2 - panelCornerRadius*2 - powerSwitchClearanceDiameter/2;
// Place manual switch in lower left corner.
manualSwitchX = -panelWidth/2 + panelCornerRadius*2 + powerSwitchClearanceDiameter/2;
manualSwitchY = powerSwitchY - 2*textClearance - textSize - 2*powerSwitchRadius; // UNDONE: -panelHeight/2 + (panelHeight - mountHoleWidth)/2; 

// Calculate offset of manual switch cover plate with respect to switch center itself.
manualSwitchCoverOffset = 19.96;

// Dimensions of the sliding door on the back of the PDB.
// Thickness of the top of the door. It will also have the following lip.
doorTopThickness = 6.0;
// How much of a lip the door will have at the top to act as a handle.
doorTopOverlap = 1.0;
// The slope of the door edge, used to calculate the required door thickness.
doorSlope = 45;
// How much open space there should be at the bottom of the door to allow cable entry.
doorOpening = 20.0;
// How much clearance to add for the matching slot in the PDB.
doorClearance = 0.1;




// The sliding back door of the PDB.
module Door(clearance) {
    let(doorHeight = panelHeight - 2*panelCornerRadius - doorOpening,
        doorWidth = panelWidth-2*panelCornerRadius+2*clearance,
        doorThickness = panelCornerRadius*tan(doorSlope))
    {
        translate([0, doorOpening/2+panelCornerRadius, 0]) {
            hull() {
                translate([0, 0, doorThickness/2+clearance])
                    cube([doorWidth, doorHeight, 0.0001], center=true);
                translate([0, 0, -doorThickness/2-clearance])
                    cube([doorWidth-2*panelCornerRadius, doorHeight, 0.0001], center=true);
            }
        }
    }
}

module PDB() {
    difference() {
        Case();
        // Make slot for sliding back door.
        translate([0, 0.001, -sideDepth+doorTopThickness/2])
            Door(clearance=doorClearance);
    }
}

module Case() {
    // The front panel is a rounded rect with holes removed for the switches, screen, mounting holes, etc.
    difference() {
        // Front panel to which screen and switches are attached.
        RoundedRect(width=panelWidth, height=panelHeight, thickness=panelThickness, radius=panelCornerRadius);
        // Remove rectangle for viewable area of LCD.
        translate([screenCenterX, 0, panelThickness/2]) 
            cube([lcdViewableWidth, lcdViewableHeight, panelThickness+0.2], center=true);
        // Remove rectangle from back for larger backlight of LCD.
        translate([screenCenterX, -(lcdBacklightHeight/2-lcdViewableHeight/2)+lcdBacklightUpOffset, (lcdBacklightDepth+0.1)/2-0.1]) 
            cube([lcdBacklightWidth, lcdBacklightHeight, lcdBacklightDepth+0.1], center = true);
        // Remove holes for 4 screws to mount LCD/PCB to panel using M2.5 hardware. 
        translate([screenCenterX+mountHoleWidth/2, mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
        translate([screenCenterX-mountHoleWidth/2, -mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
        translate([screenCenterX-mountHoleWidth/2, mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
        translate([screenCenterX+mountHoleWidth/2, -mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
        // Remove hole for round power switch, upper left.
        translate([powerSwitchX, powerSwitchY, -0.1]) cylinder(r=powerSwitchRadius, h=panelThickness+0.2);
        // Remove hole for round manual switch, lower left.
        translate([manualSwitchX, manualSwitchY, -0.1]) cylinder(r=powerSwitchRadius, h=panelThickness+0.2);
        // Bevel the LCD screen hole.
        translate([screenCenterX, 0, 0])
            hull() {
                translate([0, 0, panelThickness-lcdBevelDepth]) 
                    cube([lcdViewableWidth, lcdViewableHeight, 0.001], center=true);
                translate([0, 0, panelThickness]) 
                    cube([lcdBevelWidth, lcdBevelHeight, 0.001], center=true);
            }
        
        // Emboss switch names into surface of front panel.
        translate([powerSwitchX, powerSwitchY-powerSwitchRadius-textClearance-textSize, panelThickness-textThickness]) 
            translate([0, 0, textThickness/2]) linear_extrude(height=textThickness+0.1) 
                text(text="Power", size=textSize, font="Menlo:style=Bold", halign="center", valign="bottom");
        translate([manualSwitchX, manualSwitchY-powerSwitchRadius-textClearance-textSize, panelThickness-textThickness]) 
            translate([0, 0, textThickness/2]) linear_extrude(height=textThickness+0.1) 
                text(text="Manual", size=textSize, font="Menlo:style=Bold", halign="center", valign="bottom");
    }

    // Add standoffs between back of panel and LCD PCB.
    translate([screenCenterX+mountHoleWidth/2, mountHoleHeight/2, -standoffHeight])
        Standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
    translate([screenCenterX-mountHoleWidth/2, -mountHoleHeight/2, -standoffHeight])
        Standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
    translate([screenCenterX-mountHoleWidth/2, mountHoleHeight/2, -standoffHeight])
        Standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
    translate([screenCenterX+mountHoleWidth/2, -mountHoleHeight/2, -standoffHeight])
        Standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);

    // Add bottom of enclosure with mounting holes removed.
    difference() {
        hull() {
            translate([panelWidth/2-panelCornerRadius, -(panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=bottomLength);
            translate([-(panelWidth/2-panelCornerRadius), -(panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=bottomLength);
        }
        // Holes to mount PDB enclosure to battery tray.
        translate([bottomHolesXOffset, -(panelHeight/2-panelCornerRadius), -(bottomHolesRadius+panelThickness+bottomHolesZOffset)]) 
            rotate([-90, 0, 0]) 
                cylinder(r=bottomHolesRadius, h=panelCornerRadius*2+0.2, center=true);
        translate([-bottomHolesXOffset, -(panelHeight/2-panelCornerRadius), -(bottomHolesRadius+panelThickness+bottomHolesZOffset)]) 
            rotate([-90, 0, 0]) 
                cylinder(r=bottomHolesRadius, h=panelCornerRadius*2+0.2, center=true);
    }

    // Add top of enclosure.
    difference() {
        hull() {
            translate([panelWidth/2-panelCornerRadius, (panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
            translate([-(panelWidth/2-panelCornerRadius), (panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
        }
    }

    // Add sides to enclosure.
    difference() {
        hull() {
            translate([panelWidth/2-panelCornerRadius, -(panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
            translate([panelWidth/2-panelCornerRadius, (panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
        }
    }
    difference() {
        hull() {
            translate([-(panelWidth/2-panelCornerRadius), -(panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
            translate([-(panelWidth/2-panelCornerRadius), (panelHeight/2-panelCornerRadius), 0]) 
                rotate([180, 0, 0]) 
                    cylinder(r=panelCornerRadius, h=sideDepth);
        }
    }

    // UNDONE: The panel face is probably strong enough for buttons without these.
    // Add internal braces to help support the switches.
    braceAngle = atan2(panelHeight, sideDepth);
    braceInset = braceRadius * tan(braceAngle);
    *hull() {
        translate([braceXOffset, -(panelHeight/2-panelCornerRadius), 0]) 
            rotate([180, 0, 0]) 
                cylinder(r=braceRadius, h=sideDepth);
        translate([braceXOffset, -panelHeight/2+braceInset, -sideDepth+braceInset]) 
            rotate([-braceAngle, 0, 0]) 
                cylinder(r=braceRadius, h=sqrt(panelHeight*panelHeight+sideDepth*sideDepth)-braceRadius-braceInset);
    }
}



module RoundedRect(width, height, thickness, radius) {
    hull() {
        translate([width/2-radius, height/2-radius, 0]) 
            cylinder(r=radius, h=thickness);
        translate([-(width/2-radius), -(height/2-radius), 0]) 
            cylinder(r=radius, h=thickness);
        translate([width/2-radius, -(height/2-radius), 0]) 
            cylinder(r=radius, h=thickness);
        translate([-(width/2-radius), height/2-radius, 0]) 
            cylinder(r=radius, h=thickness);
    }
}

module ManualSwitchOpening(thickness) {
    difference() {
        cylinder(r=manualSwitchRadius, h=thickness);
        translate([0, -(manualSwitchRadius-manualSwitchKeyHeight/2), (thickness+0.2)/2]) 
            cube([manualSwitchKeyWidth, manualSwitchKeyHeight, thickness+0.2], center=true);
    }
    translate([0, manualSwitchCoverOffset, 0]) {
        hull() {
            translate([manualSwitchCoverUpperWidth/2-manualSwitchCoverRadius, 
                       manualSwitchCoverUpperHeight-manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness])
                cylinder(r=manualSwitchCoverRadius, h=manualSwitchCoverThickness);
            translate([-(manualSwitchCoverUpperWidth/2-manualSwitchCoverRadius), 
                       manualSwitchCoverUpperHeight-manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness])
                cylinder(r=manualSwitchCoverRadius, h=manualSwitchCoverThickness);
            translate([manualSwitchCoverUpperWidth/2-manualSwitchCoverRadius, 
                       manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness/2])
                cube([manualSwitchCoverRadius*2, manualSwitchCoverRadius*2, manualSwitchCoverThickness], center=true);
            translate([-(manualSwitchCoverUpperWidth/2-manualSwitchCoverRadius), 
                       manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness/2])
                cube([manualSwitchCoverRadius*2, manualSwitchCoverRadius*2, manualSwitchCoverThickness], center=true);
        }
        translate([0, 0.001, 0]) hull() {
            translate([manualSwitchCoverLowerWidth/2-manualSwitchCoverRadius, 
                       -(manualSwitchCoverLowerHeight-manualSwitchCoverRadius), 
                       thickness-manualSwitchCoverThickness])
                cylinder(r=manualSwitchCoverRadius, h=manualSwitchCoverThickness);
            translate([-(manualSwitchCoverLowerWidth/2-manualSwitchCoverRadius), 
                       -(manualSwitchCoverLowerHeight-manualSwitchCoverRadius), 
                       thickness-manualSwitchCoverThickness])
                cylinder(r=manualSwitchCoverRadius, h=manualSwitchCoverThickness);
            translate([manualSwitchCoverLowerWidth/2-manualSwitchCoverRadius, 
                       -manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness/2])
                cube([manualSwitchCoverRadius*2, manualSwitchCoverRadius*2, manualSwitchCoverThickness], center=true);
            translate([-(manualSwitchCoverLowerWidth/2-manualSwitchCoverRadius), 
                       -manualSwitchCoverRadius, 
                       thickness-manualSwitchCoverThickness/2])
                cube([manualSwitchCoverRadius*2, manualSwitchCoverRadius*2, manualSwitchCoverThickness], center=true);
        }
    }
}

module Standoff(radius, holeRadius, height) {
    difference() {
        cylinder(r=radius, h=height);
        translate([0, 0, -0.1]) 
            cylinder(r=holeRadius, h=height+0.2);
    }
}



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
