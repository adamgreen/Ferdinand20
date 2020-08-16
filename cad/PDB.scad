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
// Parameters
// Dimensions of panel face
panelWidth = 150-16;
panelHeight = 81;
panelThickness = 4;
panelCornerRadius = 2.5;

// Dimensions of viewable area of LCD.
lcdViewableWidth = 28.32;
lcdViewableHeight = 28.32;

// Dimensions of larger backlight surface of LCD.
lcdBacklightWidth = 35.2 + 0.2;
lcdBacklightHeight = 31.62 + 0.2;
lcdBacklightDepth = 2.0;
lcdBacklightRightOffset = 2.24 + 0.1;

// Dimensions & locations of LCD M2.5 screw mounting holes.
mountHoleDiameter = 2.5+0.2;
mountHoleRadius = mountHoleDiameter / 2;
mountHoleWidth = 38.862;
mountHoleHeight = 36.83;

// Dimensions of round power switch.
powerSwitchDiameter = 16.2;
powerSwitchRadius = powerSwitchDiameter / 2;

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

// Dimensions of switch label text.
textSize = 4;
textThickness = 1;
textYDistanceFromCenter = 21;

// Dimensions of standoffs for LCD.
standoffHeight = 1.6;
standoffRadius = 2.5;

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

// Dimensions for triangular internal braces.
braceRadius = 2;
braceXOffset = 51/2 + 3 + braceRadius;

// Place power switch half way between left edge and left mount screws.
powerSwitchX = -panelWidth/2 + (panelWidth - mountHoleWidth)/4;
// Place manual switch half way between right edge and right mount screws.
manualSwitchX = panelWidth/2 - (panelWidth - mountHoleWidth)/4;

// Calculate offset of manual switch cover plate with respect to switch center itself.
manualSwitchCoverOffset = 19.96;



// The front panel is a rounded rect with holes removed for the switches, screen, mounting holes, etc.
difference() {
    // Front panel to which screen and switches are attached.
    hull() roundedRect(width=panelWidth, height=panelHeight, thickness=panelThickness, radius=panelCornerRadius);
    // Remove rectangle for viewable area of LCD.
    translate([0, 0, panelThickness/2]) 
        cube([lcdViewableWidth, lcdViewableHeight, panelThickness+0.2], center=true);
    // Remove rectangle from back for larger backlight of LCD.
    translate([-(lcdBacklightWidth/2-lcdViewableWidth/2)+lcdBacklightRightOffset, 0, (lcdBacklightDepth+0.1)/2-0.1]) 
        cube([lcdBacklightWidth, lcdBacklightHeight, lcdBacklightDepth+0.1], center = true);
    // Remove holes for 4 screws to mount LCD/PCB to panel using M2.5 hardware. 
    translate([mountHoleWidth/2, mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
    translate([-mountHoleWidth/2, -mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
    translate([-mountHoleWidth/2, mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
    translate([mountHoleWidth/2, -mountHoleHeight/2, -0.1]) cylinder(r=mountHoleRadius, h=panelThickness+0.2);
    // Remove hole for round power switch, halfway between left edge and mounting holes.
    translate([powerSwitchX, 0, -0.1]) cylinder(r=powerSwitchRadius, h=panelThickness+0.2);
    // Remove hold for keyed manual switch, halfway between right edge and mounting holes.
    translate([manualSwitchX, 0, -0.1]) manualSwitchOpening(panelThickness+0.2);
    // Emboss switch names into surface of front panel.
    translate([powerSwitchX, -textYDistanceFromCenter, panelThickness-textThickness]) 
        translate([0, 0, textThickness/2]) linear_extrude(height=textThickness+0.1) 
            text(text="Power", size=textSize, font="Times New Roman:style=Bold", halign="center", valign="bottom");
    translate([manualSwitchX, -textYDistanceFromCenter, panelThickness-textThickness]) 
        translate([0, 0, textThickness/2]) linear_extrude(height=textThickness+0.1) 
            text(text="Manual", size=textSize, font="Times New Roman:style=Bold", halign="center", valign="bottom");
}

// Add a single layer bridge as support before starting manual switch hole.
translate([manualSwitchX, 0, panelThickness-manualSwitchCoverThickness-0.3+0.1]) 
    cylinder(r=manualSwitchRadius+0.2, h=0.3);

// Add standoffs between back of panel and LCD PCB.
translate([mountHoleWidth/2, mountHoleHeight/2, -standoffHeight])
    standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
translate([-mountHoleWidth/2, -mountHoleHeight/2, -standoffHeight])
    standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
translate([-mountHoleWidth/2, mountHoleHeight/2, -standoffHeight])
    standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);
translate([mountHoleWidth/2, -mountHoleHeight/2, -standoffHeight])
    standoff(radius=standoffRadius, holeRadius=mountHoleRadius, height=standoffHeight);

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
    // Remove slot along top of side for top/back to snap onto.
    translate([panelWidth/2-grooveDepth/2+0.001, (panelHeight/2-grooveTopOffset), -(sideDepth-grooveLength)/2]) 
        rotate([180, 0, 0]) 
            translate([0, 0, grooveLength/2])
                cube([grooveDepth, grooveHeight, grooveLength], center=true);
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
    // Remove slot along top of side for top/back to snap onto.
    translate([-(panelWidth/2-grooveDepth/2+0.001), (panelHeight/2-grooveTopOffset), -(sideDepth-grooveLength)/2]) 
        rotate([180, 0, 0]) 
            translate([0, 0, grooveLength/2])
                cube([grooveDepth, grooveHeight, grooveLength], center=true);
}

// Add internal braces to help support the switches.
braceAngle = atan2(panelHeight/2,sideDepth);
braceInset = braceRadius * tan(braceAngle);
hull() {
    translate([braceXOffset, -(panelHeight/2-panelCornerRadius), 0]) 
        rotate([180, 0, 0]) 
            cylinder(r=braceRadius, h=sideDepth);
    translate([braceXOffset, 0, braceInset]) 
        rotate([180-braceAngle, 0, 0]) 
            cylinder(r=braceRadius, h=sideDepth);
}
hull() {
    translate([-braceXOffset, -(panelHeight/2-panelCornerRadius), 0]) 
        rotate([180, 0, 0]) 
            cylinder(r=braceRadius, h=sideDepth);
    translate([-braceXOffset, 0, braceInset]) 
        rotate([180-braceAngle, 0, 0]) 
            cylinder(r=braceRadius, h=sideDepth);
}







module roundedRect(width, height, thickness, radius) {
    translate([width/2-radius, height/2-radius, 0]) 
        cylinder(r=radius, h=thickness);
    translate([-(width/2-radius), -(height/2-radius), 0]) 
        cylinder(r=radius, h=thickness);
    translate([width/2-radius, -(height/2-radius), 0]) 
        cylinder(r=radius, h=thickness);
    translate([-(width/2-radius), height/2-radius, 0]) 
        cylinder(r=radius, h=thickness);
}

module manualSwitchOpening(thickness) {
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

module standoff(radius, holeRadius, height) {
    difference() {
        cylinder(r=radius, h=height);
        translate([0, 0, -0.1]) 
            cylinder(r=holeRadius, h=height+0.2);
    }
}



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
