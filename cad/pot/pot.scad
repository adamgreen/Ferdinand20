/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Diagram of potentiometer internals for explaining what happens in LX-16A servo.

angle = 0;

potDiameter = 320.0;
trackWidth = 30.0;
padExternalDiameter = 12.0;
padInternalDiameter = 5.0;
padDistanceFromPot = 10.0;
padAngleDelta = 5.0;
trackToPadWidth = 0.5 * padExternalDiameter;
centerRadius = 20.0;
wiperAngle = angle+60;


color("lightgrey")
{
    difference() {
        cylinder(d=potDiameter, h=1);
        resistiveTrack();
    }
}
color("orange") union()
{
    translate([0, 0, 0.1])
        resistiveTrack();
    padAndTrack(padAngleDelta);
    padOutsideOfPot(0);
    padAndTrack(-padAngleDelta);
    centerToPad();
}

color("black") union()
{
    centerCircle();
    carbonTrack();
}

color("red")
    translate([0, 0, 1.0])
        wiper(wiperAngle);

translate([0, potDiameter/2+5.0, 0])
    text(str("Angle: ", angle), halign="center", valign="bottom");


module resistiveTrack()
{
    distanceToTrack = 0.5*potDiameter-0.5*trackWidth;
    
    difference() {
        cylinder(d=potDiameter-1*trackWidth, h=1.1);
        cylinder(d=potDiameter-3*trackWidth, h=1.2);
        translate([0, -distanceToTrack/2, 1.2/2])
            cube([2*trackToPadWidth, distanceToTrack, 1.2], center=true);
    }
}

module carbonTrack()
{
    rotate([0, 0, -72.5])
        rotate_extrude(angle=360-40+5)
            translate([potDiameter/2-trackWidth, 0])
                square([trackWidth, 5.0], center=true);
}

module padAndTrack(angle)
{
    padOutsideOfPot(angle);
    trackToPad(angle);
}

module padOutsideOfPot(angle)
{
    rotate([0, 0, angle])
    {
        translate([0, -potDiameter/2-padExternalDiameter/2-padDistanceFromPot, 0])
        {
            difference()
            {
                cylinder(d=padExternalDiameter, h=1.0);
                translate([0, 0, -0.1])
                    cylinder(d=padInternalDiameter, h=1.2);
            }
        }
    }
}

module trackToPad(angle)
{
    trackLength = padDistanceFromPot+(padExternalDiameter-padInternalDiameter)/2+trackWidth;

    rotate([0, 0, angle])
        translate([0, -trackLength/2-potDiameter/2+trackWidth, 0.1/2+1.0])
            cube([trackToPadWidth, trackLength, 0.1], center=true);
}

module centerToPad()
{
    trackLength = potDiameter/2+padDistanceFromPot+(padExternalDiameter-padInternalDiameter)/2;

    translate([0, -trackLength/2, 0.1/2+1.0])
        cube([trackToPadWidth, trackLength, 0.1], center=true);
}

module centerCircle()
{
    translate([0, 0, 0.1/2+1.0])
        cylinder(r=centerRadius, h=0.1, center=true);
}


module wiper(angle)
{
    wiperLength = 0.5*potDiameter-trackWidth;
    wiperWidth = trackWidth/3;

    rotate([0, 0, angle])
    {
        union()
        {
            translate([0, -wiperLength/2, 1.0/2+1.0])
                cube([trackToPadWidth, wiperLength, 1.0], center=true);
            translate([0, -wiperLength, 1.0/2+1.0])
                cube([wiperWidth, wiperWidth, 1.0], center=true);
        }
    }
    translate([0, 0, 1.0/2+1.0])
        cylinder(r=centerRadius/2, h=1.0, center=true);
}


// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 3;
