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
// Rough CAD outline of the Sparkfun Thumb Joystick.
// https://www.sparkfun.com/products/9032



module joystick() {
    // Black plastic portion of the thumbstick which can be popped off of the joystick gimbal.
    translate([0, 0, 11.2]) 
        let(topHeight=13.68) union() {
            let(roundedEdgeRadius=2.5, large=100) hull() {
                // Top rounded surface of the joystick.
                difference() {
                    joystickTop();
                    // Only keep the part above the start of the rounded edge.
                    translate([0, 0, (topHeight+roundedEdgeRadius)-large/2]) 
                        cube([large, large, large], center=true);
                }
                // The rounded edge on the top portion of the joystick.
                difference() {
                    intersection() {
                        joystickRoundedEdge();
                        joystickTop(); 
                    }
                    // Remove the area below 13.68mm where the shaft of the joystick starts
                    translate([0, 0, topHeight/2]) 
                        cube([large, large, topHeight], center=true);
                }
            }
        // The plastic shaft of the joystick.
        let(shaftLength=5, shaftRadius=10.5/2)
            translate([0, 0, topHeight-shaftLength]) 
                cylinder(r=shaftRadius, h=shaftLength);
        // The rounded base at the bottom of the plastic joystick.
        let(baseRadius=26.0/2, roundedRadius=10.5, h=8.5, a=asin(h/roundedRadius), points = [ [0, 0], for(i=[0:1:a]) [cos(i)*roundedRadius, sin(i)*roundedRadius] ])
            hull() rotate_extrude()
                translate([baseRadius-roundedRadius, 0, 0])
                    polygon(points);
    }
    // The metal frame at the base of the joystick, containing the gimbal.
    translate([0, 0, 12.2/2]) cube([16.0, 16.0, 12.2], center=true);
    // The pots on the outside of the metal frame.
    translate([0, -16.0/2.0-3.18/2, 12.0/2.0]) 
        cube([9.6, 3.18, 12.0], center=true);
    rotate([0, 0, 90]) 
        translate([0, -16.0/2.0-3.18/2, 12.0/2.0]) 
            cube([9.6, 3.18, 12.0], center=true);
    // The joystick press switch.
    // White plastic base.
    translate([-16.0/2-7.0, -10.43/2, 0]) cube([7.0, 10.43, 2.4]);
    // Momentary switch.
    //  Base
    translate([-16.0/2-6.0, -6.0/2, 2.4]) cube([6.0, 6.0, 1.4]);
    //  Round button
    translate([-16.0/2-6.0/2, 0, 2.4+1.4]) cylinder(d=3.46, h=1.64);
}

// Top rounded surface of the joystick.
module joystickTop() {
    let(topRadius=25.0, roundedRadius=2.5)
        translate([0, 0, 17.3-topRadius]) 
            sphere(r=topRadius);
}

// The rounded edge on the top portion of the joystick.
module joystickRoundedEdge() {
    let(roundedRadius=2.5)
        translate([0, 0, 13.68])
            rotate_extrude()
                translate([20/2-roundedRadius, 0, 0])
                    circle(r=roundedRadius);
}
