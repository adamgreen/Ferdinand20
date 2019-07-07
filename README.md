![SRS Logo](https://raw.githubusercontent.com/adamgreen/Ferdinand16/master/photos/srslogo.gif)
Tracking the build of my robot to compete in the
[Seattle Robotics Society's Robo-Magellan](https://robothon.org/rules-robo-magellan/) competition.



## Points of Interest
[SRS Robo-Magellan rules](https://robothon.org/rules-robo-magellan/) <br>
[Notes about our 2014 bot](https://github.com/adamgreen/Ferdinand14#readme) <br>
[Notes from my stalled 2016 attempt](https://github.com/adamgreen/Ferdinand16#readme) <br>
[How to Clone GitHub Repository](#clone-this-repo-and-its-submodules) <br>


## July 6th, 2019
I attended the summer run of the Robo-Magellan competition at Seattle Center today. Watching today's competition really got me thinking about what it would take to renew my 2016 Robo-Magellan bot efforts.

### Where to Pickup
* Restore my [Arlo Robotic Platform](https://www.parallax.com/product/arlo-robotic-platform-system) back to its original state.
  * I had previously removed one of the caster wheels in an attempt to have it work better on the grass but it would really take a redesign of the chassis to mount the motors and batteries differently to correct that issue.
  * I no longer plan to use this platform as the base for my Robo-Magellan robot but I would like to use it for initial software development inside and outside on smoother terrain until I complete my new chassis.
* Start researching what I need to purchase to start building my own [Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover#readme), a rover inspired by the Curiosity Rover currently up on the surface of Mars.
* Investigate whether I should upgrade any of the electronics:
  * Switch from the Cortex-M3 based LPC1768 to a more powerful Cortex-M4F based microcontroller?
  * Use a nRF51 based microcontroller for BLE wireless bridging of the UART for debugging instead of the current ESP8266 WiFi solution?
  * Switch to using a more modern single chip IMU solution?
  * Use OpenMV instead of the Pixie camera?
* Continue developing and testing the dead reckoning code using my existing Arlo platform.
* Develop and test the code to detect the orange traffic cones using my existing Arlo platform.
* Develop and test the obstacle advoidance code using the ultrasonic sensors on my Arlo.



## Clone this Repo and its Submodules
Cloning now requires a few more options to fetch all of the necessary code from submodules.

``` console
git clone --recursive git@github.com:adamgreen/Ferdinand20.git
```

* In the gcc4mbed subdirectory you will find multiple install scripts.  Run the install script appropriate for your
platform:
** Windows: win_install.cmd
** OS X: mac_install
** Linux: linux_install
* You can then run the BuildShell script which will be created during the install to properly configure the PATH
environment variable.  You may want to edit this script to further customize your development environment.

If you encounter any issues with the install then refer to the **Important Notes** section of the README in the gcc4mbed
subdirectory.
