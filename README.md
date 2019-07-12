![SRS Logo](https://raw.githubusercontent.com/adamgreen/Ferdinand16/master/photos/srslogo.gif)
Tracking the build of my robot to compete in the
[Seattle Robotics Society's Robo-Magellan](https://robothon.org/rules-robo-magellan/) competition.



## Points of Interest
[SRS Robo-Magellan rules](https://robothon.org/rules-robo-magellan/) <br>
[Notes about our 2014 bot](https://github.com/adamgreen/Ferdinand14#readme) <br>
[Notes from my stalled 2016 attempt](https://github.com/adamgreen/Ferdinand16#readme) <br>
[How to Clone GitHub Repository](#clone-this-repo-and-its-submodules) <br>



---
## July 11th, 2019
### Restoring Arlo
When I last worked on my Robo-Magellan bot, I had removed one of the caster wheels from my Parallax Arlo platform in a failed attempt to get it to work better outside on the grass. Even with this caster removed, the battery holder still high centered. One of my first work items for this project was to reinstall that caster:

![Arlo Before](photos/20190710-01.jpg)
![Arlo After](photos/20190710-02.jpg)

### Continue with LPC1768 Microcontroller
I first wanted to take a second look at various components I originally chose for the Ferdinand16 project. This includes the microcontroller. I considered a few other Cortex-M4F based microcontrollers with more memory and floating point support:
* FRDM-K64F
  * 1MB FLASH - 256KB RAM - 120 MHz
  * I already have one of these boards and mbed supports it quite well.
* FRDM-K66F
  * 2MB FLASH - 256KB RAM - 180 MHz

In the end, I have decided to stick with the mbed 2.0 SDK and the LPC1768 (Cortex-M3 based) for now since I am very familiar with that combo and I have already developed a good remote debugging and programming solution for it.

One useful addition would be a uSD card slot that could be used for data logging. These logs could be used for post-run analysis of any failures encountered during a run.

### Remote Debugging/Programming Solution
Currently the remote debugging/programming solution for my Robo-Magellan bot is to use a [ESP8266](https://www.adafruit.com/product/2821) running [JEELABS' esp-Link UART to WiFi bridge firmware](https://github.com/jeelabs/esp-link). This solution allows GDB to use its TCP/IP socket remote debugging functionality to connect to my [Monitor for Remote Inspection (MRI) library](https://github.com/adamgreen/mri#readme) which is always linked into the robot's firmware. I plan to continue using my MRI debug monitor for remote debugging and programming of my bot but I want to experiment with using a [Nordic nRF51](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF51822) based microcontroller with BLE wireless support as the bridge instead.

I am considering the use of a nRF51 instead of the ESP8266 for a few reasons:
* It wouldn't require the use of a portable WiFi router out in the field.
* I could easily write code to connect to the robot directly in the field from either macOS or iOS.
* I could build [mriprog](https://github.com/adamgreen/bb-8/tree/master/mriprog) functionality into the TCP/BLE bridge app on macOS so that I could just use the ```load``` command in GDB instead of a separate application for programming as I do currently.
* Enables the creation of a better user input story than what I have currently since it isn't currently possible to interrupt normal process flow when the user wants to give the robot a new command.
* I have learned a lot more about the nRF5x family since I last tried and failed at doing this. I think it is more likely to work on this attempt.
* I can always fallback to the existing ESP8266 solution if this experiment fails.

### Sawppy Drive Mechanics
I reviewed the [Sawppy Arduino code](https://github.com/Roger-random/Sawppy_Rover/blob/master/arduino_sawppy/arduino_sawppy.ino) to better understand its drive mechanics:
* When driving along, it functions like a 4-wheel ackerman steering rig, where the point of rotation always falls outside of the robot's chassis.<br>
<img src="photos/20190711-01.jpg" alt="Rover Driving Along" width="160" height="142"/>
* Once stopped, it can rotate all 4 outer wheels to point their axles towards the center of the robot chassis. This allows it to turn in place like a differential wheeled robot.<br>
<img src="photos/20190711-02.jpg" alt="Rover Turning in Place" width="160" height="145"/>
* Since all 6 wheels are driven to achieve the best traction possible, the code needs to calculate the relative turning radii for all wheels and drive each wheel at the correct rate to properly turn around the desired point of rotation.

### Sawppy Rover Parts Ordered
I used the [Sawppy Rover documentation](https://github.com/Roger-random/Sawppy_Rover/tree/master/docs#readme) to finalize a list of parts and tools that needed to order. I had been looking for an excuse to purchase a [Prusa i3](https://shop.prusa3d.com/en/3d-printers/181-original-prusa-i3-mk3-3d-printer.html) for sometime now. This 3D printer has consistently received good reviews in Make magazine and elsewhere. It is also the printer used at the local library's makerspace where it has worked reliably under heavy use. It's also nice to have someone local who is experienced with the same printer should I need help with it. I have just been waiting for an excuse to purchase one and this project gives me that excuse:)

The parts and tools are now all ordered and should start arriving in the upcoming days and weeks.

Ordered from [Prusa Research](https://shop.prusa3d.com/en/):
* 1 x [Prusa i3 MK3S](https://shop.prusa3d.com/en/3d-printers/181-original-prusa-i3-mk3-3d-printer.html)
* 3 x [Prusament PETG Jet Black 1kg](https://shop.prusa3d.com/en/prusament/802-prusament-petg-jet-black-1kg.html)

Ordered from [Misumi](https://us.misumi-ec.com):
* 3 x [Misumi HFS 3-series - 15mm square - 2 meter lengths](https://us.misumi-ec.com/vona2/detail/110300465870/)
* 3 x [Misumi Square Nuts for 15mm square aluminum - 100 count](https://us.misumi-ec.com/vona2/detail/110300465710/?rid=rid2)

Ordered from [Amazon](https://www.amazon.com):
* 10 x [LewanSoul LX-16A Full Metal Gear Serial Bus Servo](https://www.amazon.com/dp/B0748BQ49M)
* 1 x [LewanSoul BusLinker TTL/USB Debug Board for LX-16A Bus Servo](https://www.amazon.com/dp/B073WRLJB2)
* 1 x [6 pack of turnbuckles](https://amazon.com/dp/B01H5PZKMK)
* 1 x [100 pack of 608 skate bearings](https://www.amazon.com/dp/B073ST742Z)
* 2 x [50 pack of M3-0.5 Threaded Heat Set Inserts for 3D Printing](https://www.amazon.com/gp/product/B077CHFGVT)
* 1 x [Heat Set Installation Tip](https://www.amazon.com/dp/B078K72615)

Ordered from [McMaster-Carr](https://www.mcmaster.com):
* 3 x [100 pack of M3x8mm Socket Head Screws](https://www.mcmaster.com/91290a113)
* 3 x [100 pack of M3 washers](https://www.mcmaster.com/98269a420)
* 1 x [100 pack of M3x16mm Socket Head Screws](https://www.mcmaster.com/91290a120)
* 1 x [100 pack of M3 nuts](https://www.mcmaster.com/90592a085)
* 4 x [25 pack of M3x8mm Set Screws with Knurl-Grip](https://www.mcmaster.com/94285a327)
* 2 x [8mm Steel Rod - 3 foot lengths](https://www.mcmaster.com/8920k26-8920K263)
* 1 x [100 pack of Retaining Rings](https://www.mcmaster.com/97431a310)
* 1 x [8mm Reamer](https://www.mcmaster.com/8851a23)
* 4 x [Loctite 242 - Single Use](https://www.mcmaster.com/91458a111)

Tools that I already had and didn't need to order:
* Drill and/or drill press
* File for creating set screw detents in steel rods
* Dremel for cutting retaining ring grooves in steel rods
* Soldering iron for threaded heat set inserts

### Next Steps
* Start experimenting with a nRF5 microcontroller based UART to BLE bridge for debugging.
* Continue research into newer technologies:
  * Would be good to use a newer IMU chip if it has better specs. Has the additional benefit of proving that my IMU code can be used with more than a single device.
  * Should I use OpenMV instead of the currently planned Pixie Cam?
* Buy a metric tap and die set locally.



----
## July 6th, 2019
I attended the summer run of the Robo-Magellan competition at Seattle Center today. Watching today's competition really got me thinking about what it would take to renew my [2016 Robo-Magellan bot efforts](https://github.com/adamgreen/Ferdinand16#readme).

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
* Develop and test the obstacle avoidance code using the ultrasonic sensors on my Arlo.



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
