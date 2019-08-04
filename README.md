![SRS Logo](https://raw.githubusercontent.com/adamgreen/Ferdinand16/master/photos/srslogo.gif)
Tracking the build of my robot to compete in the
[Seattle Robotics Society's Robo-Magellan](https://robothon.org/rules-robo-magellan/) competition.



## Points of Interest
[SRS Robo-Magellan rules](https://robothon.org/rules-robo-magellan/) <br>
[Notes about our 2014 bot](https://github.com/adamgreen/Ferdinand14#readme) <br>
[Notes from my stalled 2016 attempt](https://github.com/adamgreen/Ferdinand16#readme) <br>
[How to Clone GitHub Repository](#clone-this-repo-and-its-submodules) <br>
[Sawppy the Rover Build Instructions](https://github.com/Roger-random/Sawppy_Rover#readme) <br>



---
## August 3rd, 2019
### LX-16A Servo Couplers
<img src="photos/20190803-01.jpg" alt="LX-16A Couplers Printed" width="320" height="240"/>

I have now printed off 13 [LX-16A Servo Couplers](https://github.com/Roger-random/Sawppy_Rover/blob/master/STL/LX-16A%20-%20Coupler.stl) in PETG. I only need 10 for the [Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover#readme) build but I wanted to have a few spares. I ended up printing 6 of these at a time which took 1.5 hours. A few notes about these prints:
* I ended up just printing at 0.30mm layer resolution for all layers and not using the variable resolution slicing feature I mentioned in my previous post. Having these holes be a bit tighter fit around the heat set threaded inserts was actually an advantage.
* I took the first seven couplers to the local makerspace on Thursday evening to post-process. The central holes are so close to 8mm that the reamer isn't really required but it did function to clean out any PETG strings on the inside of the couplers. The main [post-processing task](https://github.com/Roger-random/Sawppy_Rover/blob/master/docs/Print%20Servo%20Parts.md#coupler) to complete on these couplers was to install the heat set threaded inserts using a soldering iron.
* I might have to straighten up these threaded inserts a bit in the future. They don't always go in completely square. It is a bit difficult to get fine control of the soldering iron when you are pushing these inserts in from the inside of the 8mm hole.
* I have printed off another 6 of these couplers and will post-process them this upcoming week.

### Wireless BLE based MRI Debugging Updates
![BLEMRI Prototype](photos/20190720-01.jpg)

While dogfooding my BLEMRI prototype over the last couple of weeks, I encountered a few issues that I addressed at the end of the week:
*  Originally the BLEMRI firmware running on the nRF51 expected only GDB remote packets to be sent so it would buffer data until it was full or the end of a packet was received. I modified the code so that now it buffers up the data until the buffer is full or no UART data has been sent from the LPC1768 for 1.5 serial frames. This fixes an issue where it failed to send the GDB remote protocol ack ('+') for the continue command which resulted in GDB timing out and sending the continue command again. This would cause MRI to break into the running LPC1768 and broke wireless debugging of most applications.
* The BLEMRI logs (file and stdout) on macOS now escape any bytes that aren't printable. Such bytes aren't common with GDB but they are very common when sending new firmware over to the LPC1768 using mriprog.
* I added a [deployBLE script](https://github.com/adamgreen/Ferdinand20/blob/master/software/blemri/test/deployBLE) which can use my previously created [mriprog](https://github.com/adamgreen/bb-8/tree/master/mriprog) program to wirelessly upload new firmware into the LPC1768 using BLEMRI.
* I found a few bugs in my previously created **mriprog** application while testing it with BLEMRI. I fixed those this week too.

### What is mriprog?
I mentioned my previously created [mriprog](https://github.com/adamgreen/bb-8/tree/master/mriprog) program in the last section and in previous posts but haven't really explained what it is. It's time I remedy that!

**mriprog** is a program which uses my [MRI](https://github.com/adamgreen/mri#readme) debug monitor to reprogram the LPC1768. MRI is a library which is linked into your code to enable GDB debugging via the serial port. Since MRI is linked into your code and running out of FLASH, it doesn't have an easy way to contain code capable of erasing FLASH and overwriting itself. **mriprog** works around these limitation by making creative use of the debugging facilities that MRI does support:
* The MRI debug monitor allows GDB to write to RAM, set registers, continue execution, etc.
* You just give **mriprog** the filename of the .elf that you desire to have uploaded into the LPC1768 and it parses the .elf to find the sections that need to be uploaded to the FLASH of the device.
* When **mriprog** first starts running, it pretends to be GDB and instructs MRI to write a [serial bootloader](https://github.com/adamgreen/bb-8/blob/master/mriprog/boot-lpc1768/main.c) into an area of RAM that it knows isn't being used by MRI for its stack or globals. Once the serial bootloader is loaded into RAM, the PC register (R15) is modified to point to the beginning of this bootloader. After the command is sent to resume execution, the serial bootloader will be running.
* **mriprog** can now switch communication protocols to use the one understood by the serial bootloader. It uses this protocol to erase the FLASH, load the new program into FLASH, and then reset the device to start the new program running.
* As long as the new code is always linked with the MRI debug monitor, **mriprog** should be able to upload new code into the device. This works great for wireless solutions like BLEMRI.

### Next Steps
* Print [Sawppy wheels](https://github.com/Roger-random/Sawppy_Rover/blob/master/STL/Wheel.stl). PrusaSlicer indicates that it will take 9.5 hours to print each of the six wheels.
* Continue work on the [LX-16A Servo](http://www.lewansoul.com/product/detail-17.html) software. Now that I know I can successfully send them commands from my mbed based LPC1768, I want to write a half-duplex serial driver to enable 2-way communication with them.
* Post process the last six [LX-16A Servo Couplers](https://github.com/Roger-random/Sawppy_Rover/blob/master/STL/LX-16A%20-%20Coupler.stl).



---
## July 31st, 2019
### OpenMV Camera has Arrived!
![Just opened OpenMV](photos/20190730-01.jpg)

The new OpenMV camera arrived on Tuesday. Since I decided to concentrate on the 3D printer this week, I didn't have time to experiment with it yet. I am really interested to see how it performs outside. I just want to get Sawppy's 3D printed parts rolling off of the printer first :)

### First Prints Off New Prusa 3D Printer
![Tree frog model from Prusa SD card](photos/20190731-01.jpg)

The photo above shows one of the earliest prints to come off of my new 3D printer. It is a PLA print of the tree frog model that came with the printer on the included SD card. Once that print came off of the printer, I was hooked! I just kept throwing various prints at it. Some from the same card and others from [Thingiverse](https://www.thingiverse.com).

Once I was done **playing around** with my new printer, I got down to business by loading up the [LX-16A Servo Coupler](https://github.com/Roger-random/Sawppy_Rover/blob/master/STL/LX-16A%20-%20Coupler.stl) in PrusaSlicer and started experimenting around with the various slicer settings. The image below shows some of those experimental results. From right to left:
* A 0.15mm layer height version printed in PLA. This one had a very nice looking round hole for the set screw.
* A 0.30mm layer height version printed in PLA. This one had very noticeable flat spots on the top and bottom of the set screw hole.
* A 0.30mm layer height version with finer layer height used at the top and bottom of the set screw hole. Printed in PLA. The variable layer heights used for this run results in better definition in the set screw hole with minimal impact on the print time (which increased from 17 to 18 minutes).
* The final example on the left is the same as the previous except it was printed out of black PETG instead.

<img src="photos/20190731-02.jpg" alt="Various Couplers" width="320" height="240"/>

Before printing more of the couplers in PETG (I need a total of 10 plus a few spares for my Sawppy build), I want to post process the current one and make sure that it is of sufficient quality. For example, I want to try installing the heat set insert for the set screw and make sure that it is formed well enough to allow easy installation of the inserts.



---
## July 29th, 2019
### My 3D Printer has Arrived!
![3D Printer in Entryway - Coton for Scale](photos/20190729-01.jpg)
![First 3D Print](photos/20190729-02.jpg)

My 3D printer, a Prusa i3 MK3S, and PETG filament arrived today. I will probably be doing a bit less electronics and coding this week so that I can instead get up to speed on this new toy and start cranking out Sawppy parts.



---
## July 28th, 2019
### Got LPC1768 Talking to LewanSoul LX-16A Servos
![Working LX-16A Servo](photos/20190728-01.gif)

I tried a new approach for the LX16-A driver. I skipped the BusLinker debug board and instead communicated directly with the [LX16-A servos]((http://www.lewansoul.com/product/detail-17.html)). I first tried it with the 5V level translation still in-place. It was a great success! The signal looked great with a full 5V to 0V swing and the servo responded just as expected.
<br><img src="photos/20190727-01.jpg" alt="5V Signal" width="320" height="240"/><br>
The servo also worked when connected directly to the 3.3V mbed with no level translation.
<br><img src="photos/20190727-02.jpg" alt="3.3V Signal" width="320" height="240"/>

Now I am going to flesh out this LX-16A driver a bit more. First I need to get half-duplex communications working from my LPC1768 so that LX-16A read commands can be performed.



---
## July 27th, 2019
### Wireless BLE based MRI Debugging Updates
![BLEMRI Prototype](photos/20190720-01.jpg)

I made a few updates to my BLEMRI prototype from last week:
* Software enable the pull-up on the UART Rx pin to fix UART framing errors encountered when the target LPC1768 isn't connected or powered off.
* Advertise a unique UUID for BLEMRI firmware so that the TCP/IP to BLE bridge program will only connect to BLEMRI devices and not just any device supporting Nordic's UART service.
* Make the macOS TCP/IP to BLE bridge application more robust by having it detect loss of the BLE connection and automatically attempt to reconnect.
* Add command line parsing to macOS bridge. Can now change TCP/IP port to use for GDB connections and enable logging of the traffic between GDB and MRI to a log file or stdout.
* The macOS bridge now catches CTRL+C presses and shuts down gracefully.

As I work on future firmware for the mbed-LPC1768, I will dogfood this prototype by using it to debug and program the LPC1768 wirelessly.

### New Battery for Powering LewanSoul LX-16A Servos
![Photo of battery & connectors](photos/20190724-01.jpg)

I wanted to be able to work on my LewanSoul LX-16A software driver at locations where it wouldn't be convenient to have my bench power supply so I decided to get a LiPo battery now like I will use on the final robot. Rather than order one off of the Internet, I decided to make a quick trip over to my local HobbyTown USA to purchase a **7.4V (2S) 5200mAH LiPo** battery and some matching XT60 connectors to facilitate servo experimentation while at the local makerspace.

### mbed Drivers for LewanSoul LX-16A Servos
It was now time to start playing with those [LewanSoul LX-16A servos](https://www.amazon.com/dp/B0748BQ49M) that I ordered off of Amazon a few weeks ago. I started by downloading some documentation from the **Download** tab of the [LewanSoul LX-16A product page](http://www.lewansoul.com/product/detail-17.html):
* LX-16A Bus Servo User Manual
* LewanSoul Bus Servo Communication Protocol

I also looked at the [sample Arduino code found in the Sawppy repository](https://github.com/Roger-random/Sawppy_Rover/blob/master/arduino_sawppy/lewansoul.cpp). Between this Arduino code and the **LewanSoul Bus Servo Communication Protocol** document, I had everything I needed to get started on writing the mbed driver for these servos.

#### Hit a Little Snag!
<img src="photos/20190726-01.jpg" alt="LX-16A Setup" width="320" height="240"/>

I took my hardware to the local makerspace this week and started working on this driver. While I got the beginnings of a driver completed during the visit, I couldn't get it to work. I probably spent more than an hour looking at my code and comparing it to working Arduino versions and couldn't find any differences that I hadn't already addressed.

Once I got back home, I was able to look at it in more detail with a logic analyzer and oscilloscope. It appears that when the LPC1768 is driving the serial Rx line, the line just transitions between 4V and 3.5V instead of 3.3V and 0V as expected. Before the LPC1768 is powered up, I see that the BusLinker has pulled the same Rx line up to 4V.

LewanSoul has an Arduino sample which I was able to load up onto my Arduino Uno. Much to my surprise, that sample worked a charm! I looked at its serial signaling on the oscilloscope and saw that it transitions were between 5V and 1V. Not the perfect 5V to 0V swing that I was hoping for but much better than my LPC1768 was doing.

<img src="photos/20190726-02.jpg" alt="LPC1768 Signal After 5V Level Translation" width="320" height="240"/><br>
**LPC1768 Signal After 5V Level Translation**

I thought that adding a level shifter to convert the 3.3V signaling of the LPC1768 to match the Arduino Uno's 5V would help but it still never pulls the signal lower than ~3.5V for the LOW portions of the signal. The mbed just doesn't seem to be able to sink enough current to pull down the Rx line on the BusLinker. The [level shifter](https://www.adafruit.com/product/757) that I connected uses pull-up resistors for sourcing the HIGH portions of the signal but still requires the LPC1768's pin to sink the current for LOW portions. At this point I removed the LPC1768 from the picture and just installed a 100 ohm resistor between the Rx line and ground. This resulted in a 3V drop across the resistor, for a 30mA (3v / 100ohm) current. That board must have a very strong pull-up resistor on that Rx line and the mbed just can't pull it down while the ATMEGA328P in the Uno can.

#### A Different Path
At this point, I am ready to give up on the BusLinker board and try communicating with the LX-16A servos directly from the LPC1768 using its Half-Duplex protocol. Maybe I will have more success with that approach.

### Sensors Ordered
* Ordered and received a [NXP 9-DoF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463) IMU board from Adafruit.
* Ordered an [OpenMV Cam H7](https://www.sparkfun.com/products/15325) from Sparkfun.

### Next Steps
* Try a new approach for the LX16-A driver. Bypass the BusLinker board and attempt to communicate directly with the LX16-A servos using half-duplex communication.
* Fix any issues in the BLEMRI firmware or macOS application that I encounter as I dogfood it.
* My 3D printer has shipped so I may be able to start playing with it next week.



---
## July 21st, 2019
### Christmas in July
<img src="photos/20190721-01.jpg" alt="Christmas in July" width="320" height="295"/>

The photo above shows everything that has arrived during the last week for the Sawppy Rover build. It kind of feels like Christmas in July around here :)

At this point everything has arrived except for the 3D printer and filament from Prusa Research. I don't expect those to arrive until sometime in August. That gives me time to work on other electronic and software related work items.

### Wireless BLE based MRI Debugging
![BLEMRI Prototype](photos/20190720-01.jpg)

I have used an ESP8266 to enable remote debugging and programming of robots (like [Ferdinand16](https://github.com/adamgreen/Ferdinand16#readme)) in the past. It works quite well when working on the robots within my home but it isn't as convenient when working on them out in the field. Switching to Bluetooth Low Energy (BLE) could be more convenient for a few reasons:
* wouldn't require a portable WiFi router out in the field.
* a phone could easily connect to the robot over BLE in the field for parameter tweaking, logging, etc and still stay connected to the Internet via LTE without any special setup.

I have tried to build a BLE to Serial bridge with a Nordic nRF51822 once before but I failed miserably and decided to use the ESP8266 instead. Since that time I have developed a few successful BLE projects with the nRF51 family of microcontrollers so I was a bit more confident that I would know how to debug and fix any issues encountered this time.

I figured it was time to make a second attempt with this Ferdinand20 project. To that end, I spent some time over the last week working on a new prototype. There were a few pieces of the puzzle that I need to put together before I could use **arm-none-eabi-gdb** to wirelessly debug a NXP-LPC1768 microcontroller:

* I should first point out that what I mean by debugging here is not just printf() debugging but actually being able to use GDB to do things like set breakpoints, single step, inspect and modify variables, etc. It would also be nice if I could just use the **load** command in GDB to upload new code into the LPC1768 as well.
* GDB can perform remote target debugging via:
  * Serial - I already have my [MRI debug monitor](https://github.com/adamgreen/mri) which allows GDB to debug a LPC1768 microcontroller via a serial port connection. MRI is a library that is linked into the firmware to be debugged. It supports serial connections from GDB and allows GDB to do things like set breakpoints/watchpoints, single step, view variables, modify variables, etc.
  * stdin/stdout Piping
  * UDP
  * TCP - This is the method commonly used by hardware debugging solutions like OpenOCD, JLinkGDBServer, etc. Tools like OpenOCD and JLinkGDBServer are software programs which act a bridge between GDB and JTAG/SWD debug probes. They connect to the debug probes (usually via USB) and allow GDB to connect via TCP/IP. The main work performed by these applications is to map GDB requests sent via the TCP/IP port into the correct JTAG/SWD operations. I needed to write an application like this to allow GDB to connect via TCP/IP and then tunnel the subsequent requests and responses over BLE.
* I [developed firmware](software/blemri/firmware) to run on the BLE capable nRF51 Development Kit to act as a bridge between BLE and Serial. I started with Nordic's BLE UART Service SDK sample (ble_app_uart) and made a few updates to make it work better for this purpose:
  * The original sample buffered up data received from the serial port until the buffer was full or a newline character was sent. GDB's remote protocol isn't line based. It instead uses a packet format which starts with the '$' character and ends with a '#' following by a two hexadecimal digit checksum. I modified the ```uartEventHandler()``` function to instead use the checksum as the delimiter.
  * The original sample would fail to send large MRI generated response packets out via BLE. It would fail the ```ble_nus_string_send()``` call with a ```BLE_ERROR_NO_TX_PACKETS``` error because it queued up the BLE packets faster than the nRF51's stack could send them. My solution to that issue can also be seen in the ```uartEventHandler()```.
    * It now queues them up in my own circular queue implementation,```g_chunkQueue```, by calling ```pushSerialChunk()```. They can then be dequeued and sent when the previous packet's transmission is successfully completed in the ```BLE_EVT_TX_COMPLETE``` event handler of ```handleBleEventsForApplication()```.
    * The above queuing only works when there is an outstanding BLE transmission to trigger the ```BLE_EVT_TX_COMPLETE``` event handler. This doesn't work on the first chunk of a response. In that case it schedules the initial transmission to be performed by the ```transmitFirstPacket()``` function from the main loop.
  * The original sample would encounter a ```APP_UART_COMMUNICATION_ERROR``` in ```uartEventHandler()``` when the UART's internal Rx buffer overflowed because the BLE stack was taking up too much CPU for the UART data to be processed in a timely fashion. I fixed that problem by bumping the UART driver up to ```APP_IRQ_PRIORITY_HIGHEST``` in ```initUart()```. Previously it was running at the same ```APP_IRQ_PRIORITY_LOWEST``` priority as the BLE stack. This change did add the complication that the ```uartEventHandler()``` could no longer call ```ble_nus_string_send()``` directly. That is why it now uses the application scheduler to run ```transmitFirstPacket()``` in the context of ```main()``` instead.
  * The above changes allowed GDB to reliably communicate with the MRI debug monitor over BLE but the performance was quite bad. I improved the performance by setting ```MIN_CONN_INTERVAL``` and ```MAX_CONN_INTERVAL``` to the minimum of 7.5 msecs. The sample's original intervals are better for power sensitive environments but in the case of robots, things like motors tend to dwarf BLE current consumption and robots tend to use larger batteries.
* I [developed code to run on my MacBook](software/blemri/macos) to act as a bridge between BLE and TCP/IP. This application communicates wirelessly with the nRF51 device using BLE. GDB connects to it via TCP/IP. GDB's request packets and MRI's response packets are tunneled between the TCP/IP and BLE connections. I still had the code from my previous attempt sitting on my development machine so that is what I started with. I updated its BLE code to more closely match my more recent BLE applications so that it would function better on recent macOS versions. I also found and corrected one of the bugs that I didn't notice on my previous attempt. The original code was transmitting via BLE whatever sized data that it received from GDB. However the maximum size of these BLE transmits is 20 bytes (maximum MTU) so larger transmit attempts were silently failing. This was no doubt one of the main causes of the problems that I hit on my previous attempt.

The animation below shows GDB connected wirelessly to the LPC1768 via my most recent prototype.

![BLEMRI Prototype showing GDB output](photos/20190720-02.gif)

#### Prototype Evaluation
So now that I have the BLE debugging prototype, the big question is how does it compare to my current WiFi solution?
* I have had intermittent problems with the WiFi solution where the connection will slowdown or stall for short periods of time. I haven't encountered that so far with this BLE prototype. It does slow down as I get further away from the target but that is to be expected.
* It's quite nice not needing to have a WiFi connection for it to work. I have already used it outside of my home with no WiFi connectivity while developing this prototype and it worked a charm.
* Even though both are running the UART portion of the bridge at 115200, this prototype doesn't feel as snappy. I attached a logic analyzer to the serial pins between the nRF51 and the LPC1768 and noticed a few things:
  * The trace below shows how a longer GDB request is broken down into 5 smaller chunks. These chunks are spread out according to the 7.5 msec connection interval.
  <br>![Request Packet Waveforms](photos/20190720-03.png)

  * The 3 larger chunks in the middle each contain 40 bytes of data which means that macOS sends 2 BLE packets (20 bytes/packet) per connection interval. In theory this can be as high as 4-6 depending on the OS. I don't know if there is a way to get macOS to increase it or not.
  <br>![Request Packet Zoomed In](photos/20190720-04.png)

### Inertial Measurement Unit
I used the [SparkFun 9 DoF - Sensor Stick](https://www.sparkfun.com/products/retired/10724) as the IMU in my previous Robo-Magellan bot attempts. It actually contains 3 separate sensors:
* Analog Devices ADXL345 Accelerometer
* Honeywell HMC5883L Magnetometer
* InvenSense ITG-3200 Gyroscope

Given that this board is no longer available and there might now be better options on the market, I would like to do some research into the more popular IMU solutions used by hobbyists these days and compare features.

Looking at Adafruit, I see these two solutions:
* [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055](https://www.adafruit.com/product/2472) - This one requires no external filtering as it performs the filtering itself.
* [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463) - Adafruit indicates that the sensors in this unit (especially the gyro) are superior in this one. They also have their own filtering code available that I could compare and contrast to my own.

Looking at Sparkfun, I see a few more options:
* [SparkFun 9DoF Razor IMU M0](https://www.sparkfun.com/products/14001) - This uses a InvenSense MPU-9250 and includes a microcontroller to run the filtering on board. Adafruit appears to have included the MPU-9250 in their [research](https://learn.adafruit.com/comparing-gyroscope-datasheets) and decided to go with the NXP parts instead.
* [SparkFun 9DoF IMU Breakout - LSM9DS1](https://www.sparkfun.com/products/13284) - Adafruit appears to have included the LSM9DS1 in their [research](https://learn.adafruit.com/comparing-gyroscope-datasheets) and decided to go with the NXP parts instead.
* [SparkFun VR IMU Breakout - BNO080](https://www.sparkfun.com/products/14686) - This one requires no external filtering as it performs the filtering itself. It uses a Bosh part like the [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055](https://www.adafruit.com/product/2472) but it seems like this one is probably more accurate. Meant to be used as an all-in-1 solution and I don't see detailed specs on each of the individual sensors.
* [SparkFun 9DoF IMU Breakout - ICM-20948](https://www.sparkfun.com/products/15335) - This looks like an interesting sensor to consider as well. It does have built-in filtering but it can probably be bypassed.

I really like the research and development that Adafruit puts into their products so I would definitely like to consider their [Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463).  They have already written sample filtering code for it and [compared it to other sensor solutions](https://learn.adafruit.com/comparing-gyroscope-datasheets) as well.

Of the Sparkfun offerings, I find the [9DoF IMU Breakout - ICM-20948](https://www.sparkfun.com/products/15335) offering to be the most interesting.

Lets compare the [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463) and [SparkFun 9DoF IMU Breakout - ICM-20948](https://www.sparkfun.com/products/15335) to my [current sensor package](https://www.sparkfun.com/products/retired/10724):

| Feature             | Current Sensor      | FXOS8700 + FXAS21002  | ICM-20948               |
|---------------------|---------------------|-----------------------|-------------------------|
| Accel Range         | 2 to 16g            | 2 to 8g               | 2 to 16g                |
| Accel Resolution    | 4mg                 | 0.244 to 0.976 mg     | 0.06mg to 0.5 mg        |
| Accel Sampling Rate | 6.25 to 3200 Hz     | 1.563 to 400Hz        | 4.5 to 4500 kHz         |
| Mag Resolution      | 0.73 to 4.35 mGauss | 1 mGauss              | 1.5 mGauss              |
| Mag Sampling Rate   | 160 Hz              | 1.563 to 400Hz        | 100 Hz                  |
| Gyro Range          | 2000 deg/sec        | 250 to 2000 deg/sec   | 250 to 2000 deg/sec     |
| Gyro Resolution     | 1/14.375 deg/sec    | 1/128 to 1/16 deg/sec | 1/131 to 1/16.4 deg/sec |
| Gyro Sampling Rate  | 4 to 8000 Hz        | 12.5 to 800 Hz        | 4.4 to 9000 Hz          |

Both of those newer sensors look better than the current sensor that I am using. One thing I didn't track in that table is the expected drift at room temperature. The FXAS21002 gyro datasheet indicates that it has a drift value that is lower than many of the others by an order of magnitude. I think I will order up the [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463) and attempt to port my Kalman filter to it if I have time. I can also try out Adafruit's filter code as well and compare its performance to mine.

### OpenMV
I had planned to use the [Pixy camera from Charmed Labs](https://pixycam.com/pixy-cmucam5/) as my traffic cone detection sensor in my previous Robo-Magellan attempt. Now there is a newer version of the [Pixy camera model (Pixy2)](https://pixycam.com/pixy2/) available. There is also a new competitor, the [OpenMV camera module](https://openmv.io), which has emerged on the market as well.

The latest [OpenMV Cam H7](https://openmv.io/collections/cams/products/openmv-cam-h7) has some pretty impressive features:
* It uses a pretty powerful ARM Cortex-M7 microcontroller:
  * STM32H743VI
  * 480 MHz
  * 1MB of RAM
  * 2MB of FLASH
* It offers a few different camera and lens options which is pretty darn cool:
  * [Global shutter](https://openmv.io/collections/cams/products/global-shutter-camera-module) - Might better handle robot motion.
  * [FLIR Lepton  Thermal Imager](https://openmv.io/collections/cams/products/flir-lepton-adapter-module)
  * [Wide Angle Lens](https://openmv.io/collections/lenses/products/ultra-wide-angle-lens) - Would give robot a >180 degree FOV for finding the traffic cone at the cost of fewer pixels per degree of view (ie. fewer pixels belonging to the cone to be detected).
  * [Polarizing filter](https://openmv.io/collections/lenses/products/polarizing-filter) - Could improve traffic cone detection in outdoor sunlight.
* The Eagle based schematics and layout are available on [GitHub](https://github.com/openmv/openmv-boards/tree/master/openmv4/base) which would make hacking easier.
* All of the low level C/C++ source code appears to be available on [GitHub](https://github.com/openmv/openmv/tree/master/src).
  * Makefiles and GCC for the win!
* Looks like it supports all of the algorithms supported by the PixyCam and more.
* Development on this project is still very active.

I definitely want to order ones of these and start experimenting with it.

### Next Steps
* The BLE debugging prototype that I built shows promise. I want to continue development on it this week and make some robustness updates.
* Write mbed driver for the LewanSoul LX-16A servos.
* Order an [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002](https://www.adafruit.com/product/3463).
* Order an [OpenMV Cam H7](https://www.sparkfun.com/products/15325).



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
* When driving along, it functions like a 4-wheel ackerman steering rig, where the point of rotation always falls outside of the robot's chassis.
<br><img src="photos/20190711-01.jpg" alt="Rover Driving Along" width="160" height="142"/>

* Once stopped, it can rotate all 4 outer wheels to point their axles towards the center of the robot chassis. This allows it to turn in place like a differential wheeled robot.
<br><img src="photos/20190711-02.jpg" alt="Rover Turning in Place" width="160" height="145"/>

* Since all 6 wheels are driven to achieve the best traction possible, the code needs to calculate the relative turning radii for all wheels and drive each wheel at the correct rate to properly turn around the desired point of rotation.

### Sawppy Rover Parts Ordered
I used the [Sawppy Rover documentation](https://github.com/Roger-random/Sawppy_Rover/tree/master/docs#readme) to finalize a list of parts and tools that I needed to order. I had been looking for an excuse to purchase a [Prusa i3](https://shop.prusa3d.com/en/3d-printers/181-original-prusa-i3-mk3-3d-printer.html) for sometime now. This 3D printer has consistently received good reviews in Make magazine and elsewhere. It is also the printer used at the local library's makerspace where it has worked reliably under heavy use. It's also nice to have someone local who is experienced with the same printer should I need help with it. I have just been waiting for an excuse to purchase one and this project gives me that excuse:)

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
* Hacksaw
* Soldering iron for threaded heat set inserts
* Metric Allen (hex) wrenches

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



## nRF51 BLE to UART Firmware Build
I have only tested the build on macOS but it might work on other Posix operating systems too.

**Note:** Building the firmware and deploying it to the nRF51 requires the **nrfjprog** tool. The archive for your OS can be downloaded from [here](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF5-Command-Line-Tools). You will also need a SEGGER J-Link JTAG/SWD Debugger. If your projects are of a non-commercial nature, you can use this [low cost module available from Adafruit](https://www.adafruit.com/product/3571).

* `make sdk` will download and install the required Nordic SDK. This should only need to be done once.
* `make flash_softdevice` will install the required Nordic SoftDevice on the nRF51422 microcontroller using the J-Link debugger. This will typically only need to be done once for each microcontroller.
* `make all` will build the hat firmware.
* `make flash` will build the hat firmware and deploy it using the J-Link debugger.

