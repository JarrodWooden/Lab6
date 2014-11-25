Lab6
====

Make a Robot Move using the MSP430 and pulse Modulation... For experts, make robot move based on button presses from an IR Remote


#Pre Lab

This is a picture of the general idea for the Hardward Setup for the Robot:

![alt text](https://raw.githubusercontent.com/JarrodWooden/Lab6/master/HardwarePrelab.jpg "Hardware Design for the Robot")

The only difference to the hardware design is that I decided to use two pins for the PWM signal to the robot motor driver and two GPIO pins to set as either Voltage High or Voltage Low depending on whether or not I want the Robot to move Forwards or Backwards.

Here is the general idea for software design:

![alt text](https://raw.githubusercontent.com/JarrodWooden/Lab6/master/SoftwarePrelab.jpg "Software Design for the Robot")

This information was recieved from this url: www.hobbytronics.co.uk/h-bridge-driver-sn754410

Because the datasheet was not very helpful with a general overview on how it actually works and should connect to our robot and MSP430

Here is the picture of the motor driver with all the connections!:

![alt text](https://raw.githubusercontent.com/JarrodWooden/Lab6/master/sn754410-connections.jpg "SN754410 Motor Driver Connections")

"H-Bridge Motor Driver 1A - SN754410

Faster, cheaper, smaller, better, right? The SN754410 Quad Half H-Bridge is just that. Capable of driving high voltage motors using TTL 5V logic levels, the SN754410 can drive 4.5V up to 36V at 1A continuous output current!

For even higher current applications, it is possible to physically stack two devices on top of each other to get almost 2 A of drive current.

The SN754410 is a quad half H-bridge IC. This allows the chip to either control 4 motors in one direction using the 4 half H-bridges or to control 2 motors in both directions using a full H-bridge for each motor.

The following shows the connections for controlling 2 motors in either direction using 2 full H-bridges."
