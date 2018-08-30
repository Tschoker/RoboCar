# RoboCar
The principle idea is to have some kind of self steering model car that can perform several tasks (or to play around adding/removing functionalities).
The basic version will consist of the Controller (an Arduino Mega), a H-Bridge for the motors, an LCD to display some basic feedback, Ultrasonc distance sensors (HC-SR04) for collision prevention and some fancy white and red LED for front and back lights.

# Feature Brainstorm
* Collision detection using Ultrasonic sensors
* remote control using IR and/or WiFi
* NerfGun launcher
* Area Mapping of driven path
* camera stream via WiFi
* store and automatically navigate to specified places
* charging dock
* staircase detector (a sensor pointing down to detect if the floor is still there)
* speaker (to play horn sounds or other feedback)
* grip arm (to grip and move objects)
* GPS (to navigate given directions or to find home)
* IR-Sender (to drive up to a given receiver and send a signal - imagine your robot car activates the air condition and TV for you when you come home late)
* IR receiver (to send comands to the car)
* LoRa interface (for long distance communication to a controller - only makes sense when you equipp your car outdoor friendly)

# Next Steps
+ Mount motors and Ultrasonic sensors using 3D printed parts
* implement buzzer to give acoustic signals
* Move ultrasonic sensors from the side to the front so then there is 1 central and 2 pointing in the direction of the diagonal of the car

# Parts
* 1x Arduino Mega
* 4x Motors with fitted wheels
* 1x H-Bridge
* 1x 4x16 LCD
* 4x Ultrasonic sensors
* 2x white LED + resistors
* 2x red LED + resistors
* Battery
* Power switch
* wooden plank (I used ~20x30cm)

# Schematic
coming soon

# Images
![Prototype](/images/prototype1.jpg)

![Prototype](/images/prototype2.jpg)

![Prototype](/images/prototype3.jpg)
