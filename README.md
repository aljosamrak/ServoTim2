ServoTim2
===============

ServoTim2 is an Arduino library that allows to controll up to 8 servos using Timer2. Sorvos move asynchronously. You can set position, speed of the servo and optionally wait until the servo reaches target position.

The ServoTim2 is based on [nabontra/ServoTimer2](https://github.com/nabontra/ServoTimer2) and [netlabtoolkit/VarSpeedServo](https://github.com/netlabtoolkit/VarSpeedServo) libraries.

* Uses Timer2
* Supports up to 8 servos
* Can set servo speed
* `writeBlocking()` function waits until servo reaches target position

Sample Code
----------------------------

```
#include <ServoTim2.h>

ServoTim2 servo;                    // create servo object to control a servo
int pos = 0;                        // variable to store the servo position

void setup()
{
    // Attaches the servo on pin 9 to the servo object, min pulse width to 500 us
    // max pulse width to 2000 us and sets the declared speed of the motor to 120 ms/60°
    servo.attach(9, 500, 2000, 120);
}

void loop()
{
    servo.writeBlocking(0, 20);         // goes to 0 degrees and wait to get there
    servo.writeBlocking(180, 20);       // goes to 180 degrees and wait to get there
}
```

[Installation](http://arduino.cc/en/Guide/Libraries)
=============

* Download the .zip file GitHub
* In Arduino, select SKETCH>IMPORT LIBRARY...>ADD LIBRARY... and select the .zip file
