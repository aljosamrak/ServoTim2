// Servo moves from 0 degrees to 180 degrees and back.
// Same as Sweep example without delay

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
    servo.writeBlocking(0);         // goes to 0 degrees and wait to get there
    servo.writeBlocking(180);       // goes to 180 degrees and wait to get there
}
