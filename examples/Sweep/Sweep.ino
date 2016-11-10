// Servo moves from 0 degrees to 180 degrees and back

#include <ServoTim2.h>

ServoTim2 servo;                        // create servo object to control a servo
int pos = 0;                            // variable to store the servo position

void setup()
{
    servo.attach(9);                    // attaches the servo on pin 9 to the servo object
}

void loop()
{
    for(pos = 0; pos < 180; ++pos)      // goes from 0 degrees to 180 degrees
    {                                   // in steps of 1 degree
        servo.write(pos);               // tell servo to go to position in variable 'pos'
        delay(15);                      // waits 15ms for the servo to reach the position
    }
    for(pos = 180; pos>=1; --pos)       // goes from 180 degrees to 0 degrees
    {
        servo.write(pos);               // tell servo to go to position in variable 'pos'
        delay(15);                      // waits 15ms for the servo to reach the position
    }
}
