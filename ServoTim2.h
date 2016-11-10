/*
    ServoTim2.h - Interrupt driven Servo library for Arduino using Timer2- Version 1.0
    Copyright (c) 2009 Michael Margolis.  All right reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
    This library uses Timer2 to drive up to 8 servos using interrupts so no refresh activity is required from within the sketch.
    The usage and method naming is similar to the Arduino software servo library http://www.arduino.cc/playground/ComponentLib/Servo

    A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
    The servo is pulsed in the background to the value most recently written using the write() method.

    Note that analogWrite of PWM on pins 3 and 11 is disabled when the first servo is attached.
*/


#ifndef ServoTim2_h
#define ServoTim2_h

#include <inttypes.h>

#define MAX_SERVOS             8        // the maximum number of channels, don't change this
#define INVALID_SERVO         -1        // invalid servo number
#define DELAY_ADJUST          65        // number of microseconds of calculation overhead to be subtracted from pulse timings
#define MIN_PULSE_WIDTH      500        // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH     2000        // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH 1500        // default pulse width when servo is attached
#define SIGNAL_PERIOD      20000        // signal period
#define TICKS_FACTOR         128        // timer interrupt length in milliseconds
#define DEFAULT_SERVO_SPEED  255        // default servo speed in milliseconds per 60 degrees

#define US2TICKS(_us)       ((clockCyclesPerMicrosecond()* (_us)) / 8)          // Converts microseconds to tick
#define TICKS2US(_tick)     (((_tick) * 8) / clockCyclesPerMicrosecond())       // Converts ticks to microseconds
// Enables to write into two consecutive 8 bit variable simultaneously as one uint16_t
#define B2S(_v)             (*((uint16_t*)&(_v)))
// Calculates pulse width representing a 60 degrees angle
#define PULSE_WIDTH_60_DEG  (US2TICKS(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 3)

typedef struct {
    uint8_t pin        :5;              // a pin number from 0 to 31
    uint8_t isActive   :1;              // false if this channel not enabled, pin only pulsed if true
    uint8_t remainder;                  // with counter represents 16 bit counter
    uint8_t counter;
    uint16_t target;                    // used for slowly move servo
    uint8_t speed;                      // current speed for moving servo
    uint8_t declaredSpeed;              // declared speed of servo
} servo_t;

class ServoTim2 {

    public:
        /**
         * Constructor allocate next available channel.
         */
	    ServoTim2();

        /**
         * Attach the given pin to the next free channel and sets pinMode.
         * @param pin Pin number to which the servo it connected to.
         * @param min Minimal values for writes.
         * @param max Maximal values for writes.
         * @param speed Declared speed of the servo.
         * @return Returns channel number or -1 if failure.
         */
	    uint8_t attach(uint8_t pin);
	    uint8_t attach(uint8_t pin, uint16_t min, uint16_t max);
	    uint8_t attach(uint8_t pin, uint16_t min, uint16_t max, uint8_t speed);

        /**
         * Marks servo as detached.
         */
        void detach();

        /**
         * Store the pulse width. If value is less than MIN_PULSE_WIDTH it
         * is treated as an angle otherwise as a pulse width in microseconds.
         * @param value Pulse width in microseconds or an angle in degrees.
         * @param speed Speed of the servo between 0 and 255. 1 - slowest, 255 - max speed, 0 as write(value).
         */
        void write(uint16_t value);
        void write(uint16_t value, uint8_t speed);

        /**
         * Same as write(value, speed) and waits for servo to reach target angle.
         * @param value Pulse width in microseconds or an angle in degrees.
         * @param speed Speed of the servo between 0 and 255. 1 - slowest, 255 - max speed, 0 as write(value).
         */
        void writeBlocking(uint16_t value);
        void writeBlocking(uint16_t value, uint8_t speed);

        /**
         * Returns current pulse width as an angle in degrees.
         * @return Returns current pulse width as an angle in degrees.
         */
        uint16_t read();

        /**
         * Returns current pulse width in microseconds.
         * @return Returns current pulse width in microseconds.
         */
        uint16_t readMicroseconds();

        /**
         * Return true if this servo is attached.
         * @return Return true if this servo is attached.
         */
	    uint8_t isAttached();

    private:
        /**
         * Handles writing the correct pulse width.
         * @return Returns current pulse width in microseconds.
         * @param value Pulse width in microseconds or an angle in degrees.
         * @param speed Speed of the servo between 0 and 255. 1 - slowest, 255 - max speed, 0 as write(value).
         * @param blocking Indicates to wait for servo to reach target angle or not.
         */
        void writeMicroseconds(uint16_t value, uint8_t speed, uint8_t blocking);

        uint8_t servoIndex;             // servo id
        uint8_t declaredSpeed;          // declared speed of servo per 60 degrees in milliseconds
        uint16_t min;                   // minimum pulse width in microseconds
        uint16_t max;                   // maximum pulse width in microseconds
};

#endif
