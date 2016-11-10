/*
    ServoTim2.cpp - Interrupt driven Servo library for Arduino using Timer2- Version 1.0
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

extern "C" {
    // AVR LibC Includes
    #include <inttypes.h>
    #include <avr/interrupt.h>
}

#include <Arduino.h>
#include "ServoTim2.h"

static servo_t servos[MAX_SERVOS + 1];                          // static array holding servo data for all servos
static volatile uint8_t currentServo = 0;                       // counter holding id of the curent servo being pulsed
static volatile uint8_t ISRCount = 0;                           // iteration counter used in the interrupt routines;
static uint8_t servoCount = 0;                                  // number of attached servos
static uint16_t remaininPeriodTicks = US2TICKS(SIGNAL_PERIOD);  // counting ticks to the end of the 20ms period

static void initISR() {
    TIMSK2 = 0;                                                 // disable interrupts
    TCCR2A = 0;                                                 // normal counting mode
    TCCR2B = _BV(CS21);                                         // set prescaler of 8
    TCNT2 = 0;                                                  // clear the timer count
    TIFR2 = _BV(TOV2);                                          // clear pending interrupts;
    TIMSK2 = _BV(TOIE2);                                        // enable the overflow interrupt
}

static void finISR() {
    TIMSK2 &= ~_BV(TOIE2);                                      // disable the overflow interrupt
}

static uint8_t isTimerActive() {
    for (uint8_t channel = 1; channel <= MAX_SERVOS; channel++) {
        if (servos[channel].isActive)
            return true;
    }
    return false;
}

ISR (TIMER2_OVF_vect) {
    ++ISRCount;                                                 // increment the overlflow counter

    if (ISRCount == servos[currentServo].counter) {             // set counter to count off the remaining ticks
        TCNT2 = 255 - (uint8_t)(servos[currentServo].remainder);
        if (currentServo > 0) {
            remaininPeriodTicks -= B2S(servos[currentServo].remainder);
        }
    } else if (ISRCount > servos[currentServo].counter) {       // real interrupt occurred
        if (servos[currentServo].isActive) {
            digitalWrite(servos[currentServo].pin, LOW);
        }

        // Find next active servo
        while (++currentServo <= servoCount && !servos[currentServo].isActive);

	    if (servos[currentServo].speed) {
	        // Direction is used insted of two if statements for adding and substructing speed
	        int16_t direction = servos[currentServo].target > B2S(servos[currentServo].remainder) ? 1 : -1;

	        B2S(servos[currentServo].remainder) += servos[currentServo].speed * direction;
		    if (servos[currentServo].target * direction <= B2S(servos[currentServo].remainder) * direction) {
		        B2S(servos[currentServo].remainder) = servos[currentServo].target;
			    servos[currentServo].speed = 0;
		    }
	    }

        ISRCount = 0;                                           // reset interoupt counter
        TCNT2 = 0;                                              // reset count register

        if (currentServo <= servoCount) {
            digitalWrite(servos[currentServo].pin, HIGH);       // pulse servo high
        } else {
            B2S(servos[0].remainder) = remaininPeriodTicks;     // set remaining ticks of 20ms period

            remaininPeriodTicks = US2TICKS(SIGNAL_PERIOD);      // reset remaining servo period ticks counter
            currentServo = 0;                                   // reset current servo
        }
    }
}

void ServoTim2::writeMicroseconds(uint16_t value, uint8_t speed, uint8_t blocking) {
    // Treat values less than MIN_PULSE_WIDTH as angles in degrees
    if (value < MIN_PULSE_WIDTH) {
        value = map(constrain(value, 0, 180), 0, 180, min,  max);
    } else {
        value = constrain(value, min, max);
    }
    value = US2TICKS(value - DELAY_ADJUST);                     // convert to ticks after compensating for interrupt overhead

    int16_t preVal = B2S(servos[this->servoIndex].remainder);   // get current position
    servos[this->servoIndex].speed = speed;                     // set speed

    // Write pulse width
    if (speed) {
        servos[this->servoIndex].target = value;
    } else {
        B2S(servos[this->servoIndex].remainder) = value;
    }

    // Wait for the servo to reach end position
    if (blocking) {
        int32_t difference = abs((int16_t)value - preVal);
        int32_t noSpeedDelay = (difference * declaredSpeed) / PULSE_WIDTH_60_DEG;

        if (speed) {
            delay(max((difference * SIGNAL_PERIOD) / 1000 / speed, noSpeedDelay));
        } else {
            delay(noSpeedDelay);
        }
    }
}

ServoTim2::ServoTim2() {
    // Assign a channel number to this instance
    servoIndex = servoCount <= MAX_SERVOS ? ++servoCount : INVALID_SERVO;
}

uint8_t ServoTim2::attach(uint8_t pin) {
    attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, DEFAULT_SERVO_SPEED);
}

uint8_t ServoTim2::attach(uint8_t pin, uint16_t min, uint16_t max) {
    attach(pin, min, max, DEFAULT_SERVO_SPEED);
}

uint8_t ServoTim2::attach(uint8_t pin, uint16_t min, uint16_t max, uint8_t speed) {
    if (this->servoIndex > 0) {
        pinMode(pin, OUTPUT);                                       // set servo pin to output
        servos[this->servoIndex].pin = pin;                         // store servo pin
        this->min = min > MIN_PULSE_WIDTH ? min : MIN_PULSE_WIDTH;  // store servo min pulse width
        this->max = max < MAX_PULSE_WIDTH ? max : MAX_PULSE_WIDTH;  // store servo max pulse width
        this->declaredSpeed = speed;                                // store declared servo speed

        if (isTimerActive() == false) {
            initISR();
        }
        servos[this->servoIndex].isActive = true;                   // set servo as active
    }
    return this->servoIndex;
}

boolean ServoTim2::isAttached() {
    return servos[this->servoIndex].isActive;
}

void ServoTim2::detach() {
    servos[this->servoIndex].isActive = false;
    if(isTimerActive() == false) {
        finISR();
    }
}

uint16_t ServoTim2::read() {
    return  map(readMicroseconds(), min, max, 0,  180);
}

uint16_t ServoTim2::readMicroseconds() {
    return TICKS2US(B2S(servos[this->servoIndex].remainder)) + DELAY_ADJUST;
}

void ServoTim2::write(uint16_t value) {
    writeMicroseconds(value, 0, 0);
}

void ServoTim2::write(uint16_t value, uint8_t speed) {
    writeMicroseconds(value, speed, 0);
}

void ServoTim2::writeBlocking(uint16_t value) {
    writeMicroseconds(value, 0, 1);
}

void ServoTim2::writeBlocking(uint16_t value, uint8_t speed) {
    writeMicroseconds(value, speed, 1);
}
