# Arduino-Entropy-Library
An attempt to provide a source of randomness to Arduino based devices. This library uses:
 * The hardware TrueRNG of the SAM3X8E when being compiled on the Arduino Due and friends;
 * The presumably random discrepancies between the main MCU clock crystal (running at 48MHz for the Arduino Zero/MKR1000) and the low power 32KHz crystal
as a source of randomness.

## History and credits
Original code: copyright 2014 by Walter Anderson, see [here](https://code.google.com/p/avr-hardware-random-number-generation). Please read Walter's original study on the subject [at his site](https://sites.google.com/site/astudyofentropy/project-definition/timer-jitter-entropy-sources/entropy-library).
Github fork from Pascal de Bruijn, see [here](https://github.com/pmjdebruijn/Arduino-Entropy-Library), who seems to have cloned the v1.0.2 of Walter's library

