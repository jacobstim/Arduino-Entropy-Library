// Entropy - A entropy (random number) generator for the Arduino
//   The latest version of this library will always be stored in the following
//   google code repository:
//     http://code.google.com/p/avr-hardware-random-number-generation/source/browse/#git%2FEntropy
//   with more information available on the libraries wiki page
//     http://code.google.com/p/avr-hardware-random-number-generation/wiki/WikiAVRentropy
//
// Copyright 2014 by Walter Anderson
//
// This file is part of Entropy, an Arduino library.
// Entropy is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Entropy is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Entropy.  If not, see <http://www.gnu.org/licenses/>.
//
// ---------------------------------------------------------------------------
// Note:
//  - uses TRNG hardware module on Arduino Due (SAM3X8E)
//  - uses jitter between main CPU oscillator and timer oscillator (32.768kHz)
//    on Arduino AVR and SAMD21
// ---------------------------------------------------------------------------
// Additions by Tim Jacobs: SAMD21 compatibility for Arduino Zero/MKR1000/MKR1200/...
// and similar boards (Adafruit Feather M0, Sparkfun SAMD21 Dev board, ...)


#include "Entropy.h"

const uint8_t WDT_MAX_8INT=0xFF;
const uint16_t WDT_MAX_16INT=0xFFFF;
const uint32_t WDT_MAX_32INT=0xFFFFFFFF;
// Since the Due TRNG is so fast we don't need a circular buffer for it
#ifndef ARDUINO_SAM_DUE
 const uint8_t gWDT_buffer_SIZE=32;
 const uint8_t WDT_POOL_SIZE=8;
 uint8_t gWDT_buffer[gWDT_buffer_SIZE];
 uint8_t gWDT_buffer_position;
 uint8_t gWDT_loop_counter;
 volatile uint8_t gWDT_pool_start;
 volatile uint8_t gWDT_pool_end;
 volatile uint8_t gWDT_pool_count;
 volatile uint32_t gWDT_entropy_pool[WDT_POOL_SIZE];
#endif

volatile uint32_t _tickcounter;


// This function initializes the global variables needed to implement the circular entropy pool and
// the buffer that holds the raw Timer 1 values that are used to create the entropy pool.  It then
// Initializes the Watch Dog Timer (WDT) to perform an interrupt 
//  - AVR hardware: every 2048 clock cycles, (about 16 ms) which is as fast as it can be set.
//  - SAMD21 hardware: every 1ms (1kHz clock)
//
void EntropyClass::initialize(void)
{
#ifndef ARDUINO_SAM_DUE
  gWDT_buffer_position=0;
  gWDT_pool_start = 0;
  gWDT_pool_end = 0;
  gWDT_pool_count = 0;
#endif
#if defined(__AVR__)
  cli();                         // Temporarily turn off interrupts, until WDT configured
  MCUSR = 0;                     // Use the MCU status register to reset flags for WDR, BOR, EXTR, and POWR
  _WD_CONTROL_REG |= (1<<_WD_CHANGE_BIT) | (1<<WDE);
  // WDTCSR |= _BV(WDCE) | _BV(WDE);// WDT control register, This sets the Watchdog Change Enable (WDCE) flag, which is  needed to set the
  _WD_CONTROL_REG = _BV(WDIE);            // Watchdog system reset (WDE) enable and the Watchdog interrupt enable (WDIE)
  sei();                         // Turn interupts on
#elif defined(ARDUINO_SAM_DUE)
  pmc_enable_periph_clk(ID_TRNG);
  TRNG->TRNG_IDR = 0xFFFFFFFF;
  TRNG->TRNG_CR = TRNG_CR_KEY(0x524e47) | TRNG_CR_ENABLE;
#elif defined(__arm__) && defined(TEENSYDUINO)
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER;
  LPTMR0_CSR = 0b10000100;
  LPTMR0_PSR = 0b00000101;  // PCS=01 : 1 kHz clock
  LPTMR0_CMR = 0x0006;      // smaller number = faster random numbers...
  LPTMR0_CSR = 0b01000101;
  NVIC_ENABLE_IRQ(IRQ_LPTMR);
#elif defined(ARDUINO_ARCH_SAMD)
  _tickcounter = 0;
  /************************************************************************************************
    We are going to configure a timer on the SAMD21, with the following properties:
     - Clock Source: low power 32KHz oscillator 
     - Count up until value 1 is exceeded
     - Clock Divider: 16, so the counter overflows every 1/1024s
       => Once the counter reaches 2, we overflow, so the effective clock divider is 32
          (two counts per interrupt).
    This means that approximately every 1ms, our timer interrupt routine will be called; since
    our main clockfrequency is 48MHz, we expect SYSTICK to increase by 1464 at every 32KHz 
    oscillator tick (48.000.000/32.768), or with 1.500.000 every 1024 32KHz oscillator ticks. 
    Due to the thermal influences on both crystals, this number will be slightly different every 
    time, and that will be our source of randomness. 

    Some inspiration from: https://github.com/nebs/arduino-zero-timer-demo/blob/master/src/main.cpp
   ************************************************************************************************/
  // We choose a general clock generator... let's take number 2, and configure divisor = 16.
  // Since the actual divider is calculated as (2^(DIV+1)), we take DIV = 3, so we have 2^4 = 16 
  // as actual divider.
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(3);
  // Enable clock generator 2 (GCLK2) using low-power 32KHz oscillator.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |                  // Clock generator ID 2
                      GCLK_GENCTRL_GENEN |                  // Enable generator
                      GCLK_GENCTRL_SRC_OSCULP32K |          // Use low power 32KHz oscillator as source
                      GCLK_GENCTRL_DIVSEL;                  // The generic clock generator equals the clock source divided by 2^(GENDIV.DIV+1)
  // Wait for changes to process
  while(GCLK->STATUS.bit.SYNCBUSY);

  // Feed GCLK2 to TC4 (and TC5, they work together)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC4_TC5 |             // Control Timer4 and Timer5
                      GCLK_CLKCTRL_CLKEN |                  // Enable the coupling
                      GCLK_CLKCTRL_GEN_GCLK2;               // Couple GCLK2 to TC4

  // Wait for changes to process
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Configure Timer 4
  TcCount16* TC = (TcCount16*) TC4;
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;                        // Disable Timer4 if running
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;                   // Use the 16-bit timer
  while (TC->STATUS.bit.SYNCBUSY == 1);
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;                   // Count up until the configured value
  while (TC->STATUS.bit.SYNCBUSY == 1);
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;                 // Set prescaler to 1

  // Initialize Timer 4
  TC->COUNT.reg = 0;                                        // Set starting count value 
  TC->CC[0].reg = 1;                                        // Count up to this value (at a rate of 1 per clock tick = 1 per 1/1024 of a second)
  while (TC->STATUS.bit.SYNCBUSY == 1);
  TC->INTENSET.reg = 0;                                     
  TC->INTENSET.bit.MC0 = 1;                                 // Enable compare with CC[0]

  // Enable the TC4 interrupt function
  NVIC_DisableIRQ(TC4_IRQn);                                // Disable existing interrupt
  NVIC_ClearPendingIRQ(TC4_IRQn);                           // Clear pending interrupts
  NVIC_SetPriority(TC4_IRQn, 0);                            // Give TC4 the top priority in the Nested Vector Interrupt Controller (NVIC)
  NVIC_EnableIRQ(TC4_IRQn);                                 // Connect TC4 to the NVIC

  // Enable counter
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;                         // Enabler timer again
  while (TC->STATUS.bit.SYNCBUSY == 1);

#endif
}


// This function returns the current number of clockticks by our external reference
// clock. You might find it useful for various reasons.
// NOTE: Does not work on Arduino Due, it uses a TRNG instead of a timer implementation
uint32_t EntropyClass::getTicks() {
  return _tickcounter;
}

// This function returns a uniformly distributed random integer in the range
// of [0,0xFFFFFFFF] as long as some entropy exists in the pool and a 0
// otherwise.  To ensure a proper random return the available() function
// should be called first to ensure that entropy exists.
//
// The pool is implemented as an 8 value circular buffer
uint32_t EntropyClass::random(void)
{
  uint32_t retVal = 0;
#if defined(ARDUINO_SAM_DUE)
  while (! (TRNG->TRNG_ISR & TRNG_ISR_DATRDY))
    ;
  retVal = TRNG->TRNG_ODATA;
#elif defined(__SAMD21G18A__)
  uint8_t waiting = 0;
  // Wait at most 1 second...
  while ((gWDT_pool_count < 1) && (waiting < 100)) {
    waiting += 1;
    delay(10);
  }

  // Did we time out in the above loop?
  if (waiting < 100) {
    // The following code needs to be executed atomically, i.e. without our timer interrupt interfering!
    noInterrupts();             // platform independent Arduino framework implementation
    retVal = gWDT_entropy_pool[gWDT_pool_start];
    gWDT_pool_start = (gWDT_pool_start + 1) % WDT_POOL_SIZE;
    --gWDT_pool_count;
    interrupts();               // platform independent Arduino framework implementation
  }

#else
  uint8_t waiting;
  while (gWDT_pool_count < 1)
    waiting += 1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    retVal = gWDT_entropy_pool[gWDT_pool_start];
    gWDT_pool_start = (gWDT_pool_start + 1) % WDT_POOL_SIZE;
    --gWDT_pool_count;
  }
#endif
  return retVal;
}

// This function returns one byte of a single 32-bit entropy value, while preserving the remaining bytes to
// be returned upon successive calls to the method.  This makes best use of the available entropy pool when
// only bytes size chunks of entropy are needed.  Not available to public use since there is a method of using
// the default random method for the end-user to achieve the same results.  This internal method is for providing
// that capability to the random method, shown below
uint8_t EntropyClass::random8(void)
{
  static uint8_t byte_position=0;
  uint8_t retVal8;

  if (byte_position == 0)
    share_entropy.int32 = random();
  retVal8 = share_entropy.int8[byte_position++];
  byte_position = byte_position % 4;
  return(retVal8);
}

uint8_t EntropyClass::randomByte(void)
{
  return random8();
}

// This function returns one word of a single 32-bit entropy value, while preserving the remaining word to
// be returned upon successive calls to the method.  This makes best use of the available entropy pool when
// only word sized chunks of entropy are needed.  Not available to public use since there is a method of using
// the default random method for the end-user to achieve the same results.  This internal method is for providing
// that capability to the random method, shown below
uint16_t EntropyClass::random16(void)
{
  static uint8_t word_position=0;
  uint16_t retVal16;

  if (word_position == 0)
    share_entropy.int32 = random();
  retVal16 = share_entropy.int16[word_position++];
  word_position = word_position % 2;
  return(retVal16);
}

uint16_t EntropyClass::randomWord(void)
{
  return random16();
}

// This function returns a uniformly distributed integer in the range of
// of [0,max).  The added complexity of this function is required to ensure
// a uniform distribution since the naive modulus max (% max) introduces
// bias for all values of max that are not powers of two.
//
// The loops below are needed, because there is a small and non-uniform chance
// That the division below will yield an answer = max, so we just get
// the next random value until answer < max.  Which prevents the introduction
// of bias caused by the division process.  This is why we can't use the
// simpler modulus operation which introduces significant bias for divisors
// that aren't a power of two
uint32_t EntropyClass::random(uint32_t max)
{
  uint32_t slice;

  if (max < 2)
    retVal=0;
  else
    {
      retVal = WDT_MAX_32INT;
      if (max <= WDT_MAX_8INT) // If only byte values are needed, make best use of entropy
      {                      // by diving the long into four bytes and using individually
        slice = WDT_MAX_8INT / max;
        while (retVal >= max)
          retVal = random8() / slice;
      }
      else if (max <= WDT_MAX_16INT) // If only word values are need, make best use of entropy
      {                            // by diving the long into two words and using individually
        slice = WDT_MAX_16INT / max;
        while (retVal >= max)
          retVal = random16() / slice;
      }
      else
      {
        slice = WDT_MAX_32INT / max;
        while (retVal >= max)
          retVal = random() / slice;
      }
    }
  return(retVal);
}

// This function returns a uniformly distributed integer in the range of
// of [min,max).
uint32_t EntropyClass::random(uint32_t min, uint32_t max)
{
  uint32_t tmp_random, tmax;

  tmax = max - min;
  if (tmax < 1)
    retVal=min;
  else
    {
      tmp_random = random(tmax);
      retVal = min + tmp_random;
    }
  return(retVal);
}

// This function returns a uniformly distributed single precision floating point
// in the range of [0.0,1.0)
float EntropyClass::randomf(void)
{
  float fRetVal;

  // Since c++ doesn't allow bit manipulations of floating point types, we are
  // using integer type and arrange its bit pattern to follow the IEEE754 bit
  // pattern for single precision floating point value in the range of 1.0 - 2.0
  uint32_t tmp_random = random();
  tmp_random = (tmp_random & 0x007FFFFF) | 0x3F800000;
  // We then copy that binary representation from the temporary integer to the
  // returned floating point value
  memcpy((void *) &fRetVal, (void *) &tmp_random, sizeof(fRetVal));
  // Now translate the value back to its intended range by subtracting 1.0
  fRetVal = fRetVal - 1.0;
  return (fRetVal);
}

// This function returns a uniformly distributed single precision floating point
// in the range of [0.0, max)
float EntropyClass::randomf(float max)
{
  float fRetVal;
  fRetVal = randomf() * max;
  return(fRetVal);
}

// This function returns a uniformly distributed single precision floating point
// in the range of [min, max)
float EntropyClass::randomf(float min,float max)
{
  float fRetVal;
  float tmax;
  tmax = max - min;
  fRetVal = (randomf() * tmax) + min;
  return(fRetVal);
}

// This function implements the Marsaglia polar method of converting a uniformly
// distributed random numbers to a normaly distributed (bell curve) with the
// mean and standard deviation specified.  This type of random number is useful
// for a variety of purposes, like Monte Carlo simulations.
float EntropyClass::rnorm(float mean, float stdDev)
{
  static float spare;
  static float u1;
  static float u2;
  static float s;
  static bool isSpareReady = false;

  if (isSpareReady)
  {
    isSpareReady = false;
    return ((spare * stdDev) + mean);
  } else {
    do {
      u1 = (randomf() * 2) - 1;
      u2 = (randomf() * 2) - 1;
      s = (u1 * u1) + (u2 * u2);
    } while (s >= 1.0);
    s = sqrt(-2.0 * log(s) / s);
    spare = u2 * s;
    isSpareReady = true;
    return(mean + (stdDev * u1 * s));
  }
}

// This function returns a unsigned char (8-bit) with the number of unsigned long values
// in the entropy pool
uint8_t EntropyClass::available(void)
{
#ifdef ARDUINO_SAM_DUE
  return(TRNG->TRNG_ISR & TRNG_ISR_DATRDY);
#else
  return(gWDT_pool_count);
#endif
}

// Circular buffer is not needed with the speed of the Arduino Due trng hardware generator
#ifndef ARDUINO_SAM_DUE
// This interrupt service routine is called every time the WDT interrupt is triggered.
// With the default configuration that is approximately once every 16ms, producing
// approximately two 32-bit integer values every second.
//
// The pool is implemented as an 8 value circular buffer
static void isr_hardware_neutral(uint8_t val)
{
  gWDT_buffer[gWDT_buffer_position] = val;
  gWDT_buffer_position++;                     // every time the WDT interrupt is triggered
  if (gWDT_buffer_position >= gWDT_buffer_SIZE)
  {
    gWDT_pool_end = (gWDT_pool_start + gWDT_pool_count) % WDT_POOL_SIZE;
    // The following code is an implementation of Jenkin's one at a time hash
    // This hash function has had preliminary testing to verify that it
    // produces reasonably uniform random results when using WDT jitter
    // on a variety of Arduino platforms
    for(gWDT_loop_counter = 0; gWDT_loop_counter < gWDT_buffer_SIZE; ++gWDT_loop_counter)
      {
	gWDT_entropy_pool[gWDT_pool_end] += gWDT_buffer[gWDT_loop_counter];
	gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 10);
	gWDT_entropy_pool[gWDT_pool_end] ^= (gWDT_entropy_pool[gWDT_pool_end] >> 6);
      }
    gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 3);
    gWDT_entropy_pool[gWDT_pool_end] ^= (gWDT_entropy_pool[gWDT_pool_end] >> 11);
    gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 15);
    gWDT_entropy_pool[gWDT_pool_end] = gWDT_entropy_pool[gWDT_pool_end];
    gWDT_buffer_position = 0; // Start collecting the next 32 bytes of Timer 1 counts
    if (gWDT_pool_count == WDT_POOL_SIZE) // The entropy pool is full
      gWDT_pool_start = (gWDT_pool_start + 1) % WDT_POOL_SIZE;
    else // Add another unsigned long (32 bits) to the entropy pool
      ++gWDT_pool_count;
  }
}
#endif

#if defined( __AVR_ATtiny25__ ) || defined( __AVR_ATtiny45__ ) || defined( __AVR_ATtiny85__ )
ISR(WDT_vect)
{
 _tickcounter++;
 isr_hardware_neutral(TCNT0);
}

#elif defined(__AVR__)
ISR(WDT_vect)
{
  _tickcounter++;
  isr_hardware_neutral(TCNT1L); // Record the Timer 1 low byte (only one needed)
}

#elif defined(__arm__) && defined(TEENSYDUINO)
void lptmr_isr(void)
{
  _tickcounter++;
  LPTMR0_CSR = 0b10000100;
  LPTMR0_CSR = 0b01000101;
  isr_hardware_neutral(SYST_CVR);
}
#elif defined(__SAMD21G18A__)
// Interrupt Service Routine (ISR) for timer TC4
void TC4_Handler()                              
{     
  TcCount16* TC = (TcCount16*) TC4;
  if (TC->INTFLAG.bit.MC0 == 1) {
    // Our tick counter for reference purposes
    _tickcounter++;

    // Timer reached max count, store current SYSTICK value for randomness
    isr_hardware_neutral(SysTick->VAL);

    // Writing a 1 again clears the flag interrupt
    TC->INTFLAG.bit.MC0 = 1;
   }
}
#endif

// The library implements a single global instance.  There is no need, nor will the library
// work properly if multiple instances are created.
EntropyClass Entropy;
