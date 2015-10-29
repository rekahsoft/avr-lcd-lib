/**
 * (C) Copyright Collin J. Doering 2015
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * File: digital_thermometer.c
 * Author: Collin J. Doering <collin.doering@rekahsoft.ca>
 * Date: Oct 28, 2015
 */

/*---------.
| Includes |
`---------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <stdlib.h>

#include "lcdLib.h"
#include "ansi_escapes.h"

void initADC(void) {
  ADMUX  |= (1 << REFS0);                // reference voltage on AVCC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // ADC prescaler 64

  ADCSRA |= (1 << ADEN);                 // Enable ADC
  ADCSRA |= (1 << ADATE);                // Auto trigger enable
  ADCSRA |= (1 << ADSC);                 // Start first conversion
}

int main(void) {
  clock_prescale_set(clock_div_1);

  initLCD();
  initADC();

  hideCursor();
  writeStringToLCD("Temperature: ");
  saveCursorPosition();

  while (1) {
    float v = ADC * (5000 / 1024);
    int8_t t = (int8_t) ((v - 500) / 10);

    char str[5];
    uint8_t i = 1, sigDig = 0;

    if (t < 0) {
      str[0] = '-';
      t *= -1;
    } else {
      str[0] = '+';
    }

    for (uint8_t j = 100; j >= 1; j /= 10) {
      uint8_t digit = (uint8_t) (t / j);
      if (!sigDig) {
        if (digit != 0) {
          sigDig = 1;
          str[i++] = digit + 0x30;
        } else {
          str[i++] = ' ';
        }
      } else {
        str[i++] = digit + 0x30;
      }

      t = t - digit*j;
    }
    str[i] = '\0';

    writeStringToLCD(str);
    writeCharToLCD(0xdf);
    writeStringToLCD("C");

    restoreCursorPosition();
    _delay_ms(16);
  }

  return 0;
}
