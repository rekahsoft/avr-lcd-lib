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
 * File: lcdOutput.c
 * Author: Collin J. Doering <collin.doering@rekahsoft.ca>
 * Date: Sep 17, 2015
 */

/*---------.
| Includes |
`---------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>

#include "lcdLib.h"

int main(void) {
  clock_prescale_set(clock_div_1);
  
  STATUS_LED_DDR |= 1 << STATUS_LED; // DEBUG

  initLCD();
  //quickInitLCD();
  
  while (1) {
    writeCharToLCD('H');
    writeCharToLCD('e');
    writeCharToLCD('l');
    writeCharToLCD('l');
    writeCharToLCD('o');
    writeCharToLCD(' ');
    writeCharToLCD('t');
    writeCharToLCD('h');
    writeCharToLCD('e');
    writeCharToLCD('r');
    writeCharToLCD('e');
    writeCharToLCD(' ');
    writeCharToLCD('f');
    writeCharToLCD('r');
    writeCharToLCD('i');
    writeCharToLCD('e');
    writeCharToLCD('n');
    writeCharToLCD('d');
    writeCharToLCD('!');

    //flashLED(5); // DEBUG
    _delay_ms(5000);
  }

  return 0;
}
