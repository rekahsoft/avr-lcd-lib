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
#include <stdlib.h>

#include "lcdLib.h"
#include "ansi_escapes.h"
#include "USART.h"

#define STATUS_LED_PORT PORTC
#define STATUS_LED_DDR  DDRC
#define STATUS_LED      PC5

//--------------------------------------------------

void flashLED(uint8_t times) {
  while (times > 0) {
    STATUS_LED_PORT |= 1 << STATUS_LED; // turn on status LED
    _delay_ms(100);
    STATUS_LED_PORT &= ~(1 << STATUS_LED); // turn status LED off
    _delay_ms(100);
    times--;
  }
}
//--------------------------------------------------

/*
  Initialize previous, current, and next screen variables to spaces
*/
char* initScreenMem(const uint8_t len) {
  char *scrn = (char*) malloc(len * sizeof(char));
  for (uint8_t i = 0; i < len; i++) {
    scrn[i] = ' ';
  }
  return scrn;
}

int main(void) {
  clock_prescale_set(clock_div_1);
  
  STATUS_LED_DDR |= 1 << STATUS_LED; // DEBUG

  // Allocate and init first block of heap for storing LCD data
  char* scrn = initScreenMem(LCD_CHARACTERS_PER_SCREEN);

  initUSART();
  char serialChar;

  initLCD();
  //initLCDByInternalReset();
  flashLED(5); // DEBUG


  writeStringToLCD("Backwards 4" CUB(4));
  _delay_ms(5000);

  clearDisplay();
  writeStringToLCD("Backwards 3" CUB(3));
  _delay_ms(5000);

    clearDisplay();
  writeStringToLCD("Backwards 2" CUB(2));
  _delay_ms(5000);

  clearDisplay();
  writeStringToLCD("Backwards 1" CUB(1));
  _delay_ms(5000);

  clearDisplay();
  writeStringToLCD("Forwards 4" CUF(4));
  _delay_ms(5000);

  /* writeStringToLCD(CNL(1)); */
  /* _delay_ms(5000); */

  /* writeStringToLCD(CUF(10)); */
  /* _delay_ms(5000); */

  /* writeStringToLCD(CPL(1)); */
  /* _delay_ms(5000); */

  /* writeStringToLCD(CHA(20)); */
  /* _delay_ms(5000); */

  /* writeStringToLCD(CUP(20,20)); */
  /* _delay_ms(5000); */

  /* writeStringToLCD(CUP(1,1)); */
  /* _delay_ms(5000); */


  while (1) {
    serialChar = receiveByte();

    switch (serialChar) {
    case '\r':
      writeStringToLCD("\r\n");
      transmitString("\n" CNL(1) "\r");
      break;
      ;
    case '\f':
      writeCharToLCD(serialChar);
      transmitString(ED(2) CUP(1,1));
      break;
      ;
    case 0x7f: // Backspace (sent as delete)
      writeStringToLCD("\b \b");
      transmitString(CUB(1) " " CUB(1));
      break;
      ;
    default:
      writeCharToLCD(serialChar);
      transmitByte(serialChar);   // Echo character back to serial console
      ;
    }
  }

  return 0;
}
