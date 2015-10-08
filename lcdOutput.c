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
#include "USART.h"

int main(void) {
  clock_prescale_set(clock_div_1);
  
  STATUS_LED_DDR |= 1 << STATUS_LED; // DEBUG

  initUSART();
  char serialChar;

  initLCD();
  //initLCDByInternalReset();

  while (1) {
    serialChar = receiveByte();

    switch (serialChar) {
    case '\r':
      writeStringToLCD("\r\n");
      transmitString("\n\e[1E\r");
      break;
      ;
    case '\f':
      writeCharToLCD(serialChar);
      transmitString("\e[2J\e[1;1H");
      break;
      ;
    case 0x7f: // Backspace (sent as delete)
      writeStringToLCD("\b \b");
      transmitString("\e[1D \e[1D");
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
