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

void showSomePrases(void) {
  const char* data[4] = { "Hello there friend!!\nIsn't it a nice day.\nAnyways, I must go..\nTo a midnight show;)",
                          "This is some other text. It should be wrapped appropriately.\nIsn't that neat!",
                          "Welcome! (line 1)\nThis is line 2.\nAnd here is line 3.\nAnd finally, line 4.",
                          "Finally, this is the\nend; of the array,\nthat is.\nCheers."};

  for (uint8_t i = 0; i < 4; i++) {
    writeStringToLCD(data[i]);
    _delay_ms(5000);
    clearDisplay();
  }
  _delay_ms(3000);
}

int main(void) {
  clock_prescale_set(clock_div_1);
  
  STATUS_LED_DDR |= 1 << STATUS_LED; // DEBUG

  initUSART();
  char serialChar;

  initLCD();
  //initLCDByInternalReset();

  showSomePrases();

  while (1) {
    serialChar = receiveByte();
    writeCharToLCD(serialChar);
  }

  return 0;
}
