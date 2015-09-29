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
 * File: lcdLib.c
 * Author: Collin J. Doering <collin.doering@rekahsoft.ca>
 * Date: Sep 29, 2015
 */

//#include <avr/interrupt.h>
//#include <avr/power.h>
#include <util/delay.h>

// Include header
#include "lcdLib.h"

// Function definitions
void flashLED(uint8_t times) {
  while (times > 0) {
    STATUS_LED_PORT |= 1 << STATUS_LED; // turn on status LED
    _delay_ms(100);
    STATUS_LED_PORT &= ~(1 << STATUS_LED); // turn status LED off
    _delay_ms(100);
    times--;
  }
}

//------------------------------------

void clkLCD(void) {
  LCD_CTRL_PORT |= 1 << LCD_ENABLE;
  _delay_us(LCD_DELAY);
  LCD_CTRL_PORT &= ~(1 << LCD_ENABLE);
  _delay_us(LCD_DELAY);
}

void loop_until_LCD_BF_clear(void) {
  LCD_CTRL_PORT = (LCD_CTRL_PORT & ~(1 << LCD_RS)) | (1 << LCD_RW); // RS=0, RW=1
  LCD_DBUS_DDR &= ~(1 << LCD_BF); // Set LCD_BF as input
  clkLCD();

  STATUS_LED_PORT |= 1 << STATUS_LED; // DEBUG
  while (bit_is_clear(LCD_DBUS_PIN, LCD_BF)) {
    clkLCD();
  }

  /* loop_until_bit_is_clear(LCD_DBUS_PIN, LCD_BF); */
  STATUS_LED_PORT &= ~(1 << STATUS_LED); // DEBUG

    
  LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins as outputs
}

void writeLCDInstr_(uint8_t instr) {
  LCD_CTRL_PORT &= ~((1 << LCD_RS) | (1 << LCD_RW)); // RS=RW=0
  LCD_DBUS_PORT = instr;
  clkLCD();
}

void writeLCDInstr(uint8_t instr) {
  loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions

  writeLCDInstr_(instr);
}

void writeCharToLCD_(char c) {
  LCD_CTRL_PORT |= (1 << LCD_RS);  // RS=1
  LCD_CTRL_PORT &= ~(1 << LCD_RW); // RW=0
  LCD_DBUS_PORT = c;
  clkLCD();
}

void writeCharToLCD(char c) {
  loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
  
  writeCharToLCD_(c);
}

void writeStringToLCD(const char* str) {
  while (*str != '\0') {
    writeCharToLCD(*str);
    str++;
  }
}

void clearScreen(void) {
  writeLCDInstr(CMD_CLEAR_SCREEN);
  _delay_us(LCD_CLEAR_SCREEN_DELAY);
}

char readCharFromLCD(void) {
  loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions

  LCD_CTRL_PORT |= (1 << LCD_RW) | (1 << LCD_RW); // RS=RW=1
  LCD_DBUS_DDR = 0; // Set all LCD_DBUS_PORT pins as inputs
  clkLCD();

  char c = LCD_DBUS_PIN;
  LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins to outputs
  return c;
}

/*
  Set all pins of LCD_DBUS_PORT, as well as pins LCD_RS, and LCD_RW, on
  LCD_CTRL_PORT as outputs
*/
static inline void enableLCDOutput(void) {
  LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_ENABLE);
  LCD_DBUS_DDR = 0xff;
}

/*
  Set all pins of LCD_DBUS_PORT as well as LCD_RS, and LCD_RW on LCD_CTRL_PORT as
  inputs (disabling their output)
*/
static inline void disableLCDOutput(void) {
  LCD_CTRL_DDR &= ~((1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_ENABLE));
  LCD_DBUS_DDR = 0;
}

static inline void softwareLCDInitPulse(void) {
  enableLCDOutput();
  LCD_CTRL_PORT &= ~((1 << LCD_RS) | (1 << LCD_RW)); // RS=RW=0
  LCD_DBUS_PORT = 0x30;
  clkLCD();
}

/*
  Do software initialization as specified by the datasheet
*/
void initLCD (void) {
  _delay_ms(30);     // Wait 15ms as per datasheet

  softwareLCDInitPulse();
  
  // Disable transmission and wait minimum 4.1ms as per datasheet
  //disableLCDOutput();
  _delay_us(8200);

  softwareLCDInitPulse();

  // Disable transmission and wait minimum 100us as per datasheet
  //disableLCDOutput();
  _delay_us(200);

  softwareLCDInitPulse();

  // Function set (2 lines with 5x7 dot character font)
  writeLCDInstr_(0x38); // RS=RW=0, 0b00111000

  /* BF now has to be checked */

  // Set functions of LCD
  writeLCDInstr_(0x08); // Display off
  _delay_ms(1);
  writeLCDInstr_(0x01); // Clear display
  _delay_ms(16);
  //_delay_us(15200 - LCD_DELAY); // Extra delay for clear display
  writeLCDInstr_(0x06); // Increment mode, no shift
  _delay_ms(1);
  writeLCDInstr_(0x0E); // Display on, cursor on, blink off

  flashLED(5); // DEBUG
}

void initLCDByInternalReset(void) {
  enableLCDOutput();
  writeLCDInstr_(0x38);
  writeLCDInstr_(0x0F);
  writeLCDInstr_(0x06);
  writeLCDInstr_(0x01);
  _delay_ms(16);

  /* writeLCDInstr_(0x01); // Clear display */
  /* writeLCDInstr_(0x30); // RS=RW=0, 0b00110000 */
  /* writeLCDInstr_(0x08); // Display off, cursor off, blink off */
  /* writeLCDInstr_(0x06); // Increment mode, no shift */
  /* writeLCDInstr_(0x0E); // Display on, cursor on, blink off */
}
