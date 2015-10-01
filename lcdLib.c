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
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE);
  _delay_us(LCD_ENABLE_HIGH_DELAY);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE);
  _delay_us(LCD_ENABLE_LOW_DELAY);
}

void loop_until_LCD_BF_clear(void) {
  LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
  LCD_RW_PORT |= (1 << LCD_RW);  // RW=1

  // Set LCD_BF as input
#ifdef FOUR_BIT_MODE
  LCD_DBUS7_PORT &= ~(1 << LCD_BF);
#else
  LCD_DBUS_DDR &= ~(1 << LCD_BF);
#endif

  STATUS_LED_PORT |= 1 << STATUS_LED; // DEBUG
  do {
    clkLCD();
  }
#ifdef FOUR_BIT_MODE
  while (bit_is_clear(LCD_DBUS7_PIN, LCD_BF));
  #else
  while (bit_is_clear(LCD_DBUS_PIN, LCD_BF));
#endif
  STATUS_LED_PORT &= ~(1 << STATUS_LED); // DEBUG

#ifdef FOUR_BIT_MODE
  LCD_DBUS7_DDR = 0;
  LCD_DBUS6_DDR = 0;
  LCD_DBUS5_DDR = 0;
  LCD_DBUS4_DDR = 0;
#else
  LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins as outputs
#endif
}

#ifdef FOUR_BIT_MODE
/*
  Writes one nibble to the LCD data bus. Does not touch the RS or RW control lines.
  Note: the bits that are sent are the four MSBs of the given argument
 */
void writeLCDNibble_(uint8_t b) {
  // Reset data lines to zeros
  LCD_DBUS7_PORT &= ~(1 << LCD_DBUS7);
  LCD_DBUS6_PORT &= ~(1 << LCD_DBUS6);
  LCD_DBUS5_PORT &= ~(1 << LCD_DBUS5);
  LCD_DBUS4_PORT &= ~(1 << LCD_DBUS4);

  // Write 1's where appropriate on data lines
  if (b & (1 << 7)) LCD_DBUS7_PORT |= (1 << LCD_DBUS7);
  if (b & (1 << 6)) LCD_DBUS6_PORT |= (1 << LCD_DBUS6);
  if (b & (1 << 5)) LCD_DBUS5_PORT |= (1 << LCD_DBUS5);
  if (b & (1 << 4)) LCD_DBUS4_PORT |= (1 << LCD_DBUS4);

  // Pulse the enable line
  clkLCD();
}
#endif

/*
  Write a byte to the LCD data bus. Does not touch the RS or RW control lines.
*/
void writeLCDByte_(uint8_t b) {
  LCD_DBUS_PORT = b;
  clkLCD();
}

void writeLCDInstr_(uint8_t instr) {
LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

#ifdef FOUR_BIT_MODE
  writeLCDNibble_(instr);
  writeLCDNibble_(instr << 4);
#else
  LCD_DBUS_PORT = instr;
  clkLCD();
#endif
}

void writeLCDInstr(uint8_t instr) {
  loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
  writeLCDInstr_(instr);
}

void writeCharToLCD_(char c) {
  LCD_RS_PORT |= (1 << LCD_RS);  // RS=1
  LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

#ifdef FOUR_BIT_MODE
  writeLCDNibble_(c);
  writeLCDNibble_(c << 4);
#else
  LCD_DBUS_PORT = c;
  clkLCD();
#endif
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

void clearDisplay(void) {
  writeLCDInstr(CMD_CLEAR_DISPLAY);
}

void returnHome(void) {
  writeLCDInstr(CMD_RETURN_HOME);
}

/* char readCharFromLCD(void) { */
/*   loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions */

/*   LCD_CTRL_PORT |= (1 << LCD_RW) | (1 << LCD_RW); // RS=RW=1 */
/*   LCD_DBUS_DDR = 0; // Set all LCD_DBUS_PORT pins as inputs */
/*   clkLCD(); */

/*   char c = LCD_DBUS_PIN; */
/*   LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins to outputs */
/*   return c; */
/* } */

/*
  Set all pins of LCD_DBUS, as well as pins LCD_RS, and LCD_RW as outputs
*/
static inline void enableLCDOutput(void) {
  LCD_RS_DDR |= (1 << LCD_RS);
  LCD_RW_DDR |= (1 << LCD_RW);
  LCD_ENABLE_DDR |= (1 << LCD_ENABLE);

#if defined (FOUR_BIT_MODE) || defined (EIGHT_BIT_ARBITRARY_PIN_MODE)
  LCD_DBUS7_DDR |= (1 << LCD_DBUS7);
  LCD_DBUS6_DDR |= (1 << LCD_DBUS6);
  LCD_DBUS5_DDR |= (1 << LCD_DBUS5);
  LCD_DBUS4_DDR |= (1 << LCD_DBUS4);
#ifdef EIGHT_BIT_ARBITRARY_PIN_MODE
  LCD_DBUS3_DDR |= (1 << LCD_DBUS3);
  LCD_DBUS2_DDR |= (1 << LCD_DBUS2);
  LCD_DBUS1_DDR |= (1 << LCD_DBUS1);
  LCD_DBUS0_DDR |= (1 << LCD_DBUS0);
#endif
#else
  LCD_DBUS_DDR = 0xff;
#endif
}

/*
  Set all pins of LCD_DBUS as well as LCD_RS, and LCD_RW as inputs (disabling their output)
*/
static inline void disableLCDOutput(void) {
  LCD_RS_DDR &= ~(1 << LCD_RS);
  LCD_RW_DDR &= ~(1 << LCD_RW);
  LCD_ENABLE_DDR &= ~(1 << LCD_ENABLE);

#ifdef FOUR_BIT_MODE
LCD_DBUS7 &= ~(1 << LCD_DBUS7);
LCD_DBUS6 &= ~(1 << LCD_DBUS6);
LCD_DBUS5 &= ~(1 << LCD_DBUS5);
LCD_DBUS4 &= ~(1 << LCD_DBUS4);
#else
  LCD_DBUS_DDR = 0;
#endif
}

static inline void softwareLCDInitPulse(void) {
  enableLCDOutput();
  LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
  LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

#ifdef FOUR_BIT_MODE
  writeLCDNibble_(CMD_INIT);
#else
  LCD_DBUS_PORT = CMD_INIT;
  clkLCD();
#endif
}

/*
  Do software initialization as specified by the datasheet
*/
void initLCD (void) {
  enableLCDOutput();

  // Wait minimum 15ms as per datasheet
  _delay_us(LCD_INIT_DELAY0);

  softwareLCDInitPulse();
  
  // Wait minimum 4.1ms as per datasheet
  _delay_us(LCD_INIT_DELAY1);

  softwareLCDInitPulse();

  // Wait minimum 100us as per datasheet
  _delay_us(LCD_INIT_DELAY2);

  softwareLCDInitPulse();

  // Function set (2 lines with 5x7 dot character font)
  // RS=RW=0, DBUS=b00111000,0x38
  writeLCDInstr_(INSTR_FUNC_SET | (1 << INSTR_FUNC_SET_DL) | (1 << INSTR_FUNC_SET_N));

  /* BF now can be checked */

  // Set functions of LCD
  writeLCDInstr_(INSTR_DISPLAY); // Display off
  _delay_us(LCD_GENERIC_INSTR_DELAY);

  // Clear display
  writeLCDInstr_(CMD_CLEAR_DISPLAY);
  _delay_us(LCD_CLEAR_DISPLAY_DELAY);

  // Increment mode, no shift
  writeLCDInstr_(INSTR_ENTRY_SET | (1 << INSTR_ENTRY_SET_ID));
  _delay_us(LCD_GENERIC_INSTR_DELAY);

  // Display on, cursor on, blink off
  writeLCDInstr_(INSTR_DISPLAY | (1 << INSTR_DISPLAY_D) | (1 << INSTR_DISPLAY_C));
  _delay_us(LCD_GENERIC_INSTR_DELAY);

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
