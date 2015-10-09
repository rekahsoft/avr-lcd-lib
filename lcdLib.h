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
 * File: lcdLib.h
 * Author: Collin J. Doering <collin.doering@rekahsoft.ca>
 * Date: Sep 29, 2015
 */

// Includes -------------------------------------------------------------------------------

#include <avr/io.h>
#include "lcdLibConfig.h"

//------------------------------------------------------------------------------------------

/* LCD Commands */

// Simple commands with no options
#define CMD_INIT             0x30
#define CMD_INIT_FOUR_BIT    0x20
#define CMD_CLEAR_DISPLAY    0x01
#define CMD_RETURN_HOME      0x02

// Entry Set instruction and associated options
#define INSTR_ENTRY_SET      0x04
#define INSTR_ENTRY_SET_ID   1
#define INSTR_ENTRY_SET_S    0

// Display control instruction and associated options
#define INSTR_DISPLAY        0x08
#define INSTR_DISPLAY_D      2
#define INSTR_DISPLAY_C      1
#define INSTR_DISPLAY_B      0

// Cursor or display shift instruction and associated options
#define INSTR_MOV_SHIFT      0x10
#define INSTR_MOV_SHIFT_SC   0x08
#define INSTR_MOV_SHIFT_RL   0x04

// Function set instruction and associated options
#define INSTR_FUNC_SET       0x20
#define INSTR_FUNC_SET_DL    4
#define INSTR_FUNC_SET_N     3
#define INSTR_FUNC_SET_F     2

// Set CG RAM address instruction
#define INSTR_CGRAM_ADDR     0x60

// Set DD RAM address instruction
#define INSTR_DDRAM_ADDR     0x80


//------------------------------------

#define STATUS_LED_PORT PORTC
#define STATUS_LED_DDR  DDRC
#define STATUS_LED      PC5

// Function definitions
void flashLED(uint8_t times);

//------------------------------------

/*
  Bring LCD_ENABLE line high, wait for LCD_ENABLE_HIGH_DELAY; then bring LCD_ENABLE line low
  and wait for LCD_ENABLE_LOW_DELAY.

  Note: LCD_ENABLE, LCD_ENABLE_HIGH_DELAY, and LCD_ENABLE_LOW_DELAY must be defined in lcdLibConfig.h
 */
void clkLCD(void);

void loop_until_LCD_BF_clear(void);

#ifdef FOUR_BIT_MODE
/*
  Given a 8 bit integer, writes the four MSB's (one nibble) to the LCD data bus.

  Note: this is only defined in FOUR_BIT_MODE
 */
void writeLCDNibble_(uint8_t);
#endif

/*
  Given an 8 bit integer, writes it to the LCD data bus.

  Note that this function does not ensure the LCD is ready to accept new data and thus needs to
  be handled by the caller.
 */
void writeLCDByte_(uint8_t);

/*
  Given a 8 bit integer representing a LCD instruction, sends it to the LCD display.

  Note that this function does not ensure the LCD is ready to accept a new instruction and thus
  needs to be handled by the caller.
 */
void writeLCDInstr_(uint8_t);

/*
  Given a 8 bit integer representing a LCD instruction, waits until the LCD is ready and sends
  the instruction.
 */
void writeLCDInstr(uint8_t);

/*
  Writes a character to the LCD display at the current cursor position.

  Note that this function does not ensure the LCD is ready to accept a new character and thus
  needs to be handled by the caller.
 */
void writeCharToLCD_(char);

/*
  Writes a character to the LCD display at the current cursor position after the LCD display is
  ready for new data.
 */
void writeCharToLCD(char);

/*
  Writes a string to the LCD starting from the current cursor position.
 */
void writeStringToLCD(const char*);

/*
  Clears the display and positions the cursor in the top left of the LCD screen.
 */
void clearDisplay(void);

/*
  Brings the cursor the the top left of the LCD screen.
 */
void returnHome(void);

/*
  UNIMPLEMENTED
 */
char readCharFromLCD(void);

/*
  Initialize the LCD display via software initialization as specified by the datasheet.
*/
void initLCD(void);

/*
  Initialize the LCD display via its internal reset circuit.

  Note: this is not the recommended way to initialize the LCD as it is unreliable and depends
        on the power supply. Preferably the software initialization method should be used (via
        the initLCD function).
 */
void initLCDByInternalReset(void);

//------------------------------------------------------------------------------------------

/*
  Mode and settings sanity check
*/

#if !defined (LCD_RS) || !defined (LCD_RS_PORT) || !defined (LCD_RS_DDR) || !defined (LCD_RW) || !defined (LCD_RW_PORT) || !defined (LCD_RW_DDR) || !defined (LCD_ENABLE) || !defined (LCD_ENABLE_PORT) || !defined (LCD_ENABLE_DDR)
#error "All modes require LCD_RS[,_PORT,_DDR], LCD_RW[,_PORT,_DDR], and LCD_ENABLE[,_PORT,_DDR] be defined."
#endif

#if defined (EIGHT_BIT_ARBITRARY_PIN_MODE) && defined (FOUR_BIT_MODE)
#error "EIGHT_BIT_ARBITRARY_PIN_MODE and FOUR_BIT_MODE are mutually exclusive. Choose one."
#elif defined (EIGHT_BIT_ARBITRARY_PIN_MODE) || defined (FOUR_BIT_MODE)

// EIGHT_BIT_ARBITRARY_PIN_MODE specific requirements
#ifdef EIGHT_BIT_ARBITRARY_PIN_MODE
#if !defined (LCD_DBUS0) || !defined (LCD_DBUS0_PORT) || !defined (LCD_DBUS0_DDR) || !defined (LCD_DBUS0_PIN) || !defined (LCD_DBUS1) || !defined (LCD_DBUS1_PORT) || !defined (LCD_DBUS1_DDR) || !defined (LCD_DBUS1_PIN) || !defined (LCD_DBUS2) || !defined (LCD_DBUS2_PORT) || !defined (LCD_DBUS2_DDR) || !defined (LCD_DBUS2_PIN) || !defined (LCD_DBUS3) || !defined (LCD_DBUS3_PORT) || !defined (LCD_DBUS3_DDR) || !defined (LCD_DBUS3_PIN)
#error "EIGHT_BIT_ARBITRARY_PIN_MODE require that LCD_DBUS*[,_PORT,_DDR,_PIN] be defined."
#endif
#endif

// Requirements for EIGHT_BIT_ARBITRARY_PIN_MODE and FOUR_BIT_MODE
#if !defined (LCD_DBUS4) || !defined (LCD_DBUS4_PORT) || !defined (LCD_DBUS4_DDR) || !defined (LCD_DBUS4_PIN) || !defined (LCD_DBUS5) || !defined (LCD_DBUS5_PORT) || !defined (LCD_DBUS5_DDR) || !defined (LCD_DBUS5_PIN) || !defined (LCD_DBUS6) || !defined (LCD_DBUS6_PORT) || !defined (LCD_DBUS6_DDR) || !defined (LCD_DBUS6_PIN) || !defined (LCD_DBUS7) || !defined (LCD_DBUS7_PORT) || !defined (LCD_DBUS7_DDR) || !defined (LCD_DBUS7_PIN)
#error "Both EIGHT_BIT_ARBITRARY_PIN_MODE and FOUR_BIT_MODE require that LCD_DBUS*[,_PORT,_DDR,_PIN] be defined."
#endif

// Set LCD_BF automatically for both EIGHT_BIT_ARBITRARY_PIN_MODE and FOUR_BIT_MODE
#undef  LCD_BF
#define LCD_BF         LCD_DBUS7

#else
#if !defined (LCD_DBUS_PORT) || !defined (LCD_DBUS_DDR) || !defined (LCD_DBUS_PIN) || !defined (LCD_BF)
#error "Default mode requires that LCD_DBUS_[PORT,DDR,PIN] and LCD_BF be defined."
#endif

#undef  LCD_DBUS7_PORT
#define LCD_DBUS7_PORT LCD_DBUS_PORT
#undef  LCD_DBUS7_DDR
#define LCD_DBUS7_DDR  LCD_DBUS_DDR
#undef  LCD_DBUS7_PIN
#define LCD_DBUS7_PIN  LCD_DBUS_PIN
#endif
