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

// Includes
#include <avr/io.h>

// LCD data bus PORT, PIN and DDR
#define LCD_DBUS_PORT PORTB
#define LCD_DBUS_DDR  DDRB
#define LCD_DBUS_PIN  PINB

#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR  DDRD

#define LCD_RS        PD2
#define LCD_RW        PD3
#define LCD_ENABLE    PD4

#define LCD_DBUS0     PB0
#define LCD_DBUS1     PB1
#define LCD_DBUS2     PB2
#define LCD_DBUS3     PB3
#define LCD_DBUS4     PB4
#define LCD_DBUS5     PB5
#define LCD_DBUS6     PB6
#define LCD_DBUS7     PB7
#define LCD_BF        PB7

// LCD in 4 bit mode (default is 8 bit mode)
//#define FOUR_BIT_MODE

// LCD delays (in microseconds when unspecified)
#define LCD_DELAY               25
#define LCD_INIT_DELAY0         15    // milliseconds
#define LCD_INIT_DELAY1         8200
#define LCD_INIT_DELAY2         200

#define LCD_CLEAR_DISPLAY_DELAY 16000
#define LCD_RETURN_HOME_DELAY   16000
#define LCD_GENERIC_INSTR_DELAY 50

/* LCD Commands */

// Simple commands with no options
#define CMD_INIT             0x30
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

void clkLCD(void);

void loop_until_LCD_BF_clear(void);

void writeLCDInstr_(uint8_t instr);

void writeLCDInstr(uint8_t instr);

void writeCharToLCD_(char c);

void writeCharToLCD(char c);

void writeStringToLCD(const char* str);

void clearDisplay(void);

void returnHome(void);

char readCharFromLCD(void);

/*
  Do software initialization as specified by the datasheet
*/
void initLCD(void);

void initLCDByInternalReset(void);

