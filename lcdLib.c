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

// Includes -----------------------------------------------------------------------------------
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>

#include "lcdLib.h"

//---------------------------------------------------------------------------------------------
// Static global variables

static volatile uint8_t currentLineNum;
static volatile uint8_t currentLineChars;

static volatile uint8_t saveCursorLineNum;
static volatile uint8_t saveCursorLineChars;

static volatile uint8_t lcdState;

static const uint8_t lineBeginnings[LCD_NUMBER_OF_LINES] = { LCD_LINE_BEGINNINGS };

//---------------------------------------------------------------------------------------------
// Static functions

/*
  Bring LCD_ENABLE line high, wait for LCD_ENABLE_HIGH_DELAY; then bring LCD_ENABLE line low
  and wait for LCD_ENABLE_LOW_DELAY.

  Note: LCD_ENABLE, LCD_ENABLE_HIGH_DELAY, and LCD_ENABLE_LOW_DELAY must be defined in lcdLibConfig.h
 */
static void clkLCD(void) {
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE);
  _delay_us(LCD_ENABLE_HIGH_DELAY);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE);
  _delay_us(LCD_ENABLE_LOW_DELAY);
}

/*
  Wait until LCD_BF (busy flag) is cleared (low).
 */
static void loop_until_LCD_BF_clear(void) {
  uint8_t bf;

  LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
  LCD_RW_PORT |= (1 << LCD_RW);  // RW=1

  // Set LCD_BF as input
  LCD_DBUS7_DDR &= ~(1 << LCD_BF);

  do {
    bf = 0;
    LCD_ENABLE_PORT |= (1 << LCD_ENABLE);
    _delay_us(1);                          // 'delay data time' and 'enable pulse width'

    bf |= (LCD_DBUS7_PIN & (1 << LCD_BF));

    LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE);
    _delay_us(1);                          // 'address hold time', 'data hold time' and 'enable cycle width'

#ifdef FOUR_BIT_MODE
    LCD_ENABLE_PORT |= (1 << LCD_ENABLE);
    _delay_us(1);                          // 'delay data time' and 'enable pulse width'
    LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE);
    _delay_us(1);                          // 'address hold time', 'data hold time' and 'enable cycle width'
#endif
  } while (bf);

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
  LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins as outputs
#endif
}

/*
  Given a 8 bit integer, writes the four MSB's (one nibble) to the LCD data bus.

  Note: this is only defined in FOUR_BIT_MODE
 */
#ifdef FOUR_BIT_MODE
static void writeLCDDBusNibble_(uint8_t b) {
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
  Given an 8 bit integer, writes it to the LCD data bus, regardless of its
  configuration (default 8-bit mode, 8-bit arbitrary pin mode and 4-bit mode). In the default
  8-bit mode and EIGHT_BIT_ARBITRARY_PIN_MODE, the given data is written in one cycle using the
  writeLCDDBusByte_ function. In FOUR_BIT_MODE however, the given data is written in two cycles
  using two successive calls to the writeLCDDBusNibble_ function.

  This function does not ensure the LCD is ready to accept new data and thus needs to
  be handled by the caller.
 */
static void writeLCDDBusByte_(uint8_t b) {
#ifdef FOUR_BIT_MODE
  writeLCDDBusNibble_(b);
  writeLCDDBusNibble_(b << 4);
#elif defined (EIGHT_BIT_ARBITRARY_PIN_MODE)
  // Reset data lines to zeros
  LCD_DBUS7_PORT &= ~(1 << LCD_DBUS7);
  LCD_DBUS6_PORT &= ~(1 << LCD_DBUS6);
  LCD_DBUS5_PORT &= ~(1 << LCD_DBUS5);
  LCD_DBUS4_PORT &= ~(1 << LCD_DBUS4);
  LCD_DBUS3_PORT &= ~(1 << LCD_DBUS3);
  LCD_DBUS2_PORT &= ~(1 << LCD_DBUS2);
  LCD_DBUS1_PORT &= ~(1 << LCD_DBUS1);
  LCD_DBUS0_PORT &= ~(1 << LCD_DBUS0);

  // Write 1's where appropriate on data lines
  if (b & (1 << 7)) LCD_DBUS7_PORT |= (1 << LCD_DBUS7);
  if (b & (1 << 6)) LCD_DBUS6_PORT |= (1 << LCD_DBUS6);
  if (b & (1 << 5)) LCD_DBUS5_PORT |= (1 << LCD_DBUS5);
  if (b & (1 << 4)) LCD_DBUS4_PORT |= (1 << LCD_DBUS4);
  if (b & (1 << 3)) LCD_DBUS3_PORT |= (1 << LCD_DBUS3);
  if (b & (1 << 2)) LCD_DBUS2_PORT |= (1 << LCD_DBUS2);
  if (b & (1 << 1)) LCD_DBUS1_PORT |= (1 << LCD_DBUS1);
  if (b & (1 << 0)) LCD_DBUS0_PORT |= (1 << LCD_DBUS0);

  clkLCD();
#else
  LCD_DBUS_PORT = b;
  clkLCD();
#endif
}

/*
  Given a 8 bit integer representing a LCD instruction, sends it to the LCD display.

  Sets RS=RW=0 and writes the given 8 bit integer to the LCD databus.

  Note that this function does not ensure the LCD is ready to accept a new instruction and thus
  needs to be handled by the caller.
*/
static void writeLCDInstr_(uint8_t instr) {
  LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
  LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

  writeLCDDBusByte_(instr);
}

/*
  Given a 8 bit integer representing a LCD instruction, waits until the LCD is ready and sends
  the instruction.
 */
static inline void writeLCDInstr(uint8_t instr) {
  loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
  writeLCDInstr_(instr);
}

/*
  Sets RS=1, RW=0 and accepts a char (8 bit) and outputs it to the current cursor position of
  the LCD. In the default 8-bit mode and EIGHT_BIT_ARBITRARY_PIN_MODE, the given data is
  written in one cycle using the writeLCDDBusByte_ function. In FOUR_BIT_MODE however, the given
  data is written in two cycles using two successive calls to the writeLCDDBusNibble_ function.
*/
static void writeCharToLCD_(char c) {
  LCD_RS_PORT |= (1 << LCD_RS);  // RS=1
  LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

  writeLCDDBusByte_(c);
}

/*
  Given a character string, and a uint8_t pointer, reads the character string until a
  non-numerical ASCII character, returning the integer representation of the number read. At
  the end of the functions execution, the found_num uint8_t* will be updated to indicate how
  many digits were read. The new_loc char** will be updated with the new parsing position in
  the string.
 */
static uint8_t readASCIINumber(char* str, uint8_t* found_num, char** new_loc) {
  uint8_t nums[3];

  *found_num = 0;
  while (*str != '\0' && *found_num < 3) {
    if (*str >= 0x30 && *str <= 0x39) {
      // Use *str as a number (specified in ASCII)
      nums[(*found_num)++] = *str - 0x30;
    } else {
      break;
    }

    str++;
  }
  *new_loc = str;

  uint8_t ret = 0;
  uint8_t i = *found_num - 1;
  for (uint8_t fnd = 0; fnd < *found_num; fnd++)
    ret += nums[fnd] * pow(10, i--);
  return ret;
}

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

#if defined (FOUR_BIT_MODE) || defined (EIGHT_BIT_ARBITRARY_PIN_MODE)
  LCD_DBUS7_DDR &= ~(1 << LCD_DBUS7);
  LCD_DBUS6_DDR &= ~(1 << LCD_DBUS6);
  LCD_DBUS5_DDR &= ~(1 << LCD_DBUS5);
  LCD_DBUS4_DDR &= ~(1 << LCD_DBUS4);
#ifdef EIGHT_BIT_ARBITRARY_PIN_MODE
  LCD_DBUS3_DDR &= ~(1 << LCD_DBUS3);
  LCD_DBUS2_DDR &= ~(1 << LCD_DBUS2);
  LCD_DBUS1_DDR &= ~(1 << LCD_DBUS1);
  LCD_DBUS0_DDR &= ~(1 << LCD_DBUS0);
#endif
#else
  LCD_DBUS_DDR = 0;
#endif
}

/*
  Set RS=RW=0 and write the CMD_INIT command to the LCD data bus. Note that an appropriate
  pause must follow before sending new commands to the LCD using writeLCD*_ functions.
 */
static inline void softwareLCDInitPulse(void) {
  enableLCDOutput();
  LCD_RS_PORT &= ~(1 << LCD_RS); // RS=0
  LCD_RW_PORT &= ~(1 << LCD_RW); // RW=0

#ifdef FOUR_BIT_MODE
  writeLCDDBusNibble_(CMD_INIT);
#else
  writeLCDDBusByte_(CMD_INIT);
#endif
}


//---------------------------------------------------------------------------------------------
// Library function definitions

/*
  Do software initialization as specified by the datasheet
*/
void initLCD(void) {
  enableLCDOutput();

  _delay_us(LCD_INIT_DELAY0); // Wait minimum 15ms as per datasheet
  softwareLCDInitPulse();
  _delay_us(LCD_INIT_DELAY1); // Wait minimum 4.1ms as per datasheet
  softwareLCDInitPulse();
  _delay_us(LCD_INIT_DELAY2); // Wait minimum 100us as per datasheet
  softwareLCDInitPulse();

#if defined (FOUR_BIT_MODE)
  // Function Set (4-bit interface; 2 lines with 5x7 dot character font)
  writeLCDDBusNibble_(CMD_INIT_FOUR_BIT);
  writeLCDInstr_(CMD_INIT_FOUR_BIT | (1 << INSTR_FUNC_SET_N) | LCD_FONT);
#else
  // Function set (8-bit interface; 2 lines with 5x7 dot character font)
  // RS=RW=0, DBUS=b00111000,0x38
  writeLCDInstr_(INSTR_FUNC_SET | (1 << INSTR_FUNC_SET_DL) | (1 << INSTR_FUNC_SET_N) | LCD_FONT);
#endif

  /* BF now can be checked */

  // Set functions of LCD
  writeLCDInstr(INSTR_DISPLAY); // Display off

  // Clear display
  writeLCDInstr(CMD_CLEAR_DISPLAY);

  // Increment mode, no shift
  writeLCDInstr(INSTR_ENTRY_SET | (1 << INSTR_ENTRY_SET_ID));

  // Display on, cursor on, blink off
  lcdState = (1 << INSTR_DISPLAY_D) | (1 << INSTR_DISPLAY_C);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

/*
  Given a single character, checks whether its a ASCII escape and does the following:

  - Newline '\n': moves the cursor to the next physical line of the LCD display; if the cursor is on
  the last line of the display, clears the display and positions the cursor at the top left
  of the LCD
  - Carriage return '\r': moves the cursor to the beginning of the current line
  - Backspace '\b': moves the cursor one position backwards, wrapping to the end of the
  previous line when at the beginning of a line (other then the first one). A space is then
  inserted to replace the character at point, without moving the cursor. When the cursor is
  at the beginning of the first line, does nothing.
  - Form feed '\f': clears the LCD display and places the cursor at the beginning of the first line.
  - Alarm '\a': ignored

  Any other character is sent to the LCD display using writeCharToLCD_.
*/
void writeCharToLCD(char c) {
  switch (c) {
  case '\n': // Line feed
    if (currentLineNum == LCD_NUMBER_OF_LINES - 1) {
      scrollUp(1);

      currentLineChars = 0;
      writeLCDInstr(INSTR_DDRAM_ADDR | lineBeginnings[currentLineNum]);
    } else {
      currentLineChars = 0;
      writeLCDInstr(INSTR_DDRAM_ADDR | lineBeginnings[++currentLineNum]);
    }
    break;
  case '\a': // Alarm
    break;
  case '\b': // Backspace (non-destructive)
    if (currentLineChars == 0 && currentLineNum == 0) {
      // At first line, first column; there is no where to move; do nothing
      break;
    } else if (currentLineChars == 0) {
      // At beginning of line, need to move the end of previous line
      currentLineChars = LCD_CHARACTERS_PER_LINE - 1;
      writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[--currentLineNum] + currentLineChars));
    } else {
      // OK, simply go back one character
      writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + --currentLineChars));
    }

    break;
  case '\r': // Carriage return
    writeLCDInstr(INSTR_DDRAM_ADDR | lineBeginnings[currentLineNum]);
    currentLineChars = 0;
    break;
  case '\f': // Form feed
    clearDisplay();
    break;
  default:   // Printable character
    if (currentLineChars == LCD_CHARACTERS_PER_LINE - 1 && currentLineNum == LCD_NUMBER_OF_LINES - 1) {
      loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
      writeCharToLCD_(c);

      scrollUp(1);

      currentLineChars = 0;
      writeLCDInstr(INSTR_DDRAM_ADDR | lineBeginnings[currentLineNum]);
    } else if (currentLineChars == LCD_CHARACTERS_PER_LINE - 1) {
      loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
      writeCharToLCD_(c);

      currentLineChars = 0;
      writeLCDInstr(INSTR_DDRAM_ADDR | lineBeginnings[++currentLineNum]);
    } else {
      loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions
      writeCharToLCD_(c);
      currentLineChars++;
    }
  }
}

void writeStringToLCD(char* str) {
  while (*str != '\0') {
#ifdef LCD_ANSI_ESCAPE_ENABLE
    // Check for ANSI CSI (Control Sequence Introducer)
    if (*str == '\e') {
      if (*(++str) != '\0' && *str == '[') {
        char* str_ref = ++str;
        switch (*str) {
        case 's': // SCP - Save cursor position
          saveCursorPosition();
          return;
        case 'u': // RCP - Restore cursor position
          restoreCursorPosition();
          return;
        case '?': // DECTCEM
          if (*(++str_ref) != '\0' && *str_ref == '2') {
            if (*(++str_ref) != '\0' && *str_ref == '5') {
              if (*(++str_ref) != '\0') {
                if (*str_ref == 'l') {
                  hideCursor();
                } else if (*str_ref == 'h') {
                  showCursor();
                } else {
                  // Invalid escape
                }
              } // Invalid escape (early termination)
            } // Invalid escape
          } // Invalid escape
          return;
        default:
          break;
        }

        // Read optional variable length number in ASCII (0x30 - 0x3f) where 0x3a - 0x3f are
        // ignored (they are used as flags by some terminals)
        uint8_t fnd0;
        uint8_t num0 = readASCIINumber(str, &fnd0, &str);

        // Read optional (semicolon followed by optional variable length number)
        uint8_t fnd1;
        uint8_t num1;
        if (*str != '\0' && *str == ';') {
          num1 = readASCIINumber(++str, &fnd1, &str);

          // Read control character (between 0x40 - 0x7e) for two argument sequences
          switch (*str) {
          case 'f': // HVP - Horizontal and vertical position
          case 'H': // CUP - Cursor position
            num0 = fnd0 ? num0 : 1;
            num1 = fnd1 ? num1 : 1;
            setCursorPosition(num0, num1);
            break;
          default: // Invalid control character
            break;
          }
        } else if (*str != '\0') {
          // Read control character (between 0x40 - 0x7e) for single argument sequences
          switch (*str) {
          case 'A': // CUU - Cursor up
            num0 = fnd0 ? num0 : 1;
            moveCursorUp(num0);
            break;
          case 'B': // CUD - Cursor down
            num0 = fnd0 ? num0 : 1;
            moveCursorDown(num0);
            break;
          case 'C': // CUF - Cursor forward
            num0 = fnd0 ? num0 : 1;
            moveCursorForward(num0);
            break;
          case 'D': // CUB - Cursor back
            num0 = fnd0 ? num0 : 1;
            moveCursorBackward(num0);
            break;
          case 'E': // CNL - Cursor next line
            num0 = fnd0 ? num0 : 1;
            moveCursorNextLine(num0);
            break;
          case 'F': // CPL - Cursor previous line
            num0 = fnd0 ? num0 : 1;
            moveCursorPreviousLine(num0);
            break;
          case 'G': // CHA - Cursor horizontal absolute
            num0 = fnd0 ? num0 : 1;
            moveCursorToColumn(num0);
            break;
          case 'J': // ED - Erase display
            num0 = fnd0 ? num0 : 1;
            eraseDisplay(num0);
            break;
          case 'K': // EL - Erase in line
            num0 = fnd0 ? num0 : 1;
            eraseInline(num0);
            break;
          case 'S': // SU - Scroll up
            num0 = fnd0 ? num0 : 1;
            scrollUp(num0);
            break;
          case 'T': // SD Scroll down
            num0 = fnd0 ? num0 : 1;
            scrollDown(num0);
            break;
          case 'm': // SGR - Select graphic rendition (single optional argument)
            break;
          case ';': // SGR - Select graphic rendition (multiple arguments)
            if (fnd0) {
              while (fnd0) {
                readASCIINumber(++str, &fnd0, &str);
                if (fnd0) {
                  if (*str == 'm') {
                    break; // Valid SGR
                  } else if (*str == ';') {
                    continue; // More SGR parameters yet
                  } else {
                    break; // Invalid escape
                  }
                } else {
                  // Invalid escape; expected SGR parameter
                }
              }
            } else {
              // Invalid escape; expected first SGR parameter but none given
            }
            break;
          case 'n': // DSR - Device status report
            if (fnd0 && num0 == 6) {
              // Valid DSR
            } else {
              // Invalid DSR
            }
            break;
          default:  // Invalid control character
            writeCharToLCD(*str);
            break;
          }
        } else {
          return; // Invalid escape sequence (terminated early)
        }
      }
    } else {
      writeCharToLCD(*str);
    }

    str++;
#else
    writeCharToLCD(*(str++));
#endif
  }
}

//---------------------------------------------------------------------------------------------
// LCD command functions (all have associated ANSI escape)

/*
  Writes the CMD_CLEAR_DISPLAY command to the LCD using writeLCDINSTR, and clears the local
  char and line counters.
*/
void clearDisplay(void) {
  writeLCDInstr(CMD_CLEAR_DISPLAY);

  // Reset line and char number tracking
  currentLineNum   = 0;
  currentLineChars = 0;
}

/*
  Writes the CMD_RETURN_HOME command to the LCD using writeLCDInstr, and clears the local char
  and line counters.
*/
void returnHome(void) {
  writeLCDInstr(CMD_RETURN_HOME);

  // Reset line and char number tracking
  currentLineNum   = 0;
  currentLineChars = 0;
}

void getCursorPosition(uint8_t* row, uint8_t* column) {
  *row = currentLineNum + 1;
  *column = currentLineChars + 1;
}

void setCursorPosition(uint8_t row, uint8_t column) {
  // Set currentLineNum and currentLineChars
  currentLineNum = row ? row - 1 : 0;
  currentLineChars = column ? column - 1 : 0;

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorUp(uint8_t n) {
  if (n < currentLineNum + 1) {
    currentLineNum -= n;
  } else {
    currentLineNum = 0;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorDown(uint8_t n) {
  if (n + currentLineNum < LCD_NUMBER_OF_LINES) {
    currentLineNum += n;
  } else {
    currentLineNum = LCD_NUMBER_OF_LINES - 1;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorForward(uint8_t n) {
  if (n + currentLineChars < LCD_CHARACTERS_PER_LINE) {
    currentLineChars += n;
  } else {
    currentLineChars = LCD_CHARACTERS_PER_LINE - 1;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorBackward(uint8_t n) {
  if (n < currentLineChars + 1) {
    currentLineChars -= n;
  } else {
    currentLineChars = 0;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorNextLine(uint8_t n) {
  currentLineChars = 0;

  if (n + currentLineNum < LCD_NUMBER_OF_LINES) {
    currentLineNum += n;
  } else {
    currentLineNum = LCD_NUMBER_OF_LINES - 1;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorPreviousLine(uint8_t n) {
  currentLineChars = 0;

  if (n < currentLineNum + 1) {
    currentLineNum -= n;
  } else {
    currentLineNum = 0;
  }

  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void moveCursorToColumn(uint8_t n) {
  if (n <= LCD_CHARACTERS_PER_LINE) {
    currentLineChars = n ? n - 1 : 0;
    writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
  } // else index out of range (off screen column)
}

void eraseDisplay(uint8_t n) {
  switch (n) {
  case 0: // Clear from cursor to end of screen
  case 1: // Clear from cursor to beginning of screen
  case 2: // Clear entire screen
    clearDisplay();
    break;
  default: // Invalid argument; do nothing
    break;
  }
}

void eraseInline(uint8_t n) {
  char pos_x = currentLineChars;

  // BUG: when cursor is on the last character of the last line, writing currentLineChars
  // number of spaces will currently clear the screen due to the way writeCharToLCD handles
  // text wrapping. Specifically after the last character (bottom right) of the screen is
  // written the screen is automatically cleared.
  switch (n) {
  case 0: // Clear from cursor to end of line
    for (uint8_t i = 0; i < (LCD_CHARACTERS_PER_LINE - currentLineChars); i++)
      writeCharToLCD(' ');

    // Move cursor back one char and line so its in its original position
    setCursorPosition(pos_x, currentLineNum);
    break;
  case 1: // Clear from cursor to beginning of line
    writeCharToLCD('\r');
    for (uint8_t i = 0; i < pos_x; i++)
      writeCharToLCD(' ');
    moveCursorToColumn(pos_x + 1);

    break;
  case 2: // Clear entire line
    writeCharToLCD('\r');
    for (uint8_t i = 0; i < LCD_CHARACTERS_PER_LINE; i++)
      writeCharToLCD(' ');

    // Move cursor back one line so its in its original position
    setCursorPosition(pos_x + 1, currentLineNum);
  default: // Invalid argument; do nothing
    break;
  }
}

void scrollUp(uint8_t n) {
}

void scrollDown(uint8_t n) {
}

void saveCursorPosition() {
  saveCursorLineNum = currentLineNum;
  saveCursorLineChars = currentLineChars;
}

void restoreCursorPosition() {
  currentLineNum = saveCursorLineNum;
  currentLineChars = saveCursorLineChars;
  writeLCDInstr(INSTR_DDRAM_ADDR | (lineBeginnings[currentLineNum] + currentLineChars));
}

void hideCursor(void) {
  lcdState &= ~(1 << INSTR_DISPLAY_C);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

void showCursor(void) {
  lcdState |= (1 << INSTR_DISPLAY_C);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

//-----------------------------------------------------------------------------------------------
// Utility functions (with no associated ASCII or ANSI escape)

void blinkCursorOff(void) {
  lcdState &= ~(1 << INSTR_DISPLAY_B);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

void blinkCursorOn(void) {
  lcdState |= (1 << INSTR_DISPLAY_B);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

void displayOff(void) {
  lcdState &= ~(1 << INSTR_DISPLAY_D);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

void displayOn(void) {
  lcdState |= (1 << INSTR_DISPLAY_D);
  writeLCDInstr(INSTR_DISPLAY | lcdState);
}

//-----------------------------------------------------------------------------------------------

/* char readCharFromLCD(void) { */
/*   loop_until_LCD_BF_clear(); // Wait until LCD is ready for new instructions */

/*   LCD_CTRL_PORT |= (1 << LCD_RW) | (1 << LCD_RW); // RS=RW=1 */
/*   LCD_DBUS_DDR = 0; // Set all LCD_DBUS_PORT pins as inputs */
/*   clkLCD(); */

/*   char c = LCD_DBUS_PIN; */
/*   LCD_DBUS_DDR = 0xff; // Reset all LCD_DBUS_PORT pins to outputs */
/*   return c; */
/* } */


//---------------------------------------------------------------------------------------------
// Advanced functions for special cases

/*
  Initialize LCD using the internal reset circuitry.

  Note: This currently only works with 8 bit modes, but is not recommended. Instead use the
        initLCD function which uses the software initialization method and works for 8-bit
        modes as well the 4-bit mode.
 */
void initLCDByInternalReset(void) {
  enableLCDOutput();

  // Function set (8-bit interface; 2 lines with 5x7 dot character font)
  writeLCDInstr_(INSTR_FUNC_SET | (1 << INSTR_FUNC_SET_DL) | (1 << INSTR_FUNC_SET_N));

  writeLCDInstr_(0x0F);
  writeLCDInstr_(0x06);
  writeLCDInstr_(CMD_CLEAR_DISPLAY);
  _delay_ms(LCD_CLEAR_DISPLAY_DELAY);
}
