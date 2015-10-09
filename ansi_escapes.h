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
 * File: ansi_escapes.h
 * Author: Collin J. Doering <collin.doering@rekahsoft.ca>
 * Date: Oct 9, 2015
 */

#define CSI "\e["

#define CUU(n) CSI #n "A"           // Cursor up
#define CUD(n) CSI #n "B"           // Cursor down
#define CUF(n) CSI #n "C"           // Cursor forward
#define CUB(n) CSI #n "D"           // Cursor backward

#define CNL(n) CSI #n "E"           // Cursor next line
#define CPL(n) CSI #n "F"           // Cursor previous line

#define CHA(n) CSI #n "G"           // Cursor horizontal absolute
#define CUP(n,m) CSI #n ";" #m "H"  // Cursor position

#define ED(n) CSI #n "J"            // Erase display
#define EL(n) CSI #n "K"            // Erase in line
#define SU(n) CSI #n "S"            // Scroll up
#define SD(n) CSI #n "T"            // Scroll down

#define HVP(n,m) CSI #n ";" #m "f"  // Horizontal and vertical position
