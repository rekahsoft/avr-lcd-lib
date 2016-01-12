# Flexible character LCD library for AVR Microcontrollers

* [Features](#features)
* [Tools](#tools)
* [License](#license)
* [Usage](#usage)
* [Issues](#issues)
* [Road Map](#road-map)

## Features <a name="features"></a>

- Supports both 8-bit and 4-bit modes of LCD's
- Configurable, so it should work on most devices, albeit with some tweaking
- Read/Write the character LCD
- Control the display using ANSI and ASCII escapes
- Read/Write custom characters to memory and/or the LCD display (not fully implemented)

## Tools <a name="tools"></a>

This project wouldn't be possible without the following great open source tools:

- AVR-GCC, [AVR-libc][], and AVR-binutils which form the compiler toolchain that makes open
  source development for AVR microcontrollers possible.
- [avrdude][] to do in system programming of the AVR microcontrollers
- [doxygen][] to generate documentation.
- [Gnu Emacs][], because how else would I enter text?

## License <a name="license"></a>

Code licensed under [GPLv3][]. Please see the LICENSE file for full details.

## Usage <a name="usage"></a>

Clone this repository and modify lcdLibConfig.h to your needs. See the documentation for more
details.

    git clone http://git.rekahsoft.ca/lcdLib.git
    doxygen doxygen.conf # If you want to generate offline documentation

## Issues <a name="issues"></a>

- No interface is given to read/write custom characters, though it can be done manually using
  the lower level plumbing function `writeLCDInstr`.
- No examples of using the library are currently given but do exist in the git history. They
  have been split out into their own repository's and soon will be re-added as git submodules.
  Util that time look into the git history.

## Road Map <a name="road-map"></a>

- Add support for reading/writing custom characters to memory/display
- Add examples projects utilizing lcdLib as git submodules
- Finish documenting functions
- Put the [doxygen][] generated documentation for this library online


[AVR-libc]: http://www.nongnu.org/avr-libc/
[avrdude]: http://savannah.nongnu.org/projects/avrdude
[doxygen]: http://www.stack.nl/~dimitri/doxygen/
[Gnu Emacs]: http://www.gnu.org/software/emacs/
[GPLv3]: https://www.gnu.org/licenses/gpl.html
