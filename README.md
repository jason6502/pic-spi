# pic-spi
SPI Bus Peripheral for 8-Bit Microprocessor Bus

With the successful completion and testing of the PIC-UART project on an old 8-bit CPU (65C02), it became apparent
that the PMP interface on the PIC could be used as a bridge between older microprocessors and new peripherals not
only for UARTs, but also for newer and faster serial interfaces.  The Serial Peripheral Interface (SPI) is a popular
standard created by Motorola that can be used for much higher speed applications than a typical UART supports and owing to
its synchronous nature, is easier to interface to as well.

The same PIC24 was chosen for this project as was used for the PIC-UART project -- a PIC24FJ32GA002.  It comes in a
28-pin DIP package.  Other PICs in the same family could be used with only minor changes to the source code.  The
source code is based heavily on the work already done for the PIC-UART.  However, rather than modeling the interface
after the MC6850, it was instead modeled after the 65SPI (http://sbc.rictor.org/io/65spi.html).  Due to some
significant differences in hardware, the interface only approximates the 65SPI.  However, porting efforts for code
that already works with the 65SPI should be minimal.

Since all PICs with the addressable PMP port required to implement the microprocessor bus used here are only
available for 3.3V (or lower) operation, it may be necessary to perform voltage conversion when interfacing with
a 5V only microprocessor.  Fortunately, most modern SPI peripherals are designed to run at 3.3V, so this conversion
should only be necessary for interfacing to the microprocessor bus.  There are a number of solutions that could be
used for voltage conversion.  One that I myself make use of is the 74LVX4245 level converting transceiver mounted
on a SOIC-to-DIP adapter.

This is an initial check-in of the code and no testing has been performed other than to insure that it compiles
without error.  Although the code is based on working PIC-UART code, there is no guarantee that it will function
correctly as-is.  I will begin testing it as time allows and check in any bug fixes and enhancements as I find
them.

In terms of operating limitations, the maximum SPI clock rate supported by the PIC used in this project is 8 MHz.
Additionally, based on experiments with the PIC-UART, I would project that with a 6502-based system, the maximum
bus frequency that could be used will be around 5 MHz before a wait state is needed.  A single wait-state should
be sufficient to run up to 14+ MHz, however.  Finally, while there should be no speed limitations involved when
using interrupt-driven I/O, should polled I/O be used I expect that there could be some latency issues if fast
polling is combined with a high-speed bus.  However, I don't expect that to be an issue up to bus frequencies of
approximately 8 MHz.  Further testing will be required in order to accurately determine the operating limitations,
however.

Bug fixes, comments, and suggestions are welcome.

See https://github.com/jason6502/pic-spi/wiki for documentation of the PIC-SPI.
See http://forum.6502.org/viewtopic.php?f=4&t=3511 for more details and ongoing discussion related to this project.
