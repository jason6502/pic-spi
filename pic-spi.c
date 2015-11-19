/*
 * File:   pic-spi.c
 *
 * Copyright 2015 by Jason Burke
 *
 * This file is part of PIC-SPI
 *
 * PIC-SPI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PIC-SPI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pic-spi.c.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * 8-bit Bus Interface to SPI Peripherals on Microchip PIC24FJ32GA002 MCU
 *
 * PIN NAME     DESCRIPTION
 * ---------------------------------------------------------------------
 *  1  MCLRn    Reserved for programming/debugging
 *  2  RA0      (AN0) Chip Select 0 (active low)
 *  3  RA1      (AN1) Chip Select 1 (active low)
 *  4  RB0      (RP0) PGD (Chip Select 2 when development complete)
 *  5  RB1      (RP1) PGC (Chip Select 3 when development complete)
 *  6  RB2      (RP2) SDO (SPI serial data output)
 *  7  RB3      (RP3) SDI (SPI serial data input)
 *  8  VSS      Connect to ground
 *  9  RA2      IRQ output (open drain)
 * 10  RA3      (PMA0) A0
 * 11  RB4      (RP4) SCK (SPI clock)
 * 12  RA4      (PMA1) A1
 * 13  VDD      Connect to +3.3V
 * 14  RB5      *(RP5, PMD7) D7 (parallel I/O)
 * 15  RB6      *(RP6, PMD6) D6 (parallel I/O)
 * 16  RB7      *(RP7, PMD5) D5 (parallel I/O)
 * 17  RB8      *(RP8, PMD4) D4 (parallel I/O)
 * 18  RB9      *(RP9, PMD3) D3 (parallel I/O)
 * 19  DISVR    Connect to ground to enable on-board regulator
 * 20  VCAP     10uF or higher low-ESR capacitor to ground
 * 21  RB10     *(RP10, PMD2) D2 (parallel I/O)
 * 22  RB11     *(RP11, PMD1) D1 (parellel I/O)
 * 23  RB12     (RP12, PMD0) D0 (parallel I/O)
 * 24  RB13     (RP13. PMRD) RD
 * 25  RB14     (RP14, PMWR) WR
 * 26  RB15     (RP15, PMCS) CS
 * 27  AVSS     Connect to ground
 * 28  AVDD     Connect to +3.3V
 * (*) designates pins which are 5V tolerant
 *
 *
 * Addr Bit Dir Description             Dir Description
 * -------------------------------------------------------------------------
 *    0   7 W   Chip Enable             R   IRQ Flag
 *    0   6 W   Interrupt Enable        R   Interrupt Enable
 *    0   5 W   Reserved                R   Receive Data Available
 *    0   4 W   Fast RX Mode            R   Fast RX Mode
 *    0   3 W   Slave Select 1          R   Slave Select 1
 *    0   2 W   Slave Select 2          R   Slave Select 2
 *    0   1 W   Clock Polarity(1)       R   TX overrun error (clr on RDR read)
 *    0   0 W   Clock Phase(2)          R   RX overrun error (clr on RDR read)
 *    1   7 W   TDR Bit 0               R   RDR Bit 0
 *    1   6 W   TDR Bit 1               R   RDR Bit 1
 *    1   5 W   TDR Bit 2               R   RDR Bit 2
 *    1   4 W   TDR Bit 3               R   RDR Bit 3
 *    1   3 W   TDR Bit 4               R   RDR Bit 4
 *    1   2 W   TDR Bit 5               R   RDR Bit 5
 *    1   1 W   TDR Bit 6               R   RDR Bit 6
 *    1   0 W   TDR Bit 7               R   RDR Bit 7
 *    2 7-3 W   Clock Prescaler         R   Clock Prescaler
 *    2 2-0 W   Reserved                R   Reserved (FIFO count?)
 *    3 7-0 W   Reserved                R   Reserved
 *
 * (1) idle state of clock (0=low)
 * (2) SDO on idle to active (0) or active to idle (1)
 * 
 * Clock Prescaler Table (bits 4-0 of address 3):
 * ----------------------------------------------
 *
 * Value        Clock Rate
 * --------------------------
 * 00000          31.3 kHz
 * 00001          35.7 kHz
 * 00010          41.7 kHz
 * 00011          50.0 kHz
 * 00100          62.5 kHz
 * 00101          83.3 kHz
 * 00110         125.0 kHz
 * 00111         250.0 kHz
 * 01000         125.0 kHz
 * 01001         142.9 kHz
 * 01010         166.6 kHz
 * 01011         200.0 kHz
 * 01100         250.0 kHz
 * 01101         333.3 kHz
 * 01110         500.0 kHz
 * 01111        1000.0 kHz
 * 10000         500.0 kHz
 * 10001         571.5 kHz
 * 10010         666.7 kHz
 * 10011         800.0 kHz
 * 10100        1000.0 kHz
 * 10101        1333.3 kHz
 * 10110        2000.0 kHz
 * 10111        4000.0 kHz
 * 11000        2000.0 kHz
 * 11001        2285.7 kHz
 * 11010        2666.7 kHz
 * 11011        3200.0 kHz
 * 11100        4000.0 kHz
 * 11101        5333.3 kHz
 * 11110        8000.0 kHz
 * 11111        INVALID
 *
 * TODO:
 */

// Chip Configuration
#pragma config POSCMOD = NONE       // primary oscillator disabled
#pragma config I2C1SEL = PRI        // use default SCL1/SDA1 pins
#pragma config IOL1WAY = OFF        // IOLOCK may be changed via unlock sequence
#pragma config OSCIOFNC = ON        // CLKO functions as port I/O
#pragma config FCKSM = CSDCMD       // clock switching & fail-safe clk mon disabled
#pragma config FNOSC = FRCPLL       // fast RC oscillator with 4x PLL selected
#pragma config SOSCSEL = SOSC       // default secondary oscillator
#pragma config WUTSEL = LEG         // legacy wake-up timer
#pragma config IESO = OFF           // two speed start-up disabled
#pragma config WDTPS = PS256        // watchdog timer postscaler
#pragma config FWPSA = PR128        // watchdog timer prescaler
#pragma config WINDIS = ON          // standard watchdog timer selected (no window)
#pragma config FWDTEN = OFF         // watchdog timer disabled
#pragma config ICS = PGx1           // Emu EMUC1/EMUD1 pins shared with PGC1/PGD1
#pragma config COE = OFF            // reset into operational mode
#pragma config BKBUG = OFF          // device resets into operational mode
#pragma config GWRP = OFF           // writes to program memory are allowed
#pragma config GCP = OFF            // code protection is disabled
#pragma config JTAGEN = OFF         // JTAG port is disabled

#ifndef FCY
#define FCY     16000000            // FCY = FOSC / 2
#endif

// Standard includes
#include <xc.h>
#include <libpic30.h>               // __delay() macros
#include <stdint.h>

// Typedefs
typedef struct {
    uint8_t ctrl;                   // SPI control register (write only)
    uint8_t status;                 // SPI status register (read only)
    uint8_t rdr;                    // SPI receive data register (read only)
    uint8_t ctrl2;                  // SPI clock prescaler (read/write)
} regs_spi_t;

// Program constants
const uint8_t DEF_PRESCALE = 0b10100;   // Default SPI clock is 1 MHz

// Program globals
volatile regs_spi_t regs_spi;

////////////////////////////////////
// Configure hardware peripherals //
////////////////////////////////////
static void initHardware(void)
{
    // Configure PIC Oscillator
    // RC oscillator runs at 8 MHz.
    // If PLL is enabled, output frequency is multiplied by four
    CLKDIVbits.RCDIV = 0b000;               // Disable postscaler

    // Can be used to tweak the oscillator if desired
    //OSCTUNbits.TUN = 0b000000;

    // Wait for PLL to lock
    while (OSCCONbits.LOCK == 0)
        ;

    // Interrupt configuration
    INTCON1bits.NSTDIS = 1;         // disable nested interrupts

    // Configure pins as needed by application
    OSCCONL = 0x46;                 // unlock sequence 1
    OSCCONL = 0x57;                 // unlock sequence 2
    OSCCONbits.IOLOCK = 0;          // unlock RPINRx/RPORx registers
    RPOR2bits.RP4R = 8;             // map SPI1 SCK to pin 11 (RP4)
    RPOR1bits.RP2R = 7;             // map SPI1 SDO to pin 6 (RP2)
    RPINR20bits.SDI1R = 3;          // map SPI1 SDI to pin 7 (RP3)
    OSCCONL = 0x46;                 // unlock sequence 1
    OSCCONL = 0x57;                 // unlock sequence 2
    OSCCONbits.IOLOCK = 1;          // lock RPINRx/RPORx registers

    // Configure the PMP (Parallel Master Port)
    PMCONbits.PSIDL = 0;            // run in idle mode
    PMCONbits.ADRMUX = 0b00;        // address and data separate
    PMCONbits.PTBEEN = 0;           // byte enable port DISABLED
    PMCONbits.PTWREN = 1;           // PMWR port enabled (on in slave mode)
    PMCONbits.PTRDEN = 1;           // PMRD port enabled (on in slave mode)
    PMCONbits.CSF = 0b10;           // PMCS1 functions as chip select
    PMCONbits.CS1P = 0;             // set chip select polarity to active low
    PMCONbits.WRSP = 0;             // set PMWR strobe polarity to active low
    PMCONbits.RDSP = 0;             // set PMRD strobe polarity to active low
    PMMODEbits.IRQM = 0b01;         // PMP intrs generated at end of R/W cycle
    PMMODEbits.INCM = 0b00;         // auto-{inc,dec}rement disabled
    PMMODEbits.MODE16 = 0;          // 8-bit mode
    PMMODEbits.MODE = 0b01;         // select enhanced PSP mode
    PMMODEbits.WAITB = 0b00;        // wait state config bits
    PMMODEbits.WAITE = 0b00;        // read to byte enable wait state config
    PMAENbits.PTEN14 = 1;           // PMCS1 strobe enable bit
    PMAENbits.PTEN1 = 1;            // PMA1 strobe enable bit
    PMAENbits.PTEN0 = 1;            // PMA0 strobe enable bit
    IFS2bits.PMPIF = 0;             // clear PMP interrupt flag
    IPC11bits.PMPIP = 6;            // set PMP interrupt priority level
    IEC2bits.PMPIE = 1;             // enable PMP interrupts
    PMCONbits.PMPEN = 1;            // enable PMP

    // Configure SPI Module
    SPI1CON1bits.DISSCK = 0;        // Internal SPI1 clock is enabled
    SPI1CON1bits.DISSDO =  0;       // Internal SDO1 output is enabled
    SPI1CON1bits.MODE16 = 0;        // Communications is byte-wide
    SPI1CON1bits.SSEN = 0;          // Slave select enable bit
    SPI1CON1bits.MSTEN = 1;         // Master mode enable bit
    SPI1CON2bits.FRMEN = 0;         // Framed SPI1 support is disabled
    SPI1CON2bits.SPIFSD = 0;        // SPI1 frame sync pulse direction
    SPI1CON2bits.SPIFPOL = 0;       // SPI1 frame sync pulse polarity
    SPI1CON2bits.SPIFE = 0;         // SPI1 frame sync pulse edge select bit
    SPI1CON2bits.SPIBEN = 0;        // SPI1 enhanced buffer enable bit
    SPI1STATbits.SPISIDL = 0;       // SPI1 stop in idle mode bit
    SPI1STATbits.SPIROV = 0;        // Clear SPI1 overflow flag
    IFS0bits.SPI1IF = 0;            // clear SPI1 interrupt flag
    IPC2bits.SPI1IP = 5;            // set SPI interrupt priority level
    IEC0bits.SPI1IE = 1;            // enable SPI1 interrupts
    SPI1CON1bits.SMP = 0;           // SPI1 data input sample phase (middle)

    // Configure analog inputs
    AD1PCFG = 0b1001111000111111;   // disable all A/D ports

    // Configure PORTA
    // Ports RA0 and RA1 are chip select outputs
    // Port RA2 is the IRQ output
    LATAbits.LATA0 = 1;             // set initial value of RA0 output high
    LATAbits.LATA1 = 1;             // set initial value of RA1 output high
    LATAbits.LATA2 = 1;             // set initial value of RA2 output high
    ODCAbits.ODA2 = 1;              // config port RA2 for open-drain output
    TRISA = 0b11000;                // config RA0-RA2 as outputs

    // Configure PORTB
    // Ports B0 and B1 are chip select outputs
    LATBbits.LATB0 = 1;             // set initial value of RB0 output high
    LATBbits.LATB1 = 1;             // set initial value of RB1 output high
    TRISB = 0b1111111111111100;     // config RB0-RB1
}

///////////////////////////////////////////////////////////////
// SPI1 ISR
///////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv, shadow)) _SPI1Interrupt(void)
{
    // A new character is available.  If the receive data register is
    // not already full, copy the new character to it.
    if ((regs_spi.status&0x20) == 0) {
        regs_spi.rdr = SPI1BUF;
    }

    // Disable chip select on active SPI peripheral
    switch (regs_spi.ctrl&0x0c) {
        case 0:
            LATAbits.LATA0 = 1;         // active low
            break;
        case 4:
            LATAbits.LATA1 = 1;         // active low
            break;
        case 8:
            LATBbits.LATB0 = 1;         // active low
            break;
        case 12:
            LATBbits.LATB1 = 1;         // active low
            break;
    }

    // Update the following bits in the status register:
    // b7 - IRQ state
    // b5 - receive data available flag
    // b0 - receive overrun error flag
    regs_spi.status &= 0b01011110;
    regs_spi.status |= ((regs_spi.ctrl&0x40)<<1) | 0x20 | SPI1STATbits.SPIROV;

    // Clear SPI1 interrupt flag
    IFS0bits.SPI1IF = 0;
}

///////////////////////////////////////////////////////////////
// PMP ISR
///////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv, shadow)) _PMPInterrupt(void)
{
    uint8_t ctrl;

    // Process writes to address 0 (control register) first
    if (PMSTATbits.IB0F) {
        // read in new value for control register
        ctrl = PMDIN1;

        // Did anything change?
        if (regs_spi.ctrl != ctrl) {
            // Update SPI1 configuration
            SPI1CON1bits.CKE = ctrl&0x1;        // set clock phase
            SPI1CON1bits.CKP = (ctrl&0x2)>>1;   // set clock polarity
            SPI1CON1bits.PPRE = (regs_spi.ctrl2&0xc0)>>6;
            SPI1CON1bits.SPRE = (regs_spi.ctrl2&0x38)>>3;
            SPI1STATbits.SPIEN = (ctrl&0x80)>>7;// enable/disable SPI port

            // Update the control register
            regs_spi.ctrl = ctrl;
        }
    }
    // Process writes to address 1 (transmit data register)
    else if (PMSTATbits.IB1F) {
        // Clear any existing interrupts on writes to the TDR
        LATAbits.LATA2 = 1;

        // Mask off bits in the status register to be cleared or updated
        // b7 - clear IRQ state
        // b1 - clear TX overrun error bit
        regs_spi.status &= 0b01111101;

        // If the SPI transmit buffer is not full, copy the new value
        // to the transmit buffer (which will begin the transmission)
        // otherwise set TX overrun error bit.
        if (SPI1STATbits.SPITBF == 0) {
            // Enable chip select on active SPI peripheral
            switch (regs_spi.ctrl&0x0c) {
                case 0:
                    LATAbits.LATA0 = 0;     // active low
                    break;
                case 4:
                    LATAbits.LATA1 = 0;     // active low
                    break;
                case 8:
                    LATBbits.LATB0 = 0;     // active low
                    break;
                case 12:
                    LATBbits.LATB1 = 0;     // active low
                    break;
            }
            SPI1BUF = PMDIN1 >> 8;
        }
        else {
            regs_spi.status |= 0x02;
        }

        // Update PMDOUT1
        PMDOUT1 = (uint16_t)regs_spi.status | ((uint16_t)regs_spi.rdr << 8);
    }

    // Process reads from address 1 (receive data register)
    if (PMSTATbits.OB1E) {
        // Clear any existing interrupts on reads from the RDR
        LATAbits.LATA2 = 1;

        // Are we in Fast RX mode?
        if (regs_spi.ctrl&0x10) {
            // Update three bits in the status register (b7, b1, b0):
            // b7 = clear IRQ state
            // b1 = update TX overrun error bit
            // b0 = clear RX overrun error bit
            regs_spi.status &= 0b01111100;

            // Initiate another SPI transfer
            if (SPI1STATbits.SPITBF == 0) {
                // Enable chip select on active SPI peripheral
                switch (regs_spi.ctrl&0x0c) {
                    case 0:
                        LATAbits.LATA0 = 0;     // active low
                        break;
                    case 4:
                        LATAbits.LATA1 = 0;     // active low
                        break;
                    case 8:
                        LATBbits.LATB0 = 0;     // active low
                        break;
                    case 12:
                        LATBbits.LATB1 = 0;     // active low
                        break;
                }

                // Initiate transfer
                SPI1BUF = 0;
            }
            else {
                regs_spi.status |= 0x02;    // set TX overflow flag
            }
        }
        else {
            // Update two bits in the status register (b7, b0):
            // b7 - clear IRQ state
            // b0 - clear RX overrun error bit
            regs_spi.status &= 0b01111110;
        }

        // Update PMDOUT1 (also clears the OB1E flag)
        PMDOUT1 = (uint16_t)regs_spi.status | ((uint16_t)regs_spi.rdr << 8);

        // Clear the SPI1 overflow flag
        SPI1STATbits.SPIROV = 0;
    }

    // Process writes to address 2 (control register 2)
    if (PMSTATbits.IB2F) {
        // Read in new value for low byte of control register 2
        regs_spi.ctrl2 &= 0xff00;
        regs_spi.ctrl2 |= PMDIN2 & 0xff;
    }
    // Process writes to address 3 (reserved)
    else if (PMSTATbits.IB3F) {
        // Read in new value for address 3
        regs_spi.ctrl2 &= 0x00ff;
        regs_spi.ctrl2 |= PMDIN2 & 0xff00;
    }

    // Clear input buffer overflow status bit (otherwise an overflow will
    // prevent PMDIN1 and PMDIN2 from being updated by further writes).
    PMSTATbits.IBOV = 0;

    // Clear PMP interrupt flag
    IFS2bits.PMPIF = 0;
}

int main(void)
{
    uint16_t temp;

    initHardware();                 // initialize PIC ports & peripherals

    // Set initial values of variables
    regs_spi.ctrl = 0;              // Default control register
    regs_spi.ctrl2 = DEF_PRESCALE;  // SPI clock defaults to 1 MHz
    regs_spi.status = 0;            // Initial contents of status register
    regs_spi.rdr = 0;               // Initial contents of data register

    // Event Dispatch Loop
    for (;;) {
        // BEGIN CRITICAL SECTION
        __builtin_disi(0x3fff);

        // Copy spi status register & rdr to PMP output port
        temp = (uint16_t)regs_spi.status | ((uint16_t)regs_spi.rdr << 8);
        if ((temp != PMDOUT1) && (PMSTATbits.OB1E == 0)) {
            PMDOUT1 = temp;

            // Set IRQ pin to be the inverse of b7 of regs_spi.status
            // (this triggers an interrupt on a connected CPU)
            LATAbits.LATA2 = (regs_spi.status&0x80) ? 0 : 1;
        }

        // END CRITICAL SECTION
        __builtin_disi(0);

        // Copy spi control register 2 to PMP output port
        PMDOUT2 = regs_spi.ctrl2;
    }
}