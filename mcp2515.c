/********************************************************************
 File: mcp2515.c

 Description:
 This file contains the MCP2515 interface functions.

 Authors and Copyright:
 (c) 2012-2016, Thomas Fischl <tfischl@gmx.de>

 Device: PIC18F14K50
 Compiler: Microchip MPLAB XC8 C Compiler V1.34

 License:
 This file is open source. You can use it or parts of it in own
 open source projects. For closed or commercial projects you have to
 contact the authors listed above to get the permission for using
 this source code.

 ********************************************************************/

#include <p18cxxx.h>
//#include "clock.h"
#include "mcp2515.h"

/** current transmit buffer priority */
unsigned char txprio = 3;

/**
 * \brief Transmit one byte over SPI bus
 *
 * \param c Character to send
 * \return Received byte
 */
unsigned char spi_transmit(unsigned char c) {
    SSPBUF = c;
    while (!SSPSTATbits.BF) {};
    return SSPBUF;
}


/**
 * \brief Write to given register
 *
 * \param address Register address
 * \param data Value to write to given register
 */
void mcp2515_write_register(unsigned char address, unsigned char data) {

    // pull SS to low level
    MCP2515_SS = 0;
   
    spi_transmit(MCP2515_CMD_WRITE);
    spi_transmit(address);
    spi_transmit(data);
   
    // release SS
    MCP2515_SS = 1;
}


/**
 * \brief Initialize spi interface, reset the MCP2515 and activate clock output signal
 */
void mcp2515_init() {

    unsigned char dummy;

    // init SPI
    SSPSTAT = 0x40; // CKE=1
    SSPCON1 = 0x21; // 3MHz SPI clock
    dummy = SSPBUF; // dummy read to clear BF
    dummy = 0;

    TRISBbits.TRISB4 = 1; // set TRIS of SDI
    TRISCbits.TRISC6 = 0; // clear TRIS of SS
    TRISCbits.TRISC7 = 0; // clear TRIS of SDO
    TRISBbits.TRISB6 = 0; // clear TRIS of SCK
    LATCbits.LATC6 = 1; // SS

    while (++dummy) {};

    // reset device
    LATCbits.LATC6 = 0; // SS
    spi_transmit(0xC0); // reset device
    LATCbits.LATC6 = 1; // SS

    while (++dummy) {};

    mcp2515_write_register(MCP2515_REG_CANCTRL, 0x85); // set config mode, clock prescaling 1:2 and clock output

    // configure filter
    mcp2515_write_register(MCP2515_REG_RXB0CTRL, 0x00); // use filter for standard and extended frames
    mcp2515_write_register(MCP2515_REG_RXB1CTRL, 0x00); // use filter for standard and extended frames

    // initialize filter mask
    mcp2515_write_register(MCP2515_REG_RXM0SIDH, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM0SIDL, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM0EID8, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM0EID0, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM1SIDH, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM1SIDL, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM1EID8, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM1EID0, 0x00);

    mcp2515_write_register(MCP2515_REG_CANINTE, 0x03); // RX interrupt

}

