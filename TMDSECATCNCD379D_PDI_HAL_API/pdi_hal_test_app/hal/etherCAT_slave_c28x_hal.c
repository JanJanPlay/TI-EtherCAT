//#############################################################################
//
// FILE:   ethercat_slave_c28x_hal.c
//
// TITLE:  C28x HAL level functions for EtherCAT slave controller (ESC)
//
//! \addtogroup C2k_EtherCAT_adapater_examples_list
//! <h1> PDI Interface test Example </h1>
//!
//! The functions in this file provide a HAL layer interface for EtherCAT slave
//! applications that can be built on C2k EtherCAT adapter board
//!
//! The HAL could be EMIF or SPI based depending on the PDI interface chosen
//! or configured for the C2k ET1100 EtherCAT adapater board
//!
//!-----------------------------------------------------------------------------
//! C28x Addressing vs. TwinCat3 software addresses for EMIF PDI
//!   The C28x address used below is a WORD (16b) address for the ET1100 PDI
//!   interface, while Beckhoff EtherCAT documentation and TwinCat3 software use
//!   BYTE addresses. The USER RAM on the ET1100 starts at 0x1000 offset BYTE
//!   address. Divide this by 2 to get the correct 16b word offset from EMIF2
//!   start address
//!
//!  EMIF PDI reads two bytes at a time from ET1100 address space, that is the
//!  minimum data size thats readable by C28x CPU
//!------------------------------------------------------------------------------
//!------------------------------------------------------------------------------
//! C28x Addressing vs. TwinCat3 software addresses for SPI PDI
//!   For the SPI PDI the addressing of ET1100 memory space is straight forward
//!   The SPI PDI uses 8 bit character length for SPI reads/writes but the HAL API
//!   is adjusted to read 16bits at a time to be consistent with the EMIF PDI.
//!   Users can modify the SPI PDI to read/write one byte at a time from ET1100
//!   address space. But since C28x data bus is 16 bit wide this example shows
//!   16bit SPI PDI reads/writes as well.
//!
//!   SPI MISO pin read for error status on the last transaction is not supported
//!      in this HAL
//!   SPI MODE 3 is supported by this HAL
//!   I0 and I1 byte reads on the MISO pin are ignored in this HAL
//!------------------------------------------------------------------------------
//! \b External \b Connections \n
//!  Users can connect a PC running TWINCAT3 to the Ethercat Slave and view the
//!  memory window of ET1100 for both registers and ET1100 RAM
//!
//! \b Watch \b Variables \n
//!  - escRegs data structure is filled in with some ET1100 registers which can be
//!    viewed in memory window if HAL Test is enabled.
//!
//!
////###########################################################################
// $TI Release: C2000 EtherCAT solutions support v1.00 $
// $Release Date: 07/2017 $
// $Copyright:
// Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_spi.h"
#include "ethercat_slave_c28x_hal.h"

//
// Defines
//
//#define ETHERCAT_STACK  1   //only use if EtherCAT slave stack is used

//
//globals
//

//Pointer to the ESC memory, initialized in the ESC_initHW()
//used only for the ASYNC16 (EMIF1 or EMIF2) PDI
uint16_t *pEsc;

#ifdef PDI_HAL_TEST
//Debug array to log esc registers, used only for PDI HAL TEST
esc_et1100_regs_t escRegs[ESC_DEBUG_REGS_LENGTH];
#endif

#ifdef INTERFACE_SPI
// SPI Variables
volatile uint16_t   SPI_RxData[16];     // Receive data buffer
volatile uint16_t   SPI_XmitInProgress;
#endif


//
//HAL level functions
//
#ifndef __cplusplus

#ifdef INTERFACE_SPI
#pragma CODE_SECTION(ESC_readSPI, ".TI.ramfunc");
#pragma CODE_SECTION(ESC_writeSPI, ".TI.ramfunc");
#endif
#pragma CODE_SECTION(ESC_timerIncPerMilliSec, ".TI.ramfunc");
#pragma CODE_SECTION(ESC_getTimer, ".TI.ramfunc");
#endif

/***********************************************************************************/

uint32_t ESC_getTimer(void)
{
    //C28x timer is decrements from 0xFFFFFFFF while the stack understands it as of 
    //increment type. 
    return ~ ((uint32_t)(CpuTimer0Regs.TIM.all));
}
/***********************************************************************************/
void ESC_clearTimer(void)
{
    CpuTimer0Regs.TIM.all = 0;
}
/***********************************************************************************/
uint32_t ESC_timerIncPerMilliSec(void)
{
    return (uint32_t) 200000UL; //at 200MHz
}
/***********************************************************************************/
#ifdef INTERFACE_SPI
//
// SPI HAL functions for EtherCAT slave stack
//

//SPI peripehral register pointer, will be initialized depending on the SPI chosen as per
// build configurations
volatile struct SPI_REGS *SpixRegs;

/*****************************************************************************************
 * @fn          ESC_readSPI
 * @brief       function reads up to 12 bytes of data
 *
 * @param
 *     offset_addr  - ESC address from which data has to be read
 *     numbytes     - number of bytes to be read, limited to 12 at a time by the caller
 *     buffer       - pointer to the buffer where read data has to be copied to
 *                  - if a NULL is passed then data is copied to SPI_RxData global array
 *
 * @return          - None
 ****************************************************************************************/
void ESC_readSPI(uint16_t offset_addr,uint16_t numbytes, uint16_t* buffer)
{

    uint16_t i,cmd, readval, numwords = 0, j;
    uint16_t *buf = (uint16_t *)0;
    uint16_t readphase[16];


    if(((void *)buffer) == NULL)
    {
        buf = (uint16_t *)&SPI_RxData[0];
    }
    else
    {
        buf = buffer;
    }

    // Construct Address cmd bytes into 16-bit words for SPI xmission,
    //    SPI xmits MSBit 1st, so must swap bytes in this 16b word for transmission
    // Byte order of READ cmd sequence:
    // Byte 0: A[12:5]
    // Byte 1: A[4:0], 110b             (110b is 3-byte cmd extension)
    // Byte 2: A[15:13], CMD[2:0], 00b  (011b is READ w/ WS)
    // Byte 3: FFh                      (Wait State)
    //cmd = offset_addr & 0x1f
    cmd =(offset_addr & 0x1FE0)<<3; // offset_addr[12:5] is 1st address phase byte, shift to upper byte
    cmd |= (((offset_addr & 0x1F) << 3) | ESC_ETHERCAT_3BYTEADDR);
    readphase[0] = cmd;
    numwords++;
    readphase[1] = (offset_addr & 0xE000) | (ESC_ETHERCAT_READ_WAIT <<10) | ESC_ETHERCAT_WAIT;
    numwords++;

    for(i=2, j = 0; j<numbytes ; i++){
        readphase[i] = (uint16_t) ESC_ETHERCAT_CONTINUE;
        numwords++;
        j++;
        j++;
    }
    readphase[--i] |= (ESC_ETHERCAT_RDTERMINATE); // set last byte as 0xFF

#ifdef USE_SPIA
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
        asm(" NOP");    //need 15ns delay
        asm(" NOP");
        asm(" NOP");
        asm(" NOP");
#endif

    for(i = 0;i < numwords;i++){
        SpixRegs->SPITXBUF = readphase[i] & (0xFF00);
        SpixRegs->SPITXBUF = ((readphase[i] & (0xFF)) << 8);
    }
    SPI_XmitInProgress=1;

    DELAY_US(10);

    j = numbytes;

    while(SpixRegs->SPIFFRX.bit.RXFFST < (numwords))
    {
        asm(" NOP");    //need 12ns delay
        asm(" NOP");
        asm(" NOP");
    };
    while(SpixRegs->SPIFFRX.bit.RXFFST != (numbytes))
    {
        ////ignore first two words (4 bytes)
        readval = SpixRegs->SPIRXBUF; //ignore

    }

    for(i=0;((SpixRegs->SPIFFRX.bit.RXFFST != 0));i++)
    {
        readval = (SpixRegs->SPIRXBUF) & 0xFF;
        buf[i]= (readval & 0xFF);

        readval = (SpixRegs->SPIRXBUF) & 0xFF;
        buf[i] |= ((readval & 0xFF) << 8);


    }
    DELAY_US(5);
    SpixRegs->SPIFFTX.bit.TXFIFO=0;     // Reset Tx FIFO
    SpixRegs->SPIFFRX.bit.RXFIFORESET = 0; //reset the FIFO pointer
    DELAY_US(2);
    SpixRegs->SPIFFTX.bit.TXFIFO=1;     // Reenable Tx FIFO
    SpixRegs->SPIFFRX.bit.RXFIFORESET = 1; //reenable the FIFO operation
#ifdef USE_SPIA
        GpioDataRegs.GPASET.bit.GPIO22 = 1;
#endif

    SPI_XmitInProgress=0;
}


#define FIFO_LENGTH     12
/*****************************************************************************************
 * @fn          ESC_writeSPI
 * @brief       function writes up to 12 bytes of data
 *
 * @param
 *     offset_addr  - ESC address to which data has to written to
 *     numbytes     - number of bytes to be written to, limited to 12 at a time by the caller
 *     wrdata       - pointer to the buffer from where data has to be written to ESC
 *
 *
 * @return       - none
 ****************************************************************************************/
void ESC_writeSPI(uint16_t offset_addr,uint16_t *wrdata, uint16_t numbytes)
{
    uint16_t i, j,cmd, numwords = 0;
    uint16_t wptr = 0;
    uint16_t writephase[2];


    // Construct Address cmd bytes into 16-bit words for SPI xmission,
    //    SPI xmits MSBit 1st, so must swap bytes in this 16b word for transmission
    // Byte order of READ cmd sequence:
    // Byte 0: A[12:5]
    // Byte 1: A[4:0], 110b             (110b is 3-byte cmd extension)
    // Byte 2: A[15:13], CMD[2:0], 00b  (110b is 3-byte cmd extension)
    // Byte 3: Afirst byte of data
    //cmd = offset_addr & 0x1f
    cmd =(offset_addr & 0x1FE0)<<3; // offset_addr[12:5] is 1st address phase byte, shift to upper byte
    cmd |= (((offset_addr & 0x1F) << 3) | ESC_ETHERCAT_3BYTEADDR);
    writephase[0] = cmd;
    numwords++;
    cmd = 0x0000;
    cmd = (((offset_addr & 0xE000) | (ESC_ETHERCAT_WRITE <<10))) ;
    cmd |= (wrdata[wptr] & 0x00FF);
    writephase[1] = cmd;
    numwords++;
#ifdef USE_SPIA
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
        asm(" NOP");    //need 15ns delay
        asm(" NOP");
        asm(" NOP");
        asm(" NOP");
        asm(" NOP");
#endif

    i = 0;
    {
        SpixRegs->SPITXBUF = writephase[i] & (0xFF00);
        SpixRegs->SPITXBUF = ((writephase[i++] & (0xFF)) << 8);
        SpixRegs->SPITXBUF = writephase[i] & (0xFF00);
        SpixRegs->SPITXBUF = ((writephase[i] & (0xFF)) << 8);
    }
    wptr=0;
    for(j = 1; j < (numbytes-1) ; j+=2)
    {
        SpixRegs->SPITXBUF = ((wrdata[wptr]) & 0xFF00);
        SpixRegs->SPITXBUF= (((wrdata[++wptr]) & 0x00FF) << 8);
        numwords++;
    }
    if(j == (numbytes-1))
    {
        SpixRegs->SPITXBUF = ((wrdata[wptr]) & 0xFF00);
        numwords++;
    }

    SPI_XmitInProgress=1;
    while(SpixRegs->SPIFFTX.bit.TXFFST != 0)
    {
        DELAY_US(2);
    };
    SpixRegs->SPIFFTX.bit.TXFIFO=0;     // Reset Tx FIFO
    SpixRegs->SPIFFRX.bit.RXFIFORESET = 0; //reset the FIFO pointer
    DELAY_US(2);
    SpixRegs->SPIFFTX.bit.TXFIFO=1;     // Reset Tx FIFO
    SpixRegs->SPIFFRX.bit.RXFIFORESET = 1; //reenable the FIFO operation
    SPI_XmitInProgress=0;

#ifdef USE_SPIA
        GpioDataRegs.GPASET.bit.GPIO22 = 1;
#endif

}
/***********************************************************************************/
uint16_t ESC_readWordNonISR(uint16_t offset_addr)
{
    uint16_t data;
    DINT;
    ESC_readSPI(offset_addr, 2, &data);
    EINT;
    return  data;
}
/***********************************************************************************/
uint16_t ESC_readWordISR(uint16_t offset_addr)
{
    ESC_readSPI(offset_addr, 2, 0);
    return  (SPI_RxData[0]);
}
/***********************************************************************************/

uint32_t ESC_readDWordNonISR(uint16_t offset_addr)
{
    uint32_t dword;

    DINT;
    ESC_readSPI(offset_addr, 4, (uint16_t *)&dword);
    EINT;
    return dword;
}
/***********************************************************************************/
uint32_t ESC_readDWordISR(uint16_t offset_addr)
{
    uint32_t dword;

    ESC_readSPI(offset_addr, 4, (uint16_t *)&dword);

    return dword;
}
/***********************************************************************************/
void ESC_readBlockISR(uint16_t* pData, uint16_t offset_addr,uint16_t numbytes)
{
    uint16_t i = 0, current_bytes = 0, last_byte = 0;
    if(numbytes & 0x1)
    {
        current_bytes = (numbytes - 0x1); // even align
    }
    else
    {
        current_bytes = numbytes;
    }
    while(current_bytes > 0) // input is actually in bytes
    {
        if( current_bytes >= FIFO_LENGTH)
        {
            ESC_readSPI(offset_addr, FIFO_LENGTH, (uint16_t *) &pData[i]);
            current_bytes -= FIFO_LENGTH;
            i+= FIFO_LENGTH/2; // data is in 16 bits
            offset_addr += FIFO_LENGTH;
        }
        else
        {
            ESC_readSPI(offset_addr, current_bytes, (uint16_t *) &pData[i]);
            offset_addr += current_bytes;
            i+= current_bytes/2;
            current_bytes = 0;
        }
    }

    if(numbytes & 0x1)
    {

        last_byte =  ESC_readWordISR(offset_addr);
        pData[i] = pData[i] & 0xFF00;
        pData[i] |= last_byte;

    }

}
/***********************************************************************************/
void ESC_readBlockNonISR(uint16_t* pData, uint16_t offset_addr,uint16_t numbytes)
{

    DINT;
    ESC_readBlockISR(pData, offset_addr,numbytes);
    EINT;

}

/***********************************************************************************/
void ESC_writeWordNonISR(uint16_t wrdata, uint16_t offset_addr)
{
    DINT;
    ESC_writeSPI(offset_addr, &wrdata, 0x02);
    EINT;
}
/***********************************************************************************/
void ESC_writeWordISR(uint16_t wrdata, uint16_t offset_addr)
{
    ESC_writeSPI(offset_addr, &wrdata, 0x02);
}
/***********************************************************************************/
void ESC_writeDWordNonISR(uint32_t wrdata, uint16_t offset_addr)
{
    DINT;
    ESC_writeSPI(offset_addr, (uint16_t *)&wrdata, 0x04);
    EINT;
}
/***********************************************************************************/
void ESC_writeDWordISR(uint32_t wrdata, uint16_t offset_addr)
{
    ESC_writeSPI(offset_addr, (uint16_t *)&wrdata, 0x04);
}
/***********************************************************************************/

void ESC_writeBlockISR(uint16_t* pData, uint16_t offset_addr,uint16_t numbytes)
{
    uint16_t i = 0, current_bytes = 0;

    if(numbytes & 0x1)
    {
        current_bytes = (numbytes - 0x1); // even align
    }
    else
    {
        current_bytes = (numbytes);
    }
    while(current_bytes > 0) // input is actually in bytes
    {
        if( current_bytes >= FIFO_LENGTH)
        {
            ESC_writeSPI(offset_addr, (uint16_t *) &pData[i], FIFO_LENGTH);
            current_bytes -= FIFO_LENGTH;
            i+= FIFO_LENGTH/2; // data is in 16 bits
            offset_addr += FIFO_LENGTH;
        }
        else
        {
            ESC_writeSPI(offset_addr, (uint16_t *) &pData[i], current_bytes);
            offset_addr += current_bytes;
            i+= current_bytes/2;
            current_bytes = 0;
        }
    }

    if(numbytes & 0x1)
    {
        //now send the last byte with extra alignment bytes
        // note that we read the adjacent byte and write it back
        ESC_readSPI(offset_addr, 2, &i);
        i &= 0xFF00;
        i |= (pData[((numbytes-1) >> 1)]) & 0xFF;   //pData is 16bit pointer
        ESC_writeSPI(offset_addr, &i, 2);
    }
}
/***********************************************************************************/
void ESC_writeBlockNonISR(uint16_t* pData, uint16_t offset_addr,uint16_t numbytes)
{

    DINT;
    ESC_writeBlockISR(pData, offset_addr,numbytes);
    EINT;
}


//-----------------------------------------------------------------------------------
// Function to initialize SPI port
//-----------------------------------------------------------------------------------
void ESC_initSPIFIFO(void)
{
    uint16_t m;
    EALLOW;

    // FIFO configuration
    SpixRegs->SPIFFCT.all=0x0;              // place SPI in reset
    for(m=0;m<3;m++);
    SpixRegs->SPIFFRX.all=0x2040;           // RX FIFO enabled, clear FIFO int
    SpixRegs->SPIFFRX.bit.RXFFIL = 16;  // Set RX FIFO level
    SpixRegs->SPIFFTX.all=0xE040;           // FIFOs enabled, TX FIFO released,

    // SPI configuration
    SpixRegs->SPIFFTX.bit.TXFFIL = 16;  // Set TX FIFO level
    SpixRegs->SPICCR.bit.SPICHAR = 0x7;//0xF; // Character Length  = 8
    SpixRegs->SPICCR.bit.CLKPOLARITY = 1; // Rising edge
    SpixRegs->SPICTL.bit.SPIINTENA = 1;     // Enabled
    SpixRegs->SPICTL.bit.TALK = 1;          //
    SpixRegs->SPICTL.bit.MASTER_SLAVE = 1;  // Master mode
    SpixRegs->SPICTL.bit.CLK_PHASE = 0;     // Add 1/2-cycle delay of Clk wrt SPISTEA
    SpixRegs->SPICTL.bit.OVERRUNINTENA = 1; // Overrun Interrupt enabled
    SpixRegs->SPISTS.all=0x0000;        // Clear Status bits (TxBufFull,INT, Overrun)

    //  SpixRegs->SPIBRR.all = 0x63;            // LSPCLK/100
    ClkCfgRegs.LOSPCP.all = 0x1; // 0 = sysclk/1 = 200M; 1 = sysclk/2 = 100M
    SpixRegs->SPIBRR.all=0x004;     // Baud Rate = LSPCLK / (SPIBRR+1) [LSPCLK=SysClk/4 by default=50M]
    SpixRegs->SPIFFCT.all=0x00;
    SpixRegs->SPIPRI.all=0x0020;            // Stop after transaction complete on EmuStop

    SpixRegs->SPIFFTX.bit.TXFFIENA = 0; // Disable TXFF INT
    SpixRegs->SPIFFRX.bit.RXFFIENA = 0; // disable RXFF INT


    SpixRegs->SPICCR.bit.SPISWRESET=1;  // Enable SPI

   EDIS;
}

//-----------------------------------------------------------------------------------
// Function to initialize GPIOs for SPIB port
//   GPIO64, GPIO65, GPIO66, GPIO63 
//  Not used in TMDSECATCNCD379D kit
//-----------------------------------------------------------------------------------
void ESC_initSPIBGpio(void)
{
    EALLOW;
        // Enable pull-ups on SPISIMO/SPISOMI/SPICLK/SPISTE pins
        GpioCtrlRegs.GPCPUD.all &= 0xFFFFFFF8;
        GpioCtrlRegs.GPBPUD.all &= 0xFFFFFFFE;

        // Enable SPISIMO/SPISOMI/SPICLK pins
        GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0x3;
        GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 0x3;
        GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 0x3;
        GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0x3;
        GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0x3;
        GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 0x3;
        GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0x3;
        GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0x3;

        // Enable SPISIMO/SPISOMI/SPICLK pins as async
        GpioCtrlRegs.GPCQSEL1.all |= 0xF;
        GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 0x3;

    EDIS;

}

//-----------------------------------------------------------------------------------
// Function to initialize GPIOs for SPIC port
//   GPIO122, GPIO123, GPIO124, GPIO125 
//   Function used in TMDSECATCNCD379D kit
//-----------------------------------------------------------------------------------
void ESC_initSPICGpio(void)
{
    EALLOW;
        // new control card with HD connector
        // Enable pull-ups on SPISIMO/SPISOMI/SPICLK/SPISTE pins
        //GPIO122,123,124,125
        GpioCtrlRegs.GPDPUD.all &= 0xC3FFFFFF;

        // Enable SPISIMO/SPISOMI/SPICLK pins
        GpioCtrlRegs.GPDGMUX2.bit.GPIO122 = 0x1;
        GpioCtrlRegs.GPDGMUX2.bit.GPIO123 = 0x1;
        GpioCtrlRegs.GPDGMUX2.bit.GPIO124 = 0x1;
        GpioCtrlRegs.GPDGMUX2.bit.GPIO125 = 0x1;

        GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0x2;
        GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0x2;
        GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0x2;
        GpioCtrlRegs.GPDMUX2.bit.GPIO125 = 0x2;

        // Enable SPISIMO/SPISOMI/SPICLK pins as async
        GpioCtrlRegs.GPDQSEL2.all |= 0x03F00000;
    EDIS;
}

//-----------------------------------------------------------------------------------
// Function to initialize GPIOs for SPIB port
//   GPIO16, GPIO17, GPIO18, GPIO19 
//  Not used in TMDSECATCNCD379D kit
//-----------------------------------------------------------------------------------
void ESC_initSPIAGpio(void)
{
    EALLOW;
        // Enable pull-ups on SPISIMO/SPISOMI/SPICLK/SPISTE pins
        GpioCtrlRegs.GPAPUD.all &= 0xFFF0FFFF;

        // Enable SPISIMO/SPISOMI/SPICLK pins
        GpioCtrlRegs.GPAMUX2.all |= 0x00000055;

        // Enable SPISIMO/SPISOMI/SPICLK pins as async
        GpioCtrlRegs.GPAQSEL2.all |= 0x0000003F;

    EDIS;
}

#else //#ifdef INTERFACE_SPI

//-----------------------------------------------------------------------------------
// internal function called from the ESC_WriteBlock* APIs
//-----------------------------------------------------------------------------------
void ESC_writeBlockEMIF2(uint16_t* pData, uint16_t offset_addr,uint16_t numbytes)
{
    uint16_t i = 0;
    uint16_t numwords = 0;

    offset_addr = offset_addr >> 1;

    if(numbytes & 0x01) // ODD no. of bytes
    {
        numwords = (numbytes - 1) >> 1;
        while(numwords > 0)
        {
            ((volatile uint16_t *)pEsc)[offset_addr] = pData[i];
            offset_addr++;
            numwords--;
            i++;
        }
        //now the last byte
        //read last words
        i = ((volatile uint16_t *)pEsc)[offset_addr];
        i &= 0xFF00;
        //update the word with the byte that needs to be written
        i |= pData[((numbytes-1) >> 1)] & 0xFF;
        //write back the word with the updated byte
        ((volatile uint16_t *)pEsc)[offset_addr] = i;

    }
    else
    {
        numwords = (numbytes) >> 1;
        while(numwords > 0)
        {
            ((volatile uint16_t *)pEsc)[offset_addr] = pData[i];
            offset_addr++;
            numwords--;
            i++;
        }
    }
}
/***********************************************************************************/
uint16_t ESC_readWordNonISR(uint16_t offset)
{
    uint16_t value;
    DINT;
    value = ((((volatile uint16_t *)pEsc)[((offset)>>1)]));
    EINT;
    return value;
}
/***********************************************************************************/
uint16_t ESC_readWordISR(uint16_t offset)
{
    return ((((volatile uint16_t *)pEsc)[((offset)>>1)]));
}
/***********************************************************************************/
void ESC_readBlockNonISR(uint16_t *pData,uint16_t Address,uint16_t Len)
{
    DINT;
    memcpy((uint16_t *)(pData), &((uint16_t *) pEsc)[((Address) >> 1)], ((Len+1)>>1));
    EINT;
}
/***********************************************************************************/
void ESC_readBlockISR(uint16_t *pData,uint16_t Address,uint16_t Len)
{
    memcpy((uint16_t *)(pData), &((uint16_t *) pEsc)[((Address) >> 1)], ((Len+1)>>1));
}
/***********************************************************************************/
uint32_t ESC_readDWordNonISR(uint16_t Address)
{
    uint32_t value;
    DINT;
    value = (uint32_t)(((volatile uint32_t *)pEsc)[(Address>>2)]);
    EINT;
    return value;
}
/***********************************************************************************/
uint32_t ESC_readDWordISR(uint16_t Address)
{
    return (uint32_t)(((volatile uint32_t *)pEsc)[(Address>>2)]);
}
/***********************************************************************************/
void ESC_writeBlockNonISR(uint16_t *pData,uint16_t Address,uint16_t Len)
{
    DINT;
    ESC_writeBlockEMIF2(pData,Address,Len);
    EINT;
}
/***********************************************************************************/
void ESC_writeBlockISR(uint16_t *pData,uint16_t Address,uint16_t Len)
{
    ESC_writeBlockEMIF2(pData,Address,Len);
}
/***********************************************************************************/
void ESC_writeDWordNonISR(uint32_t DWordValue, uint16_t Address)
{
    DINT;
    ((((volatile uint32_t *)pEsc)[(Address>>2)]) = (DWordValue));
    EINT;
}
/***********************************************************************************/
void ESC_writeDWordISR(uint32_t DWordValue, uint16_t Address)
{
    ((((volatile uint32_t *)pEsc)[(Address>>2)]) = (DWordValue));
}
/***********************************************************************************/
void ESC_writeWordNonISR(uint16_t WordValue, uint16_t Address)
{
    DINT;
    ((((volatile uint16_t *)pEsc)[((Address)>>1)]) = (WordValue));
    EINT;
}
/***********************************************************************************/
void ESC_writeWordISR(uint16_t WordValue, uint16_t Address)
{
    ((((volatile uint16_t *)pEsc)[((Address)>>1)]) = (WordValue));
}
#endif  //#ifdefINTERFACE_SPI

/***********************************************************************************/
void ESC_releaseET1100Reset(void)
{
      GPIO_SetupPinMux(ESC_RESET_ET1100_GPIO, GPIO_MUX_CPU1, 0);
      GPIO_WritePin(ESC_RESET_ET1100_GPIO, 1);  //release reset
}
/***********************************************************************************/
void ESC_holdET1100InReset(void)
{
      GPIO_SetupPinMux(ESC_RESET_ET1100_GPIO, GPIO_MUX_CPU1, 0);
      GPIO_WritePin(ESC_RESET_ET1100_GPIO, 0);  //hold in reset
}
/***********************************************************************************/
void ESC_configureLatch0GPIO(void)
{
    //This function configures SYNC0 GPIO as LATCH OUTPUT (Input to ESC)
    GPIO_SetupPinOptions(ESC_SYNC0_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(ESC_SYNC0_GPIO, GPIO_MUX_CPU1, 0);
}
/***********************************************************************************/
void ESC_configureLatch1GPIO(void)
{
    //This function configures SYNC1 GPIO as LATCH OUTPUT
    GPIO_SetupPinOptions(ESC_SYNC1_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(ESC_SYNC1_GPIO, GPIO_MUX_CPU1, 0);
}


//-----------------------------------------------------------------------------------
// ISR to handle PDI ISR
//-----------------------------------------------------------------------------------
interrupt void ESC_applicationLayerISR()
{
    //call the slave stack ISR routine
#ifdef ETHERCAT_STACK
    PDI_Isr();
#endif
    PieCtrlRegs.PIEACK.all |= 0x01;     // Issue PIE ack
}

//-----------------------------------------------------------------------------------
// ISR to handle SYNC0 ISR
//-----------------------------------------------------------------------------------
interrupt void ESC_applicationSync0ISR()
{
#ifdef ETHERCAT_STACK
    Sync0_Isr();
#endif
    //XINT5, PIE 12.INT3
    PieCtrlRegs.PIEACK.bit.ACK12 = 1;
}

//-----------------------------------------------------------------------------------
// ISR to handle SYNC1 ISR
//-----------------------------------------------------------------------------------
interrupt void ESC_applicationSync1ISR()
{
#ifdef ETHERCAT_STACK
    Sync1_Isr();
#endif
    //XINT4, PIE 12.INT2
    PieCtrlRegs.PIEACK.bit.ACK12 = 1;
}

//-----------------------------------------------------------------------------------
// Function to enable debug of SYNC0 signal on ControlCard configurations
//-----------------------------------------------------------------------------------
void ESC_enableSync0DebugOnCCARD(void)
{
    //for debug of SYNC0 line.- the below code connects GPIO2 to the SYNC0
    //internally because on the HW board we cannot put a scope on SYNC0 on HighRose connector

    //Connect SYNC0 (GPIO86) is INPUT1 for
    EALLOW;
    InputXbarRegs.INPUT1SELECT = ESC_SYNC0_GPIO; //input1 is tied to GPIO113

    OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX1 = 0x1; //INPUTXBAR1 to OUTPUTXBAR1
    OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX1 = 0x1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0x01; //GPIO2 to OUTPUTXBAR1
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0x01;
    EDIS;
}


//-----------------------------------------------------------------------------------
// Function to configure SYNC0 signal on ControlCard configurations
//-----------------------------------------------------------------------------------
void ESC_configureSync0GPIO(void)
{

    GPIO_SetupPinOptions(ESC_SYNC0_GPIO, GPIO_INPUT, GPIO_PULLUP| GPIO_ASYNC);
    GPIO_SetupPinMux(ESC_SYNC0_GPIO, GPIO_MUX_CPU1, 0);

    EALLOW;
    InputXbarRegs.INPUT14SELECT = ESC_SYNC0_GPIO; //input14 is tied to XINT5
    PieVectTable.XINT5_INT   = &ESC_applicationSync0ISR;

    XintRegs.XINT5CR.bit.POLARITY = 1;      // Falling edge interrupt
    XintRegs.XINT5CR.bit.ENABLE = 1;

    PieCtrlRegs.PIEIER12.bit.INTx3 = 1;     // Enable Group 1, INT5 (XINT2)
    IER |= 0x0800;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    ESC_enableSync0DebugOnCCARD();
}


//-----------------------------------------------------------------------------------
// Function to enable debug of SYNC0 signal on ControlCard configurations
//-----------------------------------------------------------------------------------
void ESC_enableSync1DebugOnCCARD(void)
{
    //for debug of SYNC1 line.- the below code connects GPIO3 to the SYNC1
    //internally because on the HW board we cannot put a scope on SYNC1 on HighRose connector

    //Connect SYNC1 is INPUT1 for
    EALLOW;
    InputXbarRegs.INPUT2SELECT = ESC_SYNC1_GPIO; //input2 is tied to SYNC1

    OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX3 = 0x1; //INPUTXBAR2 to OUTPUTXBAR2
    OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX3 = 0x1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0x01; //GPIO3 to OUTPUTXBAR2
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0x01;
    EDIS;
}

//-----------------------------------------------------------------------------------
// Function to configure SYNC1 signal on ControlCard configurations
//-----------------------------------------------------------------------------------
void ESC_configureSync1GPIO(void)
{

    GPIO_SetupPinOptions(ESC_SYNC1_GPIO, GPIO_INPUT, GPIO_PULLUP| GPIO_ASYNC);
    GPIO_SetupPinMux(ESC_SYNC1_GPIO, GPIO_MUX_CPU1, 0);

    EALLOW;
    InputXbarRegs.INPUT13SELECT = ESC_SYNC1_GPIO; //input13 is tied to XINT4
    PieVectTable.XINT4_INT   = &ESC_applicationSync1ISR;

    XintRegs.XINT4CR.bit.POLARITY = 1;      // Falling edge interrupt
    XintRegs.XINT4CR.bit.ENABLE = 1;

    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;     // Enable Group 12, INT2 (XINT4)
    IER |= 0x0800;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    ESC_enableSync1DebugOnCCARD();
}
/***********************************************************************************/
void ESC_resetET1100(void)
{

    GPIO_SetupPinMux(ESC_RESET_ET1100_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_WritePin(ESC_RESET_ET1100_GPIO, 0);    //hold reset low
    DELAY_US(500*1000);
    GPIO_WritePin(ESC_RESET_ET1100_GPIO, 1);    //release reset
    DELAY_US(500*1000);
}
/***********************************************************************************/
uint16_t  ESC_ET1100EEPROMLoadedCheck(void)
{
    uint16_t ii = 0;
    GPIO_SetupPinMux(ESC_EEPROM_LOADED_GPIO, GPIO_MUX_CPU1, 0);
    while(!GPIO_ReadPin(ESC_EEPROM_LOADED_GPIO))
    {
      DELAY_US(500*1000);
      ii++;
      if(ii > 10)
          break;
    }
    if(ii > 10)
        return 0;
    else
        return 1;
}

/***********************************************************************************/
void ESC_passFailSignalSetup(void)
{
    // Note:- This function is not called when EMIF1 is used because there
    // is conflict with GPIO31 and GPIO34 with EMIF1 signals and LEDs
    // so with LAUNCHXL2.0 , user will only know if there is an error
    // Both LEDs (GPIO31 and GPIO34 HIGH or LOW always means no error on
    // Launchpad XL 2.0)

    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);

    GPIO_SetupPinOptions(34, 1, GPIO_OPENDRAIN | GPIO_PULLUP);
    GPIO_SetupPinOptions(31, 1, GPIO_OPENDRAIN | GPIO_PULLUP);

    //GPIO34 and GPIO31 at HIGH - means NO ERROR
    //keep GPIO31 and GPIO34 LOW for PASS - by default
    GpioDataRegs.GPADAT.bit.GPIO31 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

}
/***********************************************************************************/
void ESC_signalFail(void)
{
    //Toggle GPIO34 and GPIO31 for fail
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    DELAY_US(10 * 1000);
}
/***********************************************************************************/
void ESC_signalPass(void)
{
    //keep GPIO31 and GPIO34 LOW for PASS
    GpioDataRegs.GPADAT.bit.GPIO31 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

    DELAY_US(500 * 1000);
}
/***********************************************************************************/
void ESC_initHW(void)
{

    //pEsc = MAKE_PTR_TO_ESC;

#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
    pEsc = (uint16_t *)0x100000;  //EMIF1
#else
    pEsc = (uint16_t *)0x2000;    //EMIF2
#endif


#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t)&RamfuncsLoadSize);
#endif


    InitSysCtrl();

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the compiler

#ifdef FLASH
    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)


    DINT;
    //  Initialize the PIE control registers to their default state.
    //  The default state is all PIE interrupts disabled and flags
    //  are cleared.
    //  This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    EALLOW;
    IER = 0x0000;
    IFR = 0x0000;
    EDIS;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // GService Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    //Configure to run EMIF1 on full Rate (EMIF1CLK = CPU1SYSCLK/2)
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x1;
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF2CLKDIV = 0x1;
    EDIS;

    EALLOW;
    InitGpio(); // Default setup for all pins
    InitCpuTimers();

//------------------------------------------------------------------------------
#ifdef INTERFACE_SPI
    //TxCnt=0;
    SPI_XmitInProgress=0;

#ifdef USE_SPIA
  SpixRegs = &SpiaRegs;
  ESC_initSPIAGpio();
#elif USE_SPIC
  SpixRegs = &SpicRegs;
  ESC_initSPICGpio();
#else
  SpixRegs = &SpibRegs;
  ESC_initSPIBGpio();
#endif
  ESC_initSPIFIFO();

  EALLOW;
//  SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission

  // Interrupts that are used in this example are re-mapped to
  // ISR functions found within this file.
  EALLOW;  // This is needed to write to EALLOW protected registers
  PieVectTable.XINT1_INT   = &ESC_applicationLayerISR;
  EDIS;   // This is needed to disable write to EALLOW protected registers

  // Configure External Interrupt from ET1100
  EALLOW;
  InputXbarRegs.INPUT4SELECT = ESC_SPI_INT_GPIO ;
  GPIO_SetupPinOptions(ESC_SPI_INT_GPIO, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);

  EALLOW;
  //for debug
#ifdef DEBUG
  //for debug - the same below code is used for SYNC0/SYNC1 monitoring
  //user has to either change the GPIO used or comment out the monitoring of SYNC0 or
  //SYNC1 before enabling below code

  //for debug of Interrupt line.- the below code connects GPIO2 to the GPIO78
  //internally because on the HW board we cannot put a scope on GPIO78 on CCARD,

//  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX7 = 0x1; //INPUTXBAR4 to OUTPUTXBAR1
//  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX7 = 0x1;
//
//  GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0x01;
//  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0x01;
#endif

  XintRegs.XINT1CR.bit.POLARITY = 0x0;      // Falling edge interrupt
  XintRegs.XINT1CR.bit.ENABLE = 1;
  //EDIS;

  // Enable interrupts required for this example
  PieCtrlRegs.PIECTRL.bit.ENPIE = 1;        // Enable the PIE block
  PieCtrlRegs.PIEIER1.bit.INTx4 = 1;        // Enable Group 1, INT4 (XINT1)

  IER=0x01;                            // Enable CPU INT1
  EINT;                                // Enable Global Interrupts

  ESC_passFailSignalSetup();
#else
    // EMIF INTERFACE (16b, async)

  // Configure External Interrupt from ET1100
  EALLOW;
  //keep LOSPCP same as SPI MODE of PDI
  ClkCfgRegs.LOSPCP.all = 0x1; // 0 = sysclk/1 = 200M; 1 = sysclk/2 = 100M

  PieVectTable.XINT1_INT = &ESC_applicationLayerISR;

  InputXbarRegs.INPUT4SELECT = ESC_EMIF_INT_GPIO ;
  GPIO_SetupPinOptions(ESC_EMIF_INT_GPIO, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);

#ifdef DEBUG //enabled for TIMING of ISR
  EALLOW;
  //for debug - the same below code is used for SYNC0/SYNC1 monitoring
  //user has to either change the GPIO used or comment out the monitoring of SYNC0 or
  //SYNC1 before enabling below code

//  //for debug of Interrupt line.- the below code connects GPIO2 to the GPIO78
    //internally because on the HW board we cannot put a scope on GPIO78 on CCARD,

//  OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX7 = 0x1; //INPUTXBAR4 to OUTPUTXBAR1
//  OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX7 = 0x1;
//
//  GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0x01;
//  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0x01;
#endif

  XintRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt
  XintRegs.XINT1CR.bit.ENABLE = 1;
  EDIS;

  PieCtrlRegs.PIECTRL.bit.ENPIE = 1;    // Enable the PIE block
  PieCtrlRegs.PIEIER1.bit.INTx4 =1;     // Enable Group 1, INT4 (XINT1)

  IER=0x01;                             // Enable CPU INT1
  EINT;                                 // Enable Global Interrupts

  // Grab EMIF1 For CPU1
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
  Emif1ConfigRegs.EMIF1MSEL.all = (uint32_t) 0x93A5CE71;

  // Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
  Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;

  // Commit the configuration related to protection. Till this bit remains set
  // content of EMIF1ACCPROT0 register can't be changed.
  Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;

  // Lock the configuration so that EMIF1COMMIT register can't be changed any more.
  Emif1ConfigRegs.EMIF1LOCK.all = 0x1;

  //Configure GPIO pins for EMIF1
  //setup_emif1_pinmux_async_16bit(0);
  setup_emif1_pinmux_async_16bit_option2(0);

  //Configure the access timing for CS2 space
  Emif1Regs.ASYNC_CS2_CR.all =  (EMIF_ASYNC_ASIZE_16    |
                                 EMIF_ASYNC_TA_1        |   // Wr to Rd turnaround
                                 EMIF_ASYNC_RHOLD_2     |   //
                                 EMIF_ASYNC_RSTROBE_56  |   //
                                 EMIF_ASYNC_RSETUP_4    |   //
                                 EMIF_ASYNC_WHOLD_2     |   //
                                 EMIF_ASYNC_WSTROBE_28  |   //
                                 EMIF_ASYNC_WSETUP_4    |
                                 EMIF_ASYNC_EW_ENABLE   |   // EMIF_ASYNC_EW_DISABLE
                                 EMIF_ASYNC_SS_DISABLE      // Strobe Select Mode Disable.
                                );
  Emif1Regs.ASYNC_WCCR.bit.WP0 = 1;             // Active-high Wait Polarity for EM1WAIT.
  Emif1Regs.ASYNC_WCCR.bit.MAX_EXT_WAIT = 0x1;
#else

  // Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
  Emif2ConfigRegs.EMIF2ACCPROT0.all = 0x0;

  // Commit the configuration related to protection. Till this bit remains set
  // content of EMIF1ACCPROT0 register can't be changed.
  Emif2ConfigRegs.EMIF2COMMIT.all = 0x1;

  // Lock the configuration so that EMIF1COMMIT register can't be changed any more.
  Emif2ConfigRegs.EMIF2LOCK.all = 0x1;

  //Configure GPIO pins for EMIF1
  setup_emif2_pinmux_async_16bit(0);

  //Configure the access timing for CS2 space
  Emif2Regs.ASYNC_CS2_CR.all =  (EMIF_ASYNC_ASIZE_16    |
                                 EMIF_ASYNC_TA_1        |   // Wr to Rd turnaround
                                 EMIF_ASYNC_RHOLD_2     |   //
                                 EMIF_ASYNC_RSTROBE_56  |   //
                                 EMIF_ASYNC_RSETUP_4    |   //
                                 EMIF_ASYNC_WHOLD_2     |   //
                                 EMIF_ASYNC_WSTROBE_28  |   //
                                 EMIF_ASYNC_WSETUP_4    |
                                 EMIF_ASYNC_EW_ENABLE   |   // EMIF_ASYNC_EW_DISABLE
                                 EMIF_ASYNC_SS_DISABLE      // Strobe Select Mode Disable.
                                );
  Emif2Regs.ASYNC_WCCR.bit.WP0 = 1;             // Active-high Wait Polarity for EM1WAIT.
  Emif2Regs.ASYNC_WCCR.bit.MAX_EXT_WAIT = 0x1;

  ESC_passFailSignalSetup();
#endif // USE_EMIF1
#endif //INTERFACE_SPI
  EALLOW;
  //CpuTimer0Regs.TCR.bit.TIE = 1;
  CpuTimer0Regs.TCR.bit.TSS = 0;    //start timer

  ESC_configureSync0GPIO();
  ESC_configureSync1GPIO();
  //  ESC_configureLatch0GPIO()
  //  ESC_configureLatch1GPIO();

  ESC_resetET1100();
  if(!ESC_ET1100EEPROMLoadedCheck())
  {
      //EEPROM load failed
      //signal fail
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
      ESC_passFailSignalSetup();
      //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
      //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
        while(1)
        {
            //fail
            ESC_signalFail();
            DELAY_US(500 * 1000);
        }
  }

}
/***********************************************************************************/
#ifdef PDI_HAL_TEST     //enable this to test the PDI HAL API
#define TEST_DATA_LENGTH    10
uint32_t wr_test_data[TEST_DATA_LENGTH];
uint32_t rd_test_data[TEST_DATA_LENGTH];
/***********************************************************************************/
void ESC_debugInitESCRegLogs(void)
{
    uint16_t i = 0;

    for(i=0; i < ESC_DEBUG_REGS_LENGTH ; i++)
    {
        escRegs[i].address = escRegs[i].data = 0xFFFF;
    }
}
/***********************************************************************************/
void ESC_debugAddESCRegsAddress(uint16_t address)
{
    uint16_t i = 0;
    for(i=0; i < ESC_DEBUG_REGS_LENGTH ; i++)
    {
        if( escRegs[i].address == 0xFFFF)
        {
            //memcpy(&escRegs[i].name[0], name, namelen);
            escRegs[i].address = address;
            return;
        }
    }
}
/***********************************************************************************/
void ESC_debugUpdateESCRegLogs(void)
{
    uint16_t i = 0;
    while(escRegs[i].address != 0xFFFF)
    {
        escRegs[i].data = ESC_readWordNonISR(escRegs[i].address);
        i++;
    }
}
/***********************************************************************************/
void clear_test_data(void)
{
    uint16_t ii=0;
    for(ii =0; ii < TEST_DATA_LENGTH; ii++)
    {
        wr_test_data[ii] = rd_test_data[ii] = 0UL;
    }
}
/***********************************************************************************/
void init_wr_test_data(void)
{
    uint16_t ii=0;
    for(ii =0; ii < TEST_DATA_LENGTH; ii++)
    {
        wr_test_data[ii] = 0xBABAABBAUL;
    }
}
/***********************************************************************************/
//local test function pass fail defines
#define BLOCKTEST_PASS  1
#define BLOCKTEST_FAIL  0
uint16_t block_test_data(uint16_t numbytes)
{
    uint16_t ii = 0, jj = 0;
    uint16_t *wrdata = (uint16_t *)(&wr_test_data[0]);
    uint16_t *rddata = (uint16_t *)(&rd_test_data[0]);
    if(numbytes & 0x1) //odd
    {
        ii = (numbytes -1)/2;
    }
    else
    {
        ii = (numbytes)/2;
    }
    while(ii)
    {
        if((*wrdata++) != (*rddata++))
            goto fail_block_rw;
        jj++;
        ii--;
    }
    if(numbytes & 0x1) //odd
    {
        //for last byte
        if(((*wrdata) & 0x00FF) != ((*rddata) & 0x00FF))
        {
            goto fail_block_rw;
        }
        else
        {
            return BLOCKTEST_PASS;
        }
    }
    else
    {
        return BLOCKTEST_PASS;
    }

fail_block_rw:
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
      ESC_passFailSignalSetup();
      //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
      //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
    while(1)
    {
        //fail
        ESC_signalFail();
    }
}
/***********************************************************************************/
void ESC_setupPDITestInterface(void)
{
    uint32_t test_address = 0x0000UL;
    uint32_t dtest_data = 0xFFFFFFFFUL;
    uint16_t pdi_control = 0x0;
    uint16_t ii = 0;

     ESC_debugInitESCRegLogs();
     ESC_debugAddESCRegsAddress(0x0);     /* "type" */
     ESC_debugAddESCRegsAddress(0x2);     /* "build" */
     ESC_debugAddESCRegsAddress(0x4);     /* "fmmu" */
     ESC_debugAddESCRegsAddress(0x8);     /* "rams" */
     ESC_debugAddESCRegsAddress(0x100);   /* "dlc1" */
     ESC_debugAddESCRegsAddress(0x102);   /* "dlc2" */
     ESC_debugAddESCRegsAddress(0x110);   /* "dls"  */
     ESC_debugAddESCRegsAddress(0x310);   /* "llc1" */
     ESC_debugAddESCRegsAddress(0x312);   /* "llc2" */
     ESC_debugAddESCRegsAddress(0xE00);   /* "porv" */
     ESC_debugAddESCRegsAddress(0x510);   /* "miis" */
     ESC_debugAddESCRegsAddress(0x512);   /* "phya" */
     ESC_debugAddESCRegsAddress(0x514);   /* "phyd" */
     ESC_debugAddESCRegsAddress(0x516);   /* "miia" */
     ESC_debugAddESCRegsAddress(0x518);   /* "port0" */
     ESC_debugAddESCRegsAddress(0x51A);   /* "port1" */
     ESC_debugAddESCRegsAddress(0x1000);  /* "RAM address"*/

     init_wr_test_data();

     //Now wait till EEPROM is loaded
    do
    {
        pdi_control = ESC_readWordNonISR(0x0140); /*ESC_PDI_CONTROL_OFFSET*/
        if( ii++ == 0x100)
            break;
#ifdef INTERFACE_SPI
    } while  (((pdi_control & 0xFF) != 0x05));  //SPI PDI
#else
    } while  (((pdi_control & 0xFF) != 0x08));  //EMIF PDI
#endif

    if( ii >= 0x100)
    {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
      ESC_passFailSignalSetup();
      //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
      //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
        while(1)
        {
            //fail
            ESC_signalFail();
            DELAY_US(500 * 1000);
        }
    }

    // NOW EEPROM is loaded and PDI is either EMIF (16 bit) or SPI.
    // ET1100 RAM read/write tests; On ET1100 RAM being at 0x1000 and can go up to 0xFFFF

#ifdef INTERFACE_SPI
     // with SPI we can address 0x1000 to 0xFFFF address range of ET1100
     for(test_address = ESC_PDI_RAM_START_ADDRESS_OFFSET; test_address <= ESC_PDI_RAM_END_ADDRESS_OFFSET; test_address+=4)

#else
     // the EMIF2 address range is between 0x2000 to 0x2FFF words.
     // This means we can address 0x1000 - 0x2000 of the RAM address range of ET1100
     // which is equivalent to 0x2800 to 0x2FFF of the C28x EMIF2 address space.
     //please refer to ET1100 datasheet for more details
    for(test_address = ESC_PDI_RAM_START_ADDRESS_OFFSET; test_address <= ESC_PDI_RAM_END_ADDRESS_OFFSET; test_address+=4)
#endif
    {

        ///////////////////////////////////////////////////
        //////READ DWORD and WRITE DWORD NON ISR API Test
        ///////////////////////////////////////////////////
        dtest_data = ESC_readDWordNonISR(test_address);
        ESC_writeDWordNonISR(0xBAADF00D, test_address);
        dtest_data = ESC_readDWordNonISR(test_address);;
        if(dtest_data != (uint32_t)0xBAADF00D)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }

         }
        ESC_writeDWordNonISR(0x00000000, test_address);
        dtest_data = ESC_readDWordNonISR(test_address);
        if(dtest_data != (uint32_t)0x00000000)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }
         }

        ///////////////////////////////////////////////////
        //////READ WORD and WRITE WORD Non-ISR API Test
        ///////////////////////////////////////////////////
        dtest_data = ESC_readWordNonISR(test_address);
        ESC_writeWordNonISR(0xF00D, test_address);
        dtest_data = ESC_readWordNonISR(test_address);;
        if((uint16_t)dtest_data != (uint16_t)0xF00D)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }

         }
        ESC_writeWordNonISR(0x0000, test_address);
        dtest_data = ESC_readWordNonISR(test_address);
        if((uint16_t)dtest_data != (uint16_t)0x0000)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }
         }

        ///////////////////////////////////////////////////
        //////READ WORD and WRITE WORD ISR API Test
        ///////////////////////////////////////////////////

        dtest_data = ESC_readWordISR(test_address+2);
        ESC_writeWordISR(0xBAAD, test_address+2);
        dtest_data = ESC_readWordISR(test_address+2);;
        if((uint16_t)dtest_data != (uint16_t)0xBAAD)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }

         }
        ESC_writeWordISR(0x0000, test_address+2);
        dtest_data = ESC_readWordISR(test_address+2);
        if((uint16_t)dtest_data != (uint16_t)0x0000)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }
         }


        ///////////////////////////////////////////////////
        //////READ DWORD and WRITE DWORD ISR API Test
        ///////////////////////////////////////////////////
        dtest_data = ESC_readDWordISR(test_address);
        ESC_writeDWordISR(0xBAADF00D, test_address);
        dtest_data = ESC_readDWordISR(test_address);;
        if(dtest_data != (uint32_t)0xBAADF00D)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }

         }
        ESC_writeDWordNonISR(0x00000000, test_address);
        dtest_data = ESC_readDWordNonISR(test_address);
        if(dtest_data != (uint32_t)0x00000000)
         {
#ifdef USE_EMIF1    //LAUNCHXL Rev2.0 J9 connector option for accessing ET1100
            ESC_passFailSignalSetup();
            //GPIO31 and 34 are EMIF1 signals so re-use them as GPIO
            //claim it back from EMIF since there is a problem    ESC_passFailSignalSetup();
#endif
            while(1)
            {
                //fail
                ESC_signalFail();
            }
         }

    }

     ///////////////////////////////////////////////////
     //////READ BLOCK and WRITE BLOCK Non-ISR API Test
     ///////////////////////////////////////////////////

     ESC_writeBlockNonISR((uint16_t *)(&wr_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4));//even
     ESC_readBlockNonISR((uint16_t *)(&rd_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4));

     block_test_data(TEST_DATA_LENGTH*4);

     clear_test_data();

     ESC_writeBlockNonISR((uint16_t *)(&wr_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4)-1);//odd
     ESC_readBlockNonISR((uint16_t *)(&rd_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4)-1);

     block_test_data((TEST_DATA_LENGTH*4)-1);

     ///////////////////////////////////////////////////
     //////READ BLOCK and WRITE BLOCK ISR API Test
     ///////////////////////////////////////////////////

     init_wr_test_data();

     ESC_writeBlockISR((uint16_t *)(&wr_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4));//odd
     ESC_readBlockISR((uint16_t *)(&rd_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4));

     block_test_data(TEST_DATA_LENGTH*4);

     clear_test_data();

     ESC_writeBlockISR((uint16_t *)(&wr_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4)-1);//odd
     ESC_readBlockISR((uint16_t *)(&rd_test_data[0]),  ESC_PDI_RAM_START_ADDRESS_OFFSET, (TEST_DATA_LENGTH*4)-1);

     block_test_data((TEST_DATA_LENGTH*4)-1);

}
#endif //PDI_HAL_TEST



