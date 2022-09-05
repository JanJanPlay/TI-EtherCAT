//###########################################################################
//
// FILE:   pdi_test_appl.c
//
// TITLE:  EtherCAT(ET1100) PDI (Processor Data Interface) example for F2837x
//         devices
//
//! \addtogroup C2k_EtherCAT_adapater_examples_list
//! <h1> PDI Interface test Example </h1>
//!
//! This example shows how to use EMIF or SPI interface (depending on project
//! build settings)
//! to talk to ET1100 EtherCAT chip on the TI C2000 EtherCAT adapater board.
//!
//! Users will have to build the project as per the PDI interface settings
//! they have programmed on the EEPROM.
//!
//! \b External \b Connections \n
//!  Users will be running this example on a F2837x control card or launch pad
//!  with TI C2k EtherCAT adapater board connected to the control card or
//!  launch pad. The adapater board has a switch to choose EMIF or SPI
//!  interface and in addition to this being correctly configured. users will
//!  have to have proper EEPROM contents loaded for the PDI interface to be up
//!
//! \b Watch \b Variables \n
//!  esc_regs (ESC stands for EtherCAT Slave Controller) data structre will be
//!  constantly updated i nthis example, users can add or remove the registers
//!  that they want to monitor or not. Please refer to Beckhoff ET1100
//!  registers documentation for more information on the etherCAT registers.
//!
//!  The init function in the HAL also clears up all the DPRAM in ET1100 using
//!  the PDI interface, users can verify this by connecting a TWINCAT master to
//!  the C2000 EtherCAT slave device
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

#include "ethercat_slave_c28x_hal.h"

//
// Main
//
void main()
{
    //Intialize C28x MCU and HAL interface
    ESC_initHW();

    //setup PDI for test
    ESC_setupPDITestInterface();

    while(1)
    {
        //Keep updating local RAM with ET1100 registers for debug
        ESC_debugUpdateESCRegLogs();
        DELAY_US(1000 * 500);
    }
}

//dummyISR as there is no real etherCAT slave stack in this applications
void PDI_Isr(void)
{
    return;
}

//
// End of File
//
