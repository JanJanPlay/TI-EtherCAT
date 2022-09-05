/**
\addtogroup C28XX_HW Parallel ESC Access
@{
*/

/**
\file c28xxhw.c
\author TI
\brief Implementation
This file contains the interface to the ESC via MCI
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#define    _C28XXHW_ 1
#include "c28xxhw.h"
#undef _C28XXHW_
#define    _C28XXHW_ 0

#include "ecatslv.h"
#include "ecatappl.h"

/*--------------------------------------------------------------------------------------
------
------    local Types and Defines
------
--------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/
TSYNCMAN    TmpSyncMan;

/*-----------------------------------------------------------------------------------------
------
------    local functions
------
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------
------    functions
------
-----------------------------------------------------------------------------------------*/



/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0 if initialization was successful

 \brief    This function initialize the EtherCAT Slave Interface.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 HW_Init(void)
{
    
    UINT32 intMask = 0;
    /* the memory interface to the ESC, the ESC-interrupt and the ECAT-timer for the
       watchdog monitoring should be initialized here microcontroller specific*/

    //Intialize C28x MCU and HAL interface, the DPRAM pointer is initialized inside this function
    ESC_initHW();

    /* we have to wait here, until the ESC is started */
    {
    UINT16 u16PdiCtrl = 0;

    do
    {
        HW_EscReadWord(u16PdiCtrl,ESC_PDI_CONTROL_OFFSET);
        u16PdiCtrl = SWAPWORD(u16PdiCtrl);

    } while  (((u16PdiCtrl & 0xFF) != 0x08) && ((u16PdiCtrl & 0xFF) != 0x05)); //(((u16PdiCtrl & 0xFF) < 0x8) || ((u16PdiCtrl & 0xFF) > 0xD) );
    }


    /* initialize the PDI - interrupt source*/
    //INIT_ESC_INT;

    /* initialize the AL_Event Mask Register */
    /* the AL Event-Mask register is initialized with 0, so that no ESC interrupt is generated yet,
       the AL Event-Mask register will be adapted in the function StartInputHandler in ecatslv.c
        when the state transition from PREOP to SAFEOP is made */
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

    /* enable the ESC-interrupt microcontroller specific,
        the macro ENABLE_ESC_INT should be defined in ecat_def.h */
    ENABLE_ESC_INT();

    return 0;
}

void HW_Release(void)
{
}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the interrupts from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////

#ifndef _WIN32
//interrupt - TI C28x port ,commented out ; because we call this in a void interrupt function; don't want and IRET for an LCR
#endif
void HW_EcatIsr(void)
{
    PDI_Isr();
}



/** @} */
