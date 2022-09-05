/**
\addtogroup TMDSECATCNCD379D-EchoBack TMDSECATCNCD379D-EchoBack
@{
*/

/**
\file TMDSECATCNCD379D-EchoBack.c
\brief Implementation

\version 1.0.0.11
*/


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "ecat_def.h"

#include "applInterface.h"

#define _TMDSECATCNCD379_D_ECHO_BACK_ 1
#include "TMDSECATCNCD379D-EchoBack.h"
#undef _TMDSECATCNCD379_D_ECHO_BACK_
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////
/**

\brief This function resets the outputs
*////////////////////////////////////////////////////////////////////////////////////////

void PDO_ResetOutputs(void)
{
	LEDS0x7000.LED1 = 0x0;
	LEDS0x7000.LED2 = 0x0;
	LEDS0x7000.LED3 = 0x0;
	LEDS0x7000.LED4 = 0x0;
	LEDS0x7000.LED5 = 0x0;
	LEDS0x7000.LED6 = 0x0;
	LEDS0x7000.LED7 = 0x0;
	LEDS0x7000.LED8 = 0x0;

	DatafromMaster0x7010.DatafromMaster = 0x0;
	TargetMode0x7012.Mode = 0x0;
	TargetSpeedPosReq0x7014.SpeedPosReq = 0x0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
           all general settings were checked to start the input handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete 
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

#if COE_SUPPORTED
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
   
    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
//              pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>4 /*3 - TI C28x Port*/));    //goto PDO entry
    			//need for change  - C28x is 16 bit machine, so each entry is atleast 16 bits (so divide the no. of bits by 16, to get the proper
                //Pointer increment.

                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
//                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>4 /*3 - TI C28x Port*/));    //goto PDO entry
					//need for change  - C28x is 16 bit machine, so each entry is atleast 16 bits (so divide the no. of bits by 16, to get the proper
					//Pointer increment.

                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

#else
#if _WIN32
   #pragma message ("Warning: Define 'InputSize' and 'OutputSize'.")
#else
    #warning "Define 'InputSize' and 'OutputSize'."
#endif
#endif

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data

\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
    uint16_t j = 0;
    uint16_t *pTmpData = (uint16_t *)pData;
    uint16_t data;


   for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
   {
      switch (sTxPDOassign.aEntries[j])
      {
      /* TxPDO  */
      case 0x1A00: // 8 bits
    	  data = ((Switches0x6000.Switch8 << 7) | (Switches0x6000.Switch7 << 6) |
		  (Switches0x6000.Switch6 << 5) | (Switches0x6000.Switch5 << 4) | (Switches0x6000.Switch4 << 3) |
		  (Switches0x6000.Switch3 << 2) | (Switches0x6000.Switch2 << 1) | Switches0x6000.Switch1);

    	  * (volatile uint16_t *)pTmpData = data;

         break;
      /* TxPDO 1*/
	  case 0x1A01:
	  // 32 bits
            * (volatile uint16_t *)pTmpData |= ((DataToMaster0x6010.DataToMaster) & 0xFF) << 8;
            pTmpData++;
            * (volatile uint16_t *)pTmpData = ((DataToMaster0x6010.DataToMaster) & 0x00FFFF00) >> 8;
            pTmpData++;
            * (volatile uint16_t *)pTmpData = ((DataToMaster0x6010.DataToMaster) & 0xFF000000) >> 24;

	  //16bits
			* (volatile uint16_t *)pTmpData |= (( TargetModeResponse0x6012.ModeResponse) & 0x00FF) << 8; //8 bits
			pTmpData++;
			* (volatile uint16_t *)pTmpData = (( TargetModeResponse0x6012.ModeResponse) & 0xFF00) >> 8; //8 bits
	  //32bits
			* (volatile uint16_t *)pTmpData |= ((TargetSpeedPosFeedback0x6014.SpeedPosFbk) & 0xFF) << 8;
			pTmpData++;
			* (volatile uint16_t *)pTmpData = ((TargetSpeedPosFeedback0x6014.SpeedPosFbk) & 0x00FFFF00) >> 8;
			pTmpData++;
			* (volatile uint16_t *)pTmpData = ((TargetSpeedPosFeedback0x6014.SpeedPosFbk) & 0xFF000000) >> 24;
			break;

      }
   }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
    uint16_t j = 0;
    uint16_t *pTmpData = (uint16_t *)pData;// allow byte processing
    uint16_t data = 0;
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch (sRxPDOassign.aEntries[j])
        {
        /* RxPDO */
        case 0x1600: //byte (8 bits)
        	data = (*(volatile uint16_t *)pTmpData);
        	(LEDS0x7000.LED1) = data & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED2) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED3) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED4) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED5) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED6) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED7) = data  & 0x1;
        	data = data >> 1;
        	(LEDS0x7000.LED8) = data  & 0x1;
        	data = data >> 1;
        	break;
		/* RxPDO 1*/
		case 0x1601:
		//32 bits
			(DatafromMaster0x7010.DatafromMaster) = data & 0xFF;
			pTmpData++;

			DatafromMaster0x7010.DatafromMaster |= (uint32_t) ((uint32_t)((*(volatile uint16_t *)pTmpData) & 0xFFFF) << 8);
			pTmpData++; //increment the pointer now that we have read 2 bytes
			DatafromMaster0x7010.DatafromMaster |= (uint32_t) ((uint32_t)((*(volatile uint16_t *)pTmpData) & 0xFF) << 24);

		// 16 bits

			TargetMode0x7012.Mode =  (uint16_t) ((uint16_t)((*(volatile uint16_t *)pTmpData) & 0xFF00) >> 8);
            pTmpData++; //increment the pointer now that we have read 2 bytes
            TargetMode0x7012.Mode |= (uint16_t) ((uint16_t)((*(volatile uint16_t *)pTmpData) & 0x00FF) << 8);

        // 32 bits

            TargetSpeedPosReq0x7014.SpeedPosReq = (uint32_t) ((uint32_t)((*(volatile uint16_t *)pTmpData) & 0xFF00) >> 8);
            pTmpData++;

            TargetSpeedPosReq0x7014.SpeedPosReq |= (uint32_t) ((uint32_t)((*(volatile uint16_t *)pTmpData) & 0xFFFF) << 8);
            pTmpData++; //increment the pointer now that we have read 2 bytes
            TargetSpeedPosReq0x7014.SpeedPosReq |= (uint32_t) ((uint32_t)((*(volatile uint16_t *)pTmpData) & 0xFF) << 24);
			break;

        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR 
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{

// ###NOTE#####
// customers can toggle IOs basaed on the LED state requested from Master here.

//lets just loopback the  inputs and outputs with EtherCAT Master
	Switches0x6000.Switch1 	= LEDS0x7000.LED1;
	Switches0x6000.Switch2 	= LEDS0x7000.LED2;
	Switches0x6000.Switch3 	= LEDS0x7000.LED3;
	Switches0x6000.Switch4 	= LEDS0x7000.LED4;

	Switches0x6000.Switch5 	= LEDS0x7000.LED5;
	Switches0x6000.Switch6 	= LEDS0x7000.LED6;
	Switches0x6000.Switch7 	= LEDS0x7000.LED7;
	Switches0x6000.Switch8 	= LEDS0x7000.LED8;

	DataToMaster0x6010.DataToMaster = DatafromMaster0x7010.DatafromMaster;

	TargetModeResponse0x6012.ModeResponse = TargetMode0x7012.Mode;
	TargetSpeedPosFeedback0x6014.SpeedPosFbk = TargetSpeedPosReq0x7014.SpeedPosReq;

}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
#if _WIN32
   #pragma message ("Warning: Implement explicit Device ID latching")
#else
    #warning "Implement explicit Device ID latching"
#endif
    /* Explicit Device 5 is expected by Explicit Device ID conformance tests*/
    return 0x5;
}
#endif



#if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
extern void ESC_initHW(void);
void main(void)
{
    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();

#ifdef DEBUG
    //setup PDI for test
//  ESC_setupPDITestInterface();
#endif

    MainInit();

    bRunApplication = TRUE;
    do
    {
        MainLoop();
#ifdef DEBUG
		//Keep updating local RAM with ET1100 registers for debug
//		ESC_debugUpdateESCRegLogs();
#endif

    } while (bRunApplication == TRUE);

    HW_Release();
}
#endif //#if USE_DEFAULT_MAIN
/** @} */


