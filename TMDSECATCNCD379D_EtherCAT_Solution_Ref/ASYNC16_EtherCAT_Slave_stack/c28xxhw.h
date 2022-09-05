/**
 * \addtogroup C28XX_HW Parallel ESC Access
 * @{
 */

/**
\file C28xxhw.h
\author TI
\brief Defines and Macros to access the ESC via a parallel interface

 */


#ifndef _C28XXHW_H_
#define _C28XXHW_H_


/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include <string.h>
//#include <malloc.h> //commented for C28x
#include "esc.h"


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

#define ESC_MEM_ADDR    UINT16 /**< \brief ESC address type (16Bit)*/
#define ESC_MEM_SHIFT   1 /**< \brief shift to convert Byte address to ESC address type*/

#ifndef HW_GetTimer
/**
 * \todo Define a macro or implement a function to get the hardware timer value.<br>See SSC Application Note for further details, www.beckhoff.com/english.asp?download/ethercat_development_products.htm?id=71003127100387
 */
#define HW_GetTimer()				ESC_getTimer()
#endif

#ifndef HW_ClearTimer
/**
 * \todo Define a macro or implement a function to clear the hardware timer value.<br>See SSC Application Note for further details, www.beckhoff.com/english.asp?download/ethercat_development_products.htm?id=71003127100387
 */
#define HW_ClearTimer()				ESC_clearTimer()
#endif

/**
 * \todo Define the hardware timer ticks per millisecond.
 */
#define ECAT_TIMER_INC_P_MS			ESC_timerIncPerMilliSec()


//#warning "define access to timer register(counter)"

#define     HW_GetALEventRegister()             		ESC_readWordNonISR(ESC_AL_EVENT_OFFSET)
#define     HW_GetALEventRegister_Isr()           		ESC_readWordISR(ESC_AL_EVENT_OFFSET)
#define     HW_EscRead(pData,Address,Len)       		ESC_readBlockNonISR(pData,Address,Len)
#define     HW_EscReadIsr(pData,Address,Len)       		ESC_readBlockISR(pData,Address,Len)
#define     HW_EscReadDWord(DWordValue, Address)    	((DWordValue) = ESC_readDWordNonISR(Address));
#define     HW_EscReadDWordIsr(DWordValue, Address) 	((DWordValue) = ESC_readDWordISR(Address));
#define     HW_EscReadWord(WordValue, Address)  		((WordValue) = ESC_readWordNonISR(Address))
#define     HW_EscReadWordIsr(WordValue, Address) 		((WordValue) = ESC_readWordISR(Address)) /**< \brief 16Bit specific ESC (register and DPRAM) read access.<br>Called from ISRs.*/
#define     HW_EscReadMbxMem(pData,Address,Len) 		ESC_readBlockNonISR(pData,Address,Len)

#define     HW_EscWriteDWord(DWordValue, Address)   	ESC_writeDWordNonISR(DWordValue,Address)
#define     HW_EscWriteDWordIsr(DWordValue, Address) 	ESC_writeDWordISR(DWordValue,Address)
#define     HW_EscWriteWord(WordValue, Address)  		ESC_writeWordNonISR(WordValue,Address)
#define     HW_EscWriteWordIsr(WordValue, Address) 		ESC_writeWordISR(WordValue, Address)
//#define     HW_EscWriteMbxMem(pData,Address,Len)    	ESC_writeBlockNonISR(pData,Address,Len)
//#define     HW_EscWrite(pData,Address,Len)      		ESC_writeBlockNonISR(pData,Address,Len)
#define     HW_EscWriteMbxMem(pData,Address,Len)    	ESC_writeBlockNonISR(pData,Address,Len)
#define     HW_EscWrite(pData,Address,Len)      		ESC_writeBlockNonISR(pData,Address,Len)
#define     HW_EscWriteIsr(pData,Address,Len)           ESC_writeBlockISR(pData,Address,Len)



#ifndef TIMER_INT_HEADER
#define    TIMER_INT_HEADER /**< \brief Compiler directive before the hardware timer ISR function*/
#endif

#define     ESC_RD                    0x02 /**< \brief Indicates a read access to ESC or EEPROM*/
#define     ESC_WR                    0x04 /**< \brief Indicates a write access to ESC or EEPROM.*/

#define ENABLE_ESC_INT()	 __asm(" clrc INTM")
#define DISABLE_ESC_INT()	 __asm(" setc INTM")

#endif//_C28XXHW_H_


/*-----------------------------------------------------------------------------------------
------
------    Global variables
------
-----------------------------------------------------------------------------------------*/
//
//#if _C28XXHW_
//    #define PROTO
//#else
//    #define PROTO extern
//#endif

extern MEM_ADDR ESCMEM *            pEsc;            /**< \brief Pointer to the ESC.<br>Shall be initialized in HW_Init().*/


/*-----------------------------------------------------------------------------------------
------
------    Global functions
------
-----------------------------------------------------------------------------------------*/

extern UINT16 HW_Init(void);
extern void HW_Release(void);
extern void ESC_initHW(void);

/* extern HAL level functions*/
extern UINT16*            pEsc;

extern UINT32 ESC_getTimer();
extern void ESC_clearTimer();
extern UINT32 ESC_timerIncPerMilliSec();

/* etherCAT Slave stack HAL level inerface functions*/

extern UINT16 ESC_readWordNonISR(UINT16 offset);
extern UINT16 ESC_readWordISR(UINT16 offset);
extern void ESC_readBlockNonISR(UINT16 *pData,UINT16 Address,UINT16 Len);
extern void ESC_readBlockISR(UINT16 *pData,UINT16 Address,UINT16 Len);
extern UINT32 ESC_readDWordNonISR(UINT16 Address);
extern UINT32 ESC_readDWordISR(UINT16 Address);
extern void ESC_writeBlockNonISR(UINT16 *pData,UINT16 Address,UINT16 Len);
extern void ESC_writeDWordNonISR(UINT32 DWordValue, UINT16 Address);
extern void ESC_writeWordNonISR(UINT16 WordValue, UINT16 Address);
extern void ESC_writeBlockISR(UINT16 *pData,UINT16 Address,UINT16 Len);
extern void ESC_writeDWordISR(UINT32 DWordValue, UINT16 Address);
extern void ESC_writeWordISR(UINT16 WordValue, UINT16 Address);

extern void APPL_Application(void);

extern void   APPL_AckErrorInd(UINT16 stateTrans);
extern UINT16 APPL_StartMailboxHandler(void);
extern UINT16 APPL_StopMailboxHandler(void);
extern UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
extern UINT16 APPL_StopInputHandler(void);
extern UINT16 APPL_StartOutputHandler(void);
extern UINT16 APPL_StopOutputHandler(void);

extern UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize);
extern void APPL_InputMapping(UINT16* pData);
extern void APPL_OutputMapping(UINT16* pData);
/** @}*/
