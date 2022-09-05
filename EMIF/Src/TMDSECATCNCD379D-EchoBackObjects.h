/**
* \addtogroup TMDSECATCNCD379D-EchoBack TMDSECATCNCD379D-EchoBack
* @{
*/

/**
\file TMDSECATCNCD379D-EchoBackObjects
\author ET9300Utilities.ApplicationHandler (Version 1.3.6.0) | EthercatSSC@beckhoff.com

\brief TMDSECATCNCD379D-EchoBack specific objects<br>
\brief NOTE : This file will be overwritten if a new object dictionary is generated!<br>
*/

#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
#define PROTO
#else
#define PROTO extern
#endif
/******************************************************************************
*                    Object 0x1600 : LEDS process data mapping
******************************************************************************/
/**
* \addtogroup 0x1600 0x1600 | LEDS process data mapping
* @{
* \brief Object 0x1600 (LEDS process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x7000.1<br>
* SubIndex 2 - Reference to 0x7000.2<br>
* SubIndex 3 - Reference to 0x7000.3<br>
* SubIndex 4 - Reference to 0x7000.4<br>
* SubIndex 5 - Reference to 0x7000.5<br>
* SubIndex 6 - Reference to 0x7000.6<br>
* SubIndex 7 - Reference to 0x7000.7<br>
* SubIndex 8 - Reference to 0x7000.8<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1600[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x7000.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x7000.2 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex3 - Reference to 0x7000.3 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex4 - Reference to 0x7000.4 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex5 - Reference to 0x7000.5 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex6 - Reference to 0x7000.6 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex7 - Reference to 0x7000.7 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex8 - Reference to 0x7000.8 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1600[] = "LEDS process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000"
"SubIndex 004\000"
"SubIndex 005\000"
"SubIndex 006\000"
"SubIndex 007\000"
"SubIndex 008\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x7000.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x7000.2 */
UINT32 SI3; /* Subindex3 - Reference to 0x7000.3 */
UINT32 SI4; /* Subindex4 - Reference to 0x7000.4 */
UINT32 SI5; /* Subindex5 - Reference to 0x7000.5 */
UINT32 SI6; /* Subindex6 - Reference to 0x7000.6 */
UINT32 SI7; /* Subindex7 - Reference to 0x7000.7 */
UINT32 SI8; /* Subindex8 - Reference to 0x7000.8 */
} OBJ_STRUCT_PACKED_END
TOBJ1600;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1600 LEDSProcessDataMapping0x1600
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={8,0x70000101,0x70000201,0x70000301,0x70000401,0x70000501,0x70000601,0x70000701,0x70000801}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1601 : Output mapping 1
******************************************************************************/
/**
* \addtogroup 0x1601 0x1601 | Output mapping 1
* @{
* \brief Object 0x1601 (Output mapping 1) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x7010.1<br>
* SubIndex 2 - Reference to 0x7012.1<br>
* SubIndex 3 - Reference to 0x7014.1<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1601[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x7010.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x7012.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex3 - Reference to 0x7014.1 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1601[] = "Output mapping 1\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x7010.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x7012.1 */
UINT32 SI3; /* Subindex3 - Reference to 0x7014.1 */
} OBJ_STRUCT_PACKED_END
TOBJ1601;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1601 OutputMapping10x1601
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={3,0x70100120,0x70120110,0x70140120}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1A00 : switches process data mapping
******************************************************************************/
/**
* \addtogroup 0x1A00 0x1A00 | switches process data mapping
* @{
* \brief Object 0x1A00 (switches process data mapping) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x6000.1<br>
* SubIndex 2 - Reference to 0x6000.2<br>
* SubIndex 3 - Reference to 0x6000.3<br>
* SubIndex 4 - Reference to 0x6000.4<br>
* SubIndex 5 - Reference to 0x6000.5<br>
* SubIndex 6 - Reference to 0x6000.6<br>
* SubIndex 7 - Reference to 0x6000.7<br>
* SubIndex 8 - Reference to 0x6000.8<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1A00[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x6000.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x6000.2 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex3 - Reference to 0x6000.3 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex4 - Reference to 0x6000.4 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex5 - Reference to 0x6000.5 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex6 - Reference to 0x6000.6 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex7 - Reference to 0x6000.7 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex8 - Reference to 0x6000.8 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1A00[] = "switches process data mapping\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000"
"SubIndex 004\000"
"SubIndex 005\000"
"SubIndex 006\000"
"SubIndex 007\000"
"SubIndex 008\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x6000.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x6000.2 */
UINT32 SI3; /* Subindex3 - Reference to 0x6000.3 */
UINT32 SI4; /* Subindex4 - Reference to 0x6000.4 */
UINT32 SI5; /* Subindex5 - Reference to 0x6000.5 */
UINT32 SI6; /* Subindex6 - Reference to 0x6000.6 */
UINT32 SI7; /* Subindex7 - Reference to 0x6000.7 */
UINT32 SI8; /* Subindex8 - Reference to 0x6000.8 */
} OBJ_STRUCT_PACKED_END
TOBJ1A00;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1A00 SwitchesProcessDataMapping0x1A00
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={8,0x60000101,0x60000201,0x60000301,0x60000401,0x60000501,0x60000601,0x60000701,0x60000801}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1A01 : Input mapping 1
******************************************************************************/
/**
* \addtogroup 0x1A01 0x1A01 | Input mapping 1
* @{
* \brief Object 0x1A01 (Input mapping 1) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Reference to 0x6010.1<br>
* SubIndex 2 - Reference to 0x6012.1<br>
* SubIndex 3 - Reference to 0x6014.1<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1A01[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex1 - Reference to 0x6010.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }, /* Subindex2 - Reference to 0x6012.1 */
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ }}; /* Subindex3 - Reference to 0x6014.1 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x1A01[] = "Input mapping 1\000"
"SubIndex 001\000"
"SubIndex 002\000"
"SubIndex 003\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SI1; /* Subindex1 - Reference to 0x6010.1 */
UINT32 SI2; /* Subindex2 - Reference to 0x6012.1 */
UINT32 SI3; /* Subindex3 - Reference to 0x6014.1 */
} OBJ_STRUCT_PACKED_END
TOBJ1A01;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1A01 InputMapping10x1A01
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={3,0x60100120,0x60120110,0x60140120}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1C12 : SyncManager 2 assignment
******************************************************************************/
/**
* \addtogroup 0x1C12 0x1C12 | SyncManager 2 assignment
* @{
* \brief Object 0x1C12 (SyncManager 2 assignment) definition
*/
#ifdef _OBJD_
/**
* \brief Entry descriptions<br>
* 
* Subindex 0<br>
* Subindex 1 - n (the same entry description is used)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C12[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READ }};

/**
* \brief Object name definition<br>
* For Subindex 1 to n the syntax 'Subindex XXX' is used
*/
OBJCONST UCHAR OBJMEM aName0x1C12[] = "SyncManager 2 assignment\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16   u16SubIndex0;  /**< \brief Subindex 0 */
UINT16 aEntries[2];  /**< \brief Subindex 1 - 2 */
} OBJ_STRUCT_PACKED_END
TOBJ1C12;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1C12 sRxPDOassign
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={2,{0x1600,0x1601}}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x1C13 : SyncManager 3 assignment
******************************************************************************/
/**
* \addtogroup 0x1C13 0x1C13 | SyncManager 3 assignment
* @{
* \brief Object 0x1C13 (SyncManager 3 assignment) definition
*/
#ifdef _OBJD_
/**
* \brief Entry descriptions<br>
* 
* Subindex 0<br>
* Subindex 1 - n (the same entry description is used)<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x1C13[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READ }};

/**
* \brief Object name definition<br>
* For Subindex 1 to n the syntax 'Subindex XXX' is used
*/
OBJCONST UCHAR OBJMEM aName0x1C13[] = "SyncManager 3 assignment\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16   u16SubIndex0;  /**< \brief Subindex 0 */
UINT16 aEntries[2];  /**< \brief Subindex 1 - 2 */
} OBJ_STRUCT_PACKED_END
TOBJ1C13;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ1C13 sTxPDOassign
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={2,{0x1A00,0x1A01}}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6000 : switches
******************************************************************************/
/**
* \addtogroup 0x6000 0x6000 | switches
* @{
* \brief Object 0x6000 (switches) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Switch1<br>
* SubIndex 2 - Switch2<br>
* SubIndex 3 - Switch3<br>
* SubIndex 4 - Switch4<br>
* SubIndex 5 - Switch5<br>
* SubIndex 6 - Switch6<br>
* SubIndex 7 - Switch7<br>
* SubIndex 8 - Switch8<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6000[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex1 - Switch1 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex2 - Switch2 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex3 - Switch3 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex4 - Switch4 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex5 - Switch5 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex6 - Switch6 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex7 - Switch7 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex8 - Switch8 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6000[] = "switches\000"
"Switch1\000"
"Switch2\000"
"Switch3\000"
"Switch4\000"
"Switch5\000"
"Switch6\000"
"Switch7\000"
"Switch8\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
BOOLEAN(Switch1); /* Subindex1 - Switch1 */
BOOLEAN(Switch2); /* Subindex2 - Switch2 */
BOOLEAN(Switch3); /* Subindex3 - Switch3 */
BOOLEAN(Switch4); /* Subindex4 - Switch4 */
BOOLEAN(Switch5); /* Subindex5 - Switch5 */
BOOLEAN(Switch6); /* Subindex6 - Switch6 */
BOOLEAN(Switch7); /* Subindex7 - Switch7 */
BOOLEAN(Switch8); /* Subindex8 - Switch8 */
} OBJ_STRUCT_PACKED_END
TOBJ6000;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6000 Switches0x6000
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6010 : DataToMaster
******************************************************************************/
/**
* \addtogroup 0x6010 0x6010 | DataToMaster
* @{
* \brief Object 0x6010 (DataToMaster) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - DataToMaster<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6010[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - DataToMaster */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6010[] = "DataToMaster\000"
"DataToMaster\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 DataToMaster; /* Subindex1 - DataToMaster */
} OBJ_STRUCT_PACKED_END
TOBJ6010;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6010 DataToMaster0x6010
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x00000000}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6012 : TargetModeResponse
******************************************************************************/
/**
* \addtogroup 0x6012 0x6012 | TargetModeResponse
* @{
* \brief Object 0x6012 (TargetModeResponse) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - ModeResponse<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6012[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - ModeResponse */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6012[] = "TargetModeResponse\000"
"ModeResponse\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT16 ModeResponse; /* Subindex1 - ModeResponse */
} OBJ_STRUCT_PACKED_END
TOBJ6012;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6012 TargetModeResponse0x6012
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x0000}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x6014 : TargetSpeedPosFeedback
******************************************************************************/
/**
* \addtogroup 0x6014 0x6014 | TargetSpeedPosFeedback
* @{
* \brief Object 0x6014 (TargetSpeedPosFeedback) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - SpeedPosFbk<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6014[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READ | OBJACCESS_TXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - SpeedPosFbk */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x6014[] = "TargetSpeedPosFeedback\000"
"SpeedPosFbk\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SpeedPosFbk; /* Subindex1 - SpeedPosFbk */
} OBJ_STRUCT_PACKED_END
TOBJ6014;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ6014 TargetSpeedPosFeedback0x6014
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x00000000}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7000 : LEDS
******************************************************************************/
/**
* \addtogroup 0x7000 0x7000 | LEDS
* @{
* \brief Object 0x7000 (LEDS) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - LED1<br>
* SubIndex 2 - LED2<br>
* SubIndex 3 - LED3<br>
* SubIndex 4 - LED4<br>
* SubIndex 5 - LED5<br>
* SubIndex 6 - LED6<br>
* SubIndex 7 - LED7<br>
* SubIndex 8 - LED8<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7000[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex1 - LED1 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex2 - LED2 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex3 - LED3 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex4 - LED4 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex5 - LED5 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex6 - LED6 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }, /* Subindex7 - LED7 */
{ DEFTYPE_BOOLEAN , 0x1 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex8 - LED8 */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7000[] = "LEDS\000"
"LED1\000"
"LED2\000"
"LED3\000"
"LED4\000"
"LED5\000"
"LED6\000"
"LED7\000"
"LED8\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
BOOLEAN(LED1); /* Subindex1 - LED1 */
BOOLEAN(LED2); /* Subindex2 - LED2 */
BOOLEAN(LED3); /* Subindex3 - LED3 */
BOOLEAN(LED4); /* Subindex4 - LED4 */
BOOLEAN(LED5); /* Subindex5 - LED5 */
BOOLEAN(LED6); /* Subindex6 - LED6 */
BOOLEAN(LED7); /* Subindex7 - LED7 */
BOOLEAN(LED8); /* Subindex8 - LED8 */
} OBJ_STRUCT_PACKED_END
TOBJ7000;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7000 LEDS0x7000
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7010 : DatafromMaster
******************************************************************************/
/**
* \addtogroup 0x7010 0x7010 | DatafromMaster
* @{
* \brief Object 0x7010 (DatafromMaster) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - DatafromMaster<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7010[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - DatafromMaster */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7010[] = "DatafromMaster\000"
"DatafromMaster\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 DatafromMaster; /* Subindex1 - DatafromMaster */
} OBJ_STRUCT_PACKED_END
TOBJ7010;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7010 DatafromMaster0x7010
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x00000000}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7012 : TargetMode
******************************************************************************/
/**
* \addtogroup 0x7012 0x7012 | TargetMode
* @{
* \brief Object 0x7012 (TargetMode) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - Mode<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7012[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED16 , 0x10 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - Mode */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7012[] = "TargetMode\000"
"Mode\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT16 Mode; /* Subindex1 - Mode */
} OBJ_STRUCT_PACKED_END
TOBJ7012;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7012 TargetMode0x7012
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x0000}
#endif
;
/** @}*/



/******************************************************************************
*                    Object 0x7014 : TargetSpeedPosReq
******************************************************************************/
/**
* \addtogroup 0x7014 0x7014 | TargetSpeedPosReq
* @{
* \brief Object 0x7014 (TargetSpeedPosReq) definition
*/
#ifdef _OBJD_
/**
* \brief Object entry descriptions<br>
* <br>
* SubIndex 0<br>
* SubIndex 1 - SpeedPosReq<br>
*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x7014[] = {
{ DEFTYPE_UNSIGNED8 , 0x8 , ACCESS_READ },
{ DEFTYPE_UNSIGNED32 , 0x20 , ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_SETTINGS }}; /* Subindex1 - SpeedPosReq */

/**
* \brief Object/Entry names
*/
OBJCONST UCHAR OBJMEM aName0x7014[] = "TargetSpeedPosReq\000"
"SpeedPosReq\000\377";
#endif //#ifdef _OBJD_

#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
/**
* \brief Object structure
*/
typedef struct OBJ_STRUCT_PACKED_START {
UINT16 u16SubIndex0;
UINT32 SpeedPosReq; /* Subindex1 - SpeedPosReq */
} OBJ_STRUCT_PACKED_END
TOBJ7014;
#endif //#ifndef _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_

/**
* \brief Object variable
*/
PROTO TOBJ7014 TargetSpeedPosReq0x7014
#if defined(_TMDSECATCNCD379_D_ECHO_BACK_) && (_TMDSECATCNCD379_D_ECHO_BACK_ == 1)
={1,0x00000000}
#endif
;
/** @}*/







#ifdef _OBJD_
TOBJECT    OBJMEM ApplicationObjDic[] = {
/* Object 0x1600 */
{NULL , NULL ,  0x1600 , {DEFTYPE_PDOMAPPING , 8 | (OBJCODE_REC << 8)} , asEntryDesc0x1600 , aName0x1600 , &LEDSProcessDataMapping0x1600, NULL , NULL , 0x0000 },
/* Object 0x1601 */
{NULL , NULL ,  0x1601 , {DEFTYPE_PDOMAPPING , 3 | (OBJCODE_REC << 8)} , asEntryDesc0x1601 , aName0x1601 , &OutputMapping10x1601, NULL , NULL , 0x0000 },
/* Object 0x1A00 */
{NULL , NULL ,  0x1A00 , {DEFTYPE_PDOMAPPING , 8 | (OBJCODE_REC << 8)} , asEntryDesc0x1A00 , aName0x1A00 , &SwitchesProcessDataMapping0x1A00, NULL , NULL , 0x0000 },
/* Object 0x1A01 */
{NULL , NULL ,  0x1A01 , {DEFTYPE_PDOMAPPING , 3 | (OBJCODE_REC << 8)} , asEntryDesc0x1A01 , aName0x1A01 , &InputMapping10x1A01, NULL , NULL , 0x0000 },
/* Object 0x1C12 */
{NULL , NULL ,  0x1C12 , {DEFTYPE_UNSIGNED16 , 2 | (OBJCODE_ARR << 8)} , asEntryDesc0x1C12 , aName0x1C12 , &sRxPDOassign, NULL , NULL , 0x0000 },
/* Object 0x1C13 */
{NULL , NULL ,  0x1C13 , {DEFTYPE_UNSIGNED16 , 2 | (OBJCODE_ARR << 8)} , asEntryDesc0x1C13 , aName0x1C13 , &sTxPDOassign, NULL , NULL , 0x0000 },
/* Object 0x6000 */
{NULL , NULL ,  0x6000 , {DEFTYPE_RECORD , 8 | (OBJCODE_REC << 8)} , asEntryDesc0x6000 , aName0x6000 , &Switches0x6000, NULL , NULL , 0x0000 },
/* Object 0x6010 */
{NULL , NULL ,  0x6010 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x6010 , aName0x6010 , &DataToMaster0x6010, NULL , NULL , 0x0000 },
/* Object 0x6012 */
{NULL , NULL ,  0x6012 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x6012 , aName0x6012 , &TargetModeResponse0x6012, NULL , NULL , 0x0000 },
/* Object 0x6014 */
{NULL , NULL ,  0x6014 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x6014 , aName0x6014 , &TargetSpeedPosFeedback0x6014, NULL , NULL , 0x0000 },
/* Object 0x7000 */
{NULL , NULL ,  0x7000 , {DEFTYPE_RECORD , 8 | (OBJCODE_REC << 8)} , asEntryDesc0x7000 , aName0x7000 , &LEDS0x7000, NULL , NULL , 0x0000 },
/* Object 0x7010 */
{NULL , NULL ,  0x7010 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x7010 , aName0x7010 , &DatafromMaster0x7010, NULL , NULL , 0x0000 },
/* Object 0x7012 */
{NULL , NULL ,  0x7012 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x7012 , aName0x7012 , &TargetMode0x7012, NULL , NULL , 0x0000 },
/* Object 0x7014 */
{NULL , NULL ,  0x7014 , {DEFTYPE_RECORD , 1 | (OBJCODE_REC << 8)} , asEntryDesc0x7014 , aName0x7014 , &TargetSpeedPosReq0x7014, NULL , NULL , 0x0000 },
{NULL,NULL, 0xFFFF, {0, 0}, NULL, NULL, NULL, NULL}};
#endif    //#ifdef _OBJD_
#undef PROTO

/** @}*/
#define _TMDSECATCNCD379_D_ECHO_BACK_OBJECTS_H_
