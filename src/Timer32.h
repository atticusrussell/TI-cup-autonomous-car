#ifndef _TIMER32_
#define _TIMER32_


#include "msp.h"


//*****************************************************************************
// TIMER32 Registers
//*****************************************************************************
#define TIMER32_LOAD1                                      (HWREG32(0x4000C000)) /* Timer 1 Load Register */
#define TIMER32_VALUE1                                     (HWREG32(0x4000C004)) /* Timer 1 Current Value Register */
#define TIMER32_CONTROL1                                   (HWREG32(0x4000C008)) /* Timer 1 Timer Control Register */
#define TIMER32_INTCLR1                                    (HWREG32(0x4000C00C)) /* Timer 1 Interrupt Clear Register */
#define TIMER32_RIS1                                       (HWREG32(0x4000C010)) /* Timer 1 Raw Interrupt Status Register */
#define TIMER32_MIS1                                       (HWREG32(0x4000C014)) /* Timer 1 Interrupt Status Register */
#define TIMER32_BGLOAD1                                    (HWREG32(0x4000C018)) /* Timer 1 Background Load Register */
#define TIMER32_LOAD2                                      (HWREG32(0x4000C020)) /* Timer 2 Load Register */
#define TIMER32_VALUE2                                     (HWREG32(0x4000C024)) /* Timer 2 Current Value Register */
#define TIMER32_CONTROL2                                   (HWREG32(0x4000C028)) /* Timer 2 Timer Control Register */
#define TIMER32_INTCLR2                                    (HWREG32(0x4000C02C)) /* Timer 2 Interrupt Clear Register */
#define TIMER32_RIS2                                       (HWREG32(0x4000C030)) /* Timer 2 Raw Interrupt Status Register */
#define TIMER32_MIS2                                       (HWREG32(0x4000C034)) /* Timer 2 Interrupt Status Register */
#define TIMER32_BGLOAD2                                    (HWREG32(0x4000C038)) /* Timer 2 Background Load Register */

/* Register offsets from TIMER32_BASE address */
#define OFS_TIMER32_LOAD1                                  (0x00000000)          /* Timer 1 Load Register */
#define OFS_TIMER32_VALUE1                                 (0x00000004)          /* Timer 1 Current Value Register */
#define OFS_TIMER32_CONTROL1                               (0x00000008)          /* Timer 1 Timer Control Register */
#define OFS_TIMER32_INTCLR1                                (0x0000000C)          /* Timer 1 Interrupt Clear Register */
#define OFS_TIMER32_RIS1                                   (0x00000010)          /* Timer 1 Raw Interrupt Status Register */
#define OFS_TIMER32_MIS1                                   (0x00000014)          /* Timer 1 Interrupt Status Register */
#define OFS_TIMER32_BGLOAD1                                (0x00000018)          /* Timer 1 Background Load Register */
#define OFS_TIMER32_LOAD2                                  (0x00000020)          /* Timer 2 Load Register */
#define OFS_TIMER32_VALUE2                                 (0x00000024)          /* Timer 2 Current Value Register */
#define OFS_TIMER32_CONTROL2                               (0x00000028)          /* Timer 2 Timer Control Register */
#define OFS_TIMER32_INTCLR2                                (0x0000002C)          /* Timer 2 Interrupt Clear Register */
#define OFS_TIMER32_RIS2                                   (0x00000030)          /* Timer 2 Raw Interrupt Status Register */
#define OFS_TIMER32_MIS2                                   (0x00000034)          /* Timer 2 Interrupt Status Register */
#define OFS_TIMER32_BGLOAD2                                (0x00000038)          /* Timer 2 Background Load Register */

enum timer32divider{
  T32DIV1 = 0x00000000,            // maximum period of about 89 seconds at 48 MHz
  T32DIV16 = 0x00000004,           // maximum period of about 23 minutes at 48 MHz
  T32DIV256 = 0x00000008           // maximum period of about 6 hours, 21 minutes at 48 MHz
};

unsigned long  CalcPeriodFromFrequency (double Hz);


	
// ***************** Timer32_1_Init ****************
// Activate Timer32 Timer 1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/(bus clock)/div), 32 bits
//          div is clock divider for Timer32 Timer 1
//            T32DIV1   for input clock divider /1
//            T32DIV16  for input clock divider /16
//            T32DIV256 for input clock divider /256
// Outputs: none
void Timer32_1_Init(void(*task)(void), unsigned long period, enum timer32divider div);

// ***************** Timer32_2_Init ****************
// Activate Timer32 Timer 2 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/(bus clock)/div), 32 bits
//          div is clock divider for Timer32 Timer 2
//            T32DIV1   for input clock divider /1
//            T32DIV16  for input clock divider /16
//            T32DIV256 for input clock divider /256
// Outputs: none
void Timer32_2_Init(void(*task)(void), unsigned long period, enum timer32divider div);

#endif

