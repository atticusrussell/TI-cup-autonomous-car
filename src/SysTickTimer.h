#ifndef _SYSTICKTIMER_
#define _SYSTICKTIMER_
#include "msp.h"
#include "Common.h"
#include "CortexM.h"


///////////////////////////////////////////////////////////////////////////////
//
// SYSTICK Registers
//
///////////////////////////////////////////////////////////////////////////////
#define SYSTICK_STCSR                                      (HWREG32(0xE000E010)) // SysTick Control and Status Register 
#define SYSTICK_STRVR                                      (HWREG32(0xE000E014)) // SysTick Reload Value Register 
#define SYSTICK_STCVR                                      (HWREG32(0xE000E018)) // SysTick Current Value Register 
#define SYSTICK_STCR                                       (HWREG32(0xE000E01C)) // SysTick Calibration Value Register 

// Register offsets from SYSTICK_BASE address 
#define OFS_SYSTICK_STCSR                                  (0x00000010)          // SysTick Control and Status Register
#define OFS_SYSTICK_STRVR                                  (0x00000014)          // SysTick Reload Value Register 
#define OFS_SYSTICK_STCVR                                  (0x00000018)          // SysTick Current Value Register 
#define OFS_SYSTICK_STCR                                   (0x0000001C)          // SysTick Calibration Value Register 


#define DISABLE_SYSTICK_CLOCK	0
#define ENABLE_SYSTICK_CLOCK   0x00000007 

void SysTickTimer_Init(void(*task)(void), unsigned long period);


void DisableSysTickTimer(void);
void EnableSysTickTimer(void);



#endif
