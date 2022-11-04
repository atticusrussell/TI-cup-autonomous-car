#ifndef _COMMON_
#define _COMMON_

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif
#define SQR(x) (x*x)

#define HIGH_CLOCK_SPEED 48000000    // MHz
// default CLK frequency of the camera 100KHz (assume 3MHz clock)
// NOTE: we have to double 100000, because we need a clock for the rising edge and one for the falling edge.
//#define CLK_PERIOD ((double)DEFAULT_CLOCK_SPEED/200000.0) 

#define DEFAULT_CLOCK_SPEED 3000000    // MHz


///****************************************************************************
// NVIC Registers
///****************************************************************************
#ifndef NVIC_ISER0
#define NVIC_ISER0                                         (HWREG32(0xE000E100)) // Irq 0 to 31 Set Enable Register 
#endif

#ifndef NVIC_ISER1
#define NVIC_ISER1                                         (HWREG32(0xE000E104)) // Irq 32 to 63 Set Enable Register 
#endif

#ifndef NVIC_ICER0
#define NVIC_ICER0                                         (HWREG32(0xE000E180)) // Irq 0 to 31 Clear Enable Register 
#endif

#ifndef NVIC_ICER1
#define NVIC_ICER1                                         (HWREG32(0xE000E184)) // Irq 32 to 63 Clear Enable Register 
#endif

#ifndef NVIC_ISPR0
#define NVIC_ISPR0                                         (HWREG32(0xE000E200)) // Irq 0 to 31 Set Pending Register 
#endif

#ifndef NVIC_ISPR1
#define NVIC_ISPR1                                         (HWREG32(0xE000E204)) // Irq 32 to 63 Set Pending Register 
#endif

#ifndef NVIC_ICPR0
#define NVIC_ICPR0                                         (HWREG32(0xE000E280)) // Irq 0 to 31 Clear Pending Register 
#endif

#ifndef NVIC_ICPR1
#define NVIC_ICPR1                                         (HWREG32(0xE000E284)) // Irq 32 to 63 Clear Pending Register 
#endif

#ifndef NVIC_IABR0
#define NVIC_IABR0                                         (HWREG32(0xE000E300)) // Irq 0 to 31 Active Bit Register 
#endif

#ifndef NVIC_IABR1
#define NVIC_IABR1                                         (HWREG32(0xE000E304)) // Irq 32 to 63 Active Bit Register 
#endif

#ifndef NVIC_IPR0
#define NVIC_IPR0                                          (HWREG32(0xE000E400)) // Irq 0 to 3 Priority Register 
#endif

#ifndef NVIC_IPR1
#define NVIC_IPR1                                          (HWREG32(0xE000E404)) // Irq 4 to 7 Priority Register 
#endif

#ifndef NVIC_IPR2
#define NVIC_IPR2                                          (HWREG32(0xE000E408)) // Irq 8 to 11 Priority Register 
#endif

#ifndef NVIC_IPR3
#define NVIC_IPR3                                          (HWREG32(0xE000E40C)) // Irq 12 to 15 Priority Register 
#endif

#ifndef NVIC_IPR4
#define NVIC_IPR4                                          (HWREG32(0xE000E410)) // Irq 16 to 19 Priority Register 
#endif

#ifndef NVIC_IPR5
#define NVIC_IPR5                                          (HWREG32(0xE000E414)) // Irq 20 to 23 Priority Register 
#endif

#ifndef NVIC_IPR6
#define NVIC_IPR6                                          (HWREG32(0xE000E418)) // Irq 24 to 27 Priority Register 
#endif

#ifndef NVIC_IPR7
#define NVIC_IPR7                                          (HWREG32(0xE000E41C)) // Irq 28 to 31 Priority Register 
#endif

#ifndef NVIC_IPR8
#define NVIC_IPR8                                          (HWREG32(0xE000E420)) // Irq 32 to 35 Priority Register 
#endif

#ifndef NVIC_IPR9
#define NVIC_IPR9                                          (HWREG32(0xE000E424)) // Irq 36 to 39 Priority Register 
#endif

#ifndef NVIC_IPR10
#define NVIC_IPR10                                         (HWREG32(0xE000E428)) // Irq 40 to 43 Priority Register 
#endif

#ifndef NVIC_IPR11
#define NVIC_IPR11                                         (HWREG32(0xE000E42C)) // Irq 44 to 47 Priority Register 
#endif

#ifndef NVIC_IPR12
#define NVIC_IPR12                                         (HWREG32(0xE000E430)) // Irq 48 to 51 Priority Register 
#endif

#ifndef NVIC_IPR13
#define NVIC_IPR13                                         (HWREG32(0xE000E434)) // Irq 52 to 55 Priority Register 
#endif

#ifndef NVIC_IPR14
#define NVIC_IPR14                                         (HWREG32(0xE000E438)) // Irq 56 to 59 Priority Register 
#endif

#ifndef NVIC_IPR15
#define NVIC_IPR15                                         (HWREG32(0xE000E43C)) // Irq 60 to 63 Priority Register 
#endif

#ifndef NVIC_STIR
#define NVIC_STIR                                          (HWREG32(0xE000EF00)) // Software Trigger Interrupt Register 
#endif
///****************************************************************************
// SCB Registers
///****************************************************************************
#ifndef SCB_CPUID
#define SCB_CPUID                                          (HWREG32(0xE000ED00)) // CPUID Base Register 
#endif

#ifndef SCB_ICSR
#define SCB_ICSR                                           (HWREG32(0xE000ED04)) // Interrupt Control State Register 
#endif

#ifndef SCB_VTOR
#define SCB_VTOR                                           (HWREG32(0xE000ED08)) // Vector Table Offset Register 
#endif


#ifndef SCB_AIRCR
#define SCB_AIRCR                                          (HWREG32(0xE000ED0C)) // Application Interrupt/Reset Control Register 
#endif

#ifndef SCB_SCR
#define SCB_SCR                                            (HWREG32(0xE000ED10)) // System Control Register 
#endif


#ifndef SCB_CCR
#define SCB_CCR                                            (HWREG32(0xE000ED14)) // Configuration Control Register 
#endif


#ifndef SCB_SHPR1
#define SCB_SHPR1                                          (HWREG32(0xE000ED18)) // System Handlers 4-7 Priority Register 
#endif


#ifndef SCB_SHPR2
#define SCB_SHPR2                                          (HWREG32(0xE000ED1C)) // System Handlers 8-11 Priority Register 
#endif


#ifndef SCB_SHPR3
#define SCB_SHPR3                                          (HWREG32(0xE000ED20)) // System Handlers 12-15 Priority Register 
#endif


#ifndef SCB_SHCSR
#define SCB_SHCSR                                          (HWREG32(0xE000ED24)) // System Handler Control and State Register 
#endif


#ifndef SCB_CFSR
#define SCB_CFSR                                           (HWREG32(0xE000ED28)) // Configurable Fault Status Registers 
#endif


#ifndef SCB_HFSR
#define SCB_HFSR                                           (HWREG32(0xE000ED2C)) // Hard Fault Status Register 
#endif


#ifndef SCB_DFSR
#define SCB_DFSR                                           (HWREG32(0xE000ED30)) // Debug Fault Status Register 
#endif


#ifndef SCB_MMFAR
#define SCB_MMFAR                                          (HWREG32(0xE000ED34)) // Mem Manage Fault Address Register 
#endif


#ifndef SCB_BFAR
#define SCB_BFAR                                           (HWREG32(0xE000ED38)) // Bus Fault Address Register 
#endif


#ifndef SCB_AFSR
#define SCB_AFSR                                           (HWREG32(0xE000ED3C)) // Auxiliary Fault Status Register 
#endif


#ifndef SCB_PFR0
#define SCB_PFR0                                           (HWREG32(0xE000ED40)) // Processor Feature register0 
#endif


#ifndef SCB_PFR1
#define SCB_PFR1                                           (HWREG32(0xE000ED44)) // Processor Feature register1 
#endif


#ifndef SCB_DFR0
#define SCB_DFR0                                           (HWREG32(0xE000ED48)) // Debug Feature register0 
#endif


#ifndef SCB_AFR0
#define SCB_AFR0                                           (HWREG32(0xE000ED4C)) // Auxiliary Feature register0 
#endif


#ifndef SCB_MMFR0
#define SCB_MMFR0                                          (HWREG32(0xE000ED50)) // Memory Model Feature register0 
#endif


#ifndef SCB_MMFR1
#define SCB_MMFR1                                          (HWREG32(0xE000ED54)) // Memory Model Feature register1 
#endif


#ifndef SCB_MMFR2
#define SCB_MMFR2                                          (HWREG32(0xE000ED58)) // Memory Model Feature register2 
#endif


#ifndef SCB_MMFR3
#define SCB_MMFR3                                          (HWREG32(0xE000ED5C)) // Memory Model Feature register3 
#endif


#ifndef SCB_ISAR0
#define SCB_ISAR0                                          (HWREG32(0xE000ED60)) // ISA Feature register0 
#endif


#ifndef SCB_ISAR1
#define SCB_ISAR1                                          (HWREG32(0xE000ED64)) // ISA Feature register1 
#endif


#ifndef SCB_ISAR2
#define SCB_ISAR2                                          (HWREG32(0xE000ED68)) // ISA Feature register2 
#endif


#ifndef SCB_ISAR3
#define SCB_ISAR3                                          (HWREG32(0xE000ED6C)) // ISA Feature register3 
#endif


#ifndef SCB_ISAR4
#define SCB_ISAR4                                          (HWREG32(0xE000ED70)) // ISA Feature register4 
#endif


#ifndef SCB_CPACR
#define SCB_CPACR                                          (HWREG32(0xE000ED88)) // Coprocessor Access Control Register 
#endif


#endif
