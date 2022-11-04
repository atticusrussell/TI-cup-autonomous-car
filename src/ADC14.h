#ifndef _ADC_
#define _ADC_
// ADC14.h
// Runs on MSP432
// ADC input, software trigger, 14-bit conversion,
// 2.5 V static (always on) reference
// Jonathan Valvano
// June 9, 2015


// Modified by
// LJBeato
// 2021

void ADC0_InitSWTriggerCh6(void);

unsigned int ADC_In(void);

//*****************************************************************************
// ADC14 Registers
//*****************************************************************************
#define ADC14CTL0                                          (HWREG32(0x40012000)) /* Control 0 Register  */
#define ADC14CTL1                                          (HWREG32(0x40012004)) /* Control 1 Register */
#define ADC14LO0                                           (HWREG32(0x40012008)) /* Window Comparator Low Threshold 0 Register  */
#define ADC14HI0                                           (HWREG32(0x4001200C)) /* Window Comparator High Threshold 0 Register  */
#define ADC14LO1                                           (HWREG32(0x40012010)) /* Window Comparator Low Threshold 1 Register  */
#define ADC14HI1                                           (HWREG32(0x40012014)) /* Window Comparator High Threshold 1 Register  */
#define ADC14MCTL0                                         (HWREG32(0x40012018)) /* Conversion Memory Control Register */
#define ADC14MCTL1                                         (HWREG32(0x4001201C)) /* Conversion Memory Control Register */
#define ADC14MCTL2                                         (HWREG32(0x40012020)) /* Conversion Memory Control Register */
#define ADC14MCTL3                                         (HWREG32(0x40012024)) /* Conversion Memory Control Register */
#define ADC14MCTL4                                         (HWREG32(0x40012028)) /* Conversion Memory Control Register */
#define ADC14MCTL5                                         (HWREG32(0x4001202C)) /* Conversion Memory Control Register */
#define ADC14MCTL6                                         (HWREG32(0x40012030)) /* Conversion Memory Control Register */
#define ADC14MCTL7                                         (HWREG32(0x40012034)) /* Conversion Memory Control Register */
#define ADC14MCTL8                                         (HWREG32(0x40012038)) /* Conversion Memory Control Register */
#define ADC14MCTL9                                         (HWREG32(0x4001203C)) /* Conversion Memory Control Register */
#define ADC14MCTL10                                        (HWREG32(0x40012040)) /* Conversion Memory Control Register */
#define ADC14MCTL11                                        (HWREG32(0x40012044)) /* Conversion Memory Control Register */
#define ADC14MCTL12                                        (HWREG32(0x40012048)) /* Conversion Memory Control Register */
#define ADC14MCTL13                                        (HWREG32(0x4001204C)) /* Conversion Memory Control Register */
#define ADC14MCTL14                                        (HWREG32(0x40012050)) /* Conversion Memory Control Register */
#define ADC14MCTL15                                        (HWREG32(0x40012054)) /* Conversion Memory Control Register */
#define ADC14MCTL16                                        (HWREG32(0x40012058)) /* Conversion Memory Control Register */
#define ADC14MCTL17                                        (HWREG32(0x4001205C)) /* Conversion Memory Control Register */
#define ADC14MCTL18                                        (HWREG32(0x40012060)) /* Conversion Memory Control Register */
#define ADC14MCTL19                                        (HWREG32(0x40012064)) /* Conversion Memory Control Register */
#define ADC14MCTL20                                        (HWREG32(0x40012068)) /* Conversion Memory Control Register */
#define ADC14MCTL21                                        (HWREG32(0x4001206C)) /* Conversion Memory Control Register */
#define ADC14MCTL22                                        (HWREG32(0x40012070)) /* Conversion Memory Control Register */
#define ADC14MCTL23                                        (HWREG32(0x40012074)) /* Conversion Memory Control Register */
#define ADC14MCTL24                                        (HWREG32(0x40012078)) /* Conversion Memory Control Register */
#define ADC14MCTL25                                        (HWREG32(0x4001207C)) /* Conversion Memory Control Register */
#define ADC14MCTL26                                        (HWREG32(0x40012080)) /* Conversion Memory Control Register */
#define ADC14MCTL27                                        (HWREG32(0x40012084)) /* Conversion Memory Control Register */
#define ADC14MCTL28                                        (HWREG32(0x40012088)) /* Conversion Memory Control Register */
#define ADC14MCTL29                                        (HWREG32(0x4001208C)) /* Conversion Memory Control Register */
#define ADC14MCTL30                                        (HWREG32(0x40012090)) /* Conversion Memory Control Register */
#define ADC14MCTL31                                        (HWREG32(0x40012094)) /* Conversion Memory Control Register */
#define ADC14MEM0                                          (HWREG32(0x40012098)) /* Conversion Memory Register */
#define ADC14MEM1                                          (HWREG32(0x4001209C)) /* Conversion Memory Register */
#define ADC14MEM2                                          (HWREG32(0x400120A0)) /* Conversion Memory Register */
#define ADC14MEM3                                          (HWREG32(0x400120A4)) /* Conversion Memory Register */
#define ADC14MEM4                                          (HWREG32(0x400120A8)) /* Conversion Memory Register */
#define ADC14MEM5                                          (HWREG32(0x400120AC)) /* Conversion Memory Register */
#define ADC14MEM6                                          (HWREG32(0x400120B0)) /* Conversion Memory Register */
#define ADC14MEM7                                          (HWREG32(0x400120B4)) /* Conversion Memory Register */
#define ADC14MEM8                                          (HWREG32(0x400120B8)) /* Conversion Memory Register */
#define ADC14MEM9                                          (HWREG32(0x400120BC)) /* Conversion Memory Register */
#define ADC14MEM10                                         (HWREG32(0x400120C0)) /* Conversion Memory Register */
#define ADC14MEM11                                         (HWREG32(0x400120C4)) /* Conversion Memory Register */
#define ADC14MEM12                                         (HWREG32(0x400120C8)) /* Conversion Memory Register */
#define ADC14MEM13                                         (HWREG32(0x400120CC)) /* Conversion Memory Register */
#define ADC14MEM14                                         (HWREG32(0x400120D0)) /* Conversion Memory Register */
#define ADC14MEM15                                         (HWREG32(0x400120D4)) /* Conversion Memory Register */
#define ADC14MEM16                                         (HWREG32(0x400120D8)) /* Conversion Memory Register */
#define ADC14MEM17                                         (HWREG32(0x400120DC)) /* Conversion Memory Register */
#define ADC14MEM18                                         (HWREG32(0x400120E0)) /* Conversion Memory Register */
#define ADC14MEM19                                         (HWREG32(0x400120E4)) /* Conversion Memory Register */
#define ADC14MEM20                                         (HWREG32(0x400120E8)) /* Conversion Memory Register */
#define ADC14MEM21                                         (HWREG32(0x400120EC)) /* Conversion Memory Register */
#define ADC14MEM22                                         (HWREG32(0x400120F0)) /* Conversion Memory Register */
#define ADC14MEM23                                         (HWREG32(0x400120F4)) /* Conversion Memory Register */
#define ADC14MEM24                                         (HWREG32(0x400120F8)) /* Conversion Memory Register */
#define ADC14MEM25                                         (HWREG32(0x400120FC)) /* Conversion Memory Register */
#define ADC14MEM26                                         (HWREG32(0x40012100)) /* Conversion Memory Register */
#define ADC14MEM27                                         (HWREG32(0x40012104)) /* Conversion Memory Register */
#define ADC14MEM28                                         (HWREG32(0x40012108)) /* Conversion Memory Register */
#define ADC14MEM29                                         (HWREG32(0x4001210C)) /* Conversion Memory Register */
#define ADC14MEM30                                         (HWREG32(0x40012110)) /* Conversion Memory Register */
#define ADC14MEM31                                         (HWREG32(0x40012114)) /* Conversion Memory Register */
#define ADC14IER0                                          (HWREG32(0x4001213C)) /* Interrupt Enable 0 Register  */
#define ADC14IER1                                          (HWREG32(0x40012140)) /* Interrupt Enable 1 Register  */
#define ADC14IFGR0                                         (HWREG32(0x40012144)) /* Interrupt Flag 0 Register  */
#define ADC14IFGR1                                         (HWREG32(0x40012148)) /* Interrupt Flag 1 Register  */
#define ADC14CLRIFGR0                                      (HWREG32(0x4001214C)) /* Clear Interrupt Flag 0 Register  */
#define ADC14CLRIFGR1                                      (HWREG32(0x40012150)) /* Clear Interrupt Flag 1 Register  */
#define ADC14IV                                            (HWREG32(0x40012154)) /* Interrupt Vector Register */

/* Register offsets from ADC14_BASE address */
#define OFS_ADC14CTL0                                      (0x00000000)          /* Control 0 Register  */
#define OFS_ADC14CTL1                                      (0x00000004)          /* Control 1 Register */
#define OFS_ADC14LO0                                       (0x00000008)          /* Window Comparator Low Threshold 0 Register  */
#define OFS_ADC14HI0                                       (0x0000000c)          /* Window Comparator High Threshold 0 Register  */
#define OFS_ADC14LO1                                       (0x00000010)          /* Window Comparator Low Threshold 1 Register  */
#define OFS_ADC14HI1                                       (0x00000014)          /* Window Comparator High Threshold 1 Register  */
#define OFS_ADC14MCTL0                                     (0x00000018)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL1                                     (0x0000001C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL2                                     (0x00000020)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL3                                     (0x00000024)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL4                                     (0x00000028)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL5                                     (0x0000002C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL6                                     (0x00000030)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL7                                     (0x00000034)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL8                                     (0x00000038)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL9                                     (0x0000003C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL10                                    (0x00000040)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL11                                    (0x00000044)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL12                                    (0x00000048)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL13                                    (0x0000004C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL14                                    (0x00000050)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL15                                    (0x00000054)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL16                                    (0x00000058)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL17                                    (0x0000005C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL18                                    (0x00000060)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL19                                    (0x00000064)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL20                                    (0x00000068)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL21                                    (0x0000006C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL22                                    (0x00000070)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL23                                    (0x00000074)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL24                                    (0x00000078)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL25                                    (0x0000007C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL26                                    (0x00000080)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL27                                    (0x00000084)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL28                                    (0x00000088)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL29                                    (0x0000008C)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL30                                    (0x00000090)          /* Conversion Memory Control Register */
#define OFS_ADC14MCTL31                                    (0x00000094)          /* Conversion Memory Control Register */
#define OFS_ADC14MEM0                                      (0x00000098)          /* Conversion Memory Register */
#define OFS_ADC14MEM1                                      (0x0000009C)          /* Conversion Memory Register */
#define OFS_ADC14MEM2                                      (0x000000A0)          /* Conversion Memory Register */
#define OFS_ADC14MEM3                                      (0x000000A4)          /* Conversion Memory Register */
#define OFS_ADC14MEM4                                      (0x000000A8)          /* Conversion Memory Register */
#define OFS_ADC14MEM5                                      (0x000000AC)          /* Conversion Memory Register */
#define OFS_ADC14MEM6                                      (0x000000B0)          /* Conversion Memory Register */
#define OFS_ADC14MEM7                                      (0x000000B4)          /* Conversion Memory Register */
#define OFS_ADC14MEM8                                      (0x000000B8)          /* Conversion Memory Register */
#define OFS_ADC14MEM9                                      (0x000000BC)          /* Conversion Memory Register */
#define OFS_ADC14MEM10                                     (0x000000C0)          /* Conversion Memory Register */
#define OFS_ADC14MEM11                                     (0x000000C4)          /* Conversion Memory Register */
#define OFS_ADC14MEM12                                     (0x000000C8)          /* Conversion Memory Register */
#define OFS_ADC14MEM13                                     (0x000000CC)          /* Conversion Memory Register */
#define OFS_ADC14MEM14                                     (0x000000D0)          /* Conversion Memory Register */
#define OFS_ADC14MEM15                                     (0x000000D4)          /* Conversion Memory Register */
#define OFS_ADC14MEM16                                     (0x000000D8)          /* Conversion Memory Register */
#define OFS_ADC14MEM17                                     (0x000000DC)          /* Conversion Memory Register */
#define OFS_ADC14MEM18                                     (0x000000E0)          /* Conversion Memory Register */
#define OFS_ADC14MEM19                                     (0x000000E4)          /* Conversion Memory Register */
#define OFS_ADC14MEM20                                     (0x000000E8)          /* Conversion Memory Register */
#define OFS_ADC14MEM21                                     (0x000000EC)          /* Conversion Memory Register */
#define OFS_ADC14MEM22                                     (0x000000F0)          /* Conversion Memory Register */
#define OFS_ADC14MEM23                                     (0x000000F4)          /* Conversion Memory Register */
#define OFS_ADC14MEM24                                     (0x000000F8)          /* Conversion Memory Register */
#define OFS_ADC14MEM25                                     (0x000000FC)          /* Conversion Memory Register */
#define OFS_ADC14MEM26                                     (0x00000100)          /* Conversion Memory Register */
#define OFS_ADC14MEM27                                     (0x00000104)          /* Conversion Memory Register */
#define OFS_ADC14MEM28                                     (0x00000108)          /* Conversion Memory Register */
#define OFS_ADC14MEM29                                     (0x0000010C)          /* Conversion Memory Register */
#define OFS_ADC14MEM30                                     (0x00000110)          /* Conversion Memory Register */
#define OFS_ADC14MEM31                                     (0x00000114)          /* Conversion Memory Register */
#define OFS_ADC14IER0                                      (0x0000013c)          /* Interrupt Enable 0 Register  */
#define OFS_ADC14IER1                                      (0x00000140)          /* Interrupt Enable 1 Register  */
#define OFS_ADC14IFGR0                                     (0x00000144)          /* Interrupt Flag 0 Register  */
#define OFS_ADC14IFGR1                                     (0x00000148)          /* Interrupt Flag 1 Register  */
#define OFS_ADC14CLRIFGR0                                  (0x0000014c)          /* Clear Interrupt Flag 0 Register  */
#define OFS_ADC14CLRIFGR1                                  (0x00000150)          /* Clear Interrupt Flag 1 Register  */
#define OFS_ADC14IV                                        (0x00000154)          /* Interrupt Vector Register */
#endif

