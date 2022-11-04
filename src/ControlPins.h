#ifndef _CONTROL_PINS_
#define _CONTROL_PINS_
#include "Common.h"
#include "msp.h"

// SI Pin will be P5.5 A0
#define SI BIT5
// CLK Pin will be P5.4 A1
#define CLK BIT4

extern uint16_t line[128];
extern BOOLEAN g_sendData;
void ControlPin_SI_Init(void);
void ControlPin_CLK_Init(void);
void CLK_Handler(void);
void SI_Handler(void);

#endif
