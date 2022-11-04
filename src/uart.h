/*
 * Author:		Atticus Russell
 * Date: 		9/22/2022
 * File:        uart.h
 * Purpose:     header file for UART routines for serial IO
 * Exercise:	Lab04_BLE
 *
 * Notes:		
 *
 */

#include "Common.h"

#ifndef _UART_HEADER_FILE_
#define _UART_HEADER_FILE_

/* Function prototypes */
void uart0_init(void);
BYTE uart0_getchar(void);
void uart0_putchar(char ch);
void uart0_put(char *ptr_str);
int uart0_dataAvailable(void);

void uart2_init(void);
BYTE uart2_getchar(void);
void uart2_putchar(char ch);
void uart2_put(char *ptr_str);
int uart2_dataAvailable(void);

#endif
