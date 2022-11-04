// ServoMotor.c

/*  Erica Coles
    2022
    Servo Motor functionality 
 */


#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"
#include "Uart.h"


/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}


int main(void){
	// Initialize UART and PWM
	uart0_init();

	// Print welcome over serial
	uart0_put("Running... \n\r");
	
	
	//initialize cycles for period
	// uint32_t clkFrq = 3000000; // 3 MHz
	// uint16_t desiredServoFrq = 50; // 50 Hz
	// uint16_t period = clkFrq / desiredServoFrq;
	uint16_t timerPeriod = 60000;
	uint16_t d = 100; // delay constant

	uint16_t servoPeriod = 20; //20 cycles

	// start at 1.0 ms pulse - should be 90 ccw
	TIMER_A2_PWM_Init(timerPeriod, 1.0/servoPeriod, 1); 

	while(1)
	{
		//pulse of 1.x ms should xx the servo motor
		TIMER_A2_PWM_DutyCycle(2.0 / servoPeriod, 1);
		delay(d);
		//pulse of 1.x ms should xx the servo motor
		TIMER_A2_PWM_DutyCycle( 1.0/ servoPeriod, 1);
		delay(d);
//		// pulse of 1.x ms should xx the servo motor
//		TIMER_A2_PWM_DutyCycle(1.5 / servoPeriod, 1);
	}

		 return 0; // NOTE intentionally unreachable
}
