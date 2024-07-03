/*
 * 02LedButton.c
 *
 *  Created on: 8 Oct 2023
 *      Author: Eliran.Malki
 */

#include "stm32f767xx.h"
#include <string.h>

#define BTN_PRESSED ENABLE

void delay(void)
{
	for (uint32_t var = 0; var < 500000/2; ++var) ;
}

int main(void)
{

	GPIO_Handle_t GpioLed2,GpioBtn ;

	//reset the value of the registers
	memset(&GpioLed2,0,sizeof(GpioLed2));
	memset(&GpioBtn,0,sizeof(GpioBtn));
	/*************************Led Configure **************************/

	GpioLed2.pGPIOx = GPIOB;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioLed2);
	/*************************Button Configure ************************/

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI13);

	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);//clear the pending event from exti line
	GPIO_ToggleputPin(GPIOB, GPIO_PIN_NO_7);
}
