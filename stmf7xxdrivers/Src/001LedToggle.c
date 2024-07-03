/*
 * 001LedToggle.c
 *
 *  Created on: 8 Oct 2023
 *      Author: Eliran.Malki
 */


#include "stm32f767xx.h"

void delay(void)
{
	for (uint32_t var = 0; var < 25000; ++var) ;
}

int main(void)
{
	GPIO_Handle_t GpioLed2;
	GpioLed2.pGPIOx = GPIOB;
	GpioLed2.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GpioLed2.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed2.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioLed2);

	while(1)
	{
		GPIO_ToggleputPin(GPIOB,GPIO_PIN_NO_7 );
		delay();
	}

	return 0;
}
