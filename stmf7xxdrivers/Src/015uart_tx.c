/*
 * 010i2c_master_rx_testing.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Eliran.Malki
 */

#include <stdint.h>
#include<stdio.h>
#include<string.h>
#include "stm32f767xx.h"
#include "stdlib.h"

USART_Handle_t	usart2_handle;

//buffer to transmit the data
char msg[1024] ="UART Tx testing..\n\r" ;

/*
 * PB8-> SCL
 * PB9  -> SDA
 */


void USART2_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOD;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	USARTPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//RX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	GPIO_Init(&USARTPins);

	//TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIO_Init(&USARTPins);

}

void USART2_Inits(void)
{
	usart2_handle.pUSARTx=USART2;
	usart2_handle.USART_Config.USART_Baud=USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_NoOfStopBits=1;
	usart2_handle.USART_Config.USART_HWFlowControl=USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode=USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits=USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_ParityControl=USART_PARITY_DISABLE;
	usart2_handle.USART_Config.USART_WordLength=USART_WORDLEN_8BITS;
	USART_Init(&usart2_handle);
}

void GPIO_Button_Init(void)
	{
		GPIO_Handle_t GpioBtn ;
		GpioBtn.pGPIOx = GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_INP;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		GPIO_Init(&GpioBtn);
	}


int main(void)
{
	USART2_GPIOInits();

	USART2_Inits();

	USART_PeripheralControl(usart2_handle.pUSARTx, ENABLE);

	GPIO_Button_Init();

	USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));

	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		printf("button pressed\r\n");
		//to avoid button de-bouncing related issues 200ms of delay

		delay();


	}
}


