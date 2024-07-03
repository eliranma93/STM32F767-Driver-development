/**
 * @file 015_uart_tx.c
 * @author johannes regnier
 * @brief 
 * @version 0.1
 * @date 2022-10-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <string.h>
#include "stm32f767xx.h"
#include <stdio.h>

/*
 * if we want to send the serial data to PC over USB we need to use
 * Pins to communicate over USART3
 * PD8 -->TX
 * PD9 -->RX
 * ALT function mode: 7
 */


/* Pins to communicate over USART2 (Cf. datasheet, alternate function mapping)
 * PD5  ---> USART2_TX (CN9 pin 6 on NUCLEO F767)
 * PD6  ---> USART2_RX (CN9 pin 4 on NUCLEO F767)
 * ALT function mode: 7
 */

USART_Handle_t usart2_handle;

// tx buffer
uint8_t msg[1024] = "USART Tx testing...........\n\r";



void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart2_pins;
    
    usart2_pins.pGPIOx = GPIOD;
    usart2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
    usart2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    usart2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
    usart2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //TX
    usart2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&usart2_pins);

    //RX
    usart2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&usart2_pins);

}

void USART2_Init()
{
    usart2_handle.pUSARTx= USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

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

    printf("Application is running\n");

    // GPIO button init
    GPIO_Button_Init();

    // USART pin inits
    USART2_GPIOInit();

    // USART peripheral configuration
    USART2_Init();

    USART_PeripheralControl(USART2, ENABLE);

    while (1)
    {
        // wait for button press
        while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

        delay();

        // send some data to the slave
        USART_SendData(&usart2_handle, (uint8_t *)msg, strlen((char *)msg));
    }

    return 0;
}


