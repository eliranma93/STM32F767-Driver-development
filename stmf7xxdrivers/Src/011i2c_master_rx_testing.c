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
#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68



I2C_Handle_t I2C1Handle;

//buffer to get the data
uint8_t rcv_buff[32] ;
/*
 * PB8-> SCL
 * PB9  -> SDA
 */



void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config.I2C_DeviceAddress=MY_ADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM100K;

	I2C_Init(&I2C1Handle);
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
	printf("Application is running\r\n");
		uint8_t CommandCode;
		uint8_t len;


		GPIO_Button_Init();

		//i2c pin inits
		I2C1_GPIOInits();
		delay();
		//i2c peripheral configuration
		I2C1_Inits();
		delay();

		//enable the i2c peripheral
		I2C_PeripheralControl(I2C1,ENABLE);

		while(1)
		{
			//wait till button is pressed
			while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
			printf("button pressed\r\n");
			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			CommandCode=0x51;
			//send command to get the length of the data to read
			I2C_MasterSendData(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR,I2C_ENABLE_SR);

			//receive the length
			I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR);
			printf("data len: %d\n", len);
			CommandCode=0x52;
			//send command to get the data
			I2C_MasterSendData(&I2C1Handle, &CommandCode, 1, SLAVE_ADDR,I2C_ENABLE_SR);

			//receive the data
			I2C_MasterReceiveData(&I2C1Handle, rcv_buff, len, SLAVE_ADDR,I2C_DISABLE_SR);
			rcv_buff[len+1]='/0';
			printf("buffer: %s", rcv_buff);
		}

}
