/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: 25 Oct 2023
 *      Author: Eliran.Malki
 */


#include<stdio.h>
#include<string.h>
#include "stm32f767xx.h"
#include "stdlib.h"
#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68



I2C_Handle_t I2C1Handle;
//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";
uint8_t other_data[]= "We are the validate the test\n";



/*
 * PB8-> SCL
 * PB9 or PB7 -> SDA
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

		GPIO_Button_Init();

		//i2c pin inits
		I2C1_GPIOInits();
		delay();
		//i2c peripheral configuration
		I2C1_Inits();
		delay();

		I2C_PeripheralControl(I2C1, ENABLE);

		while(1)
		{

			//wait till button is pressed
			while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			//send some data
			I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR,I2C_DISABLE_SR);

			//send other data
			I2C_MasterSendData(&I2C1Handle, other_data, strlen((char*)some_data), SLAVE_ADDR,I2C_DISABLE_SR);
		}

}
