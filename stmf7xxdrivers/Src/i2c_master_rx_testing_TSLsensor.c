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
#include "TSL2561.h"

#define MY_ADDR 0x61

#define SLAVE_ADDR  0x39



I2C_Handle_t I2C1Handle;

//buffer to get the data
uint8_t rcv_buff[4] ;
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
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
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
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_FM;

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
		uint8_t CommandCode[2];
		uint8_t data[4];
		uint16_t ch0,ch1;
		float LuxValue=0;
		*CPACR |= ((3UL << 10*2)|(3UL << 11*2));
		GPIO_Button_Init();

		//i2c pin inits
		I2C1_GPIOInits();
		delay();
		//i2c peripheral configuration
		I2C1_Inits();
		delay();

		//enable the i2c peripheral
		I2C_PeripheralControl(I2C1,ENABLE);

		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		printf("button pressed\r\n");
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		while(1)
		{
			CommandCode[0]=(0x00 | 0x80);
			CommandCode[1]=0x03;

			//send command to Starts I2C communication,Select control register to Power ON mode
			I2C_MasterSendData(&I2C1Handle, CommandCode, 2, SLAVE_ADDR,I2C_DISABLE_SR);

			CommandCode[0]=(0x01 | 0x80);
			CommandCode[1]=0x02;

			//send command to Select timing register ,Nominal integration time = 402ms
			I2C_MasterSendData(&I2C1Handle, CommandCode, 2, SLAVE_ADDR,I2C_DISABLE_SR);

			delay();
			delay();
			delay();
			delay();

			for(int i = 0; i < 4; i++)
			  {
			    CommandCode[0]=(140 + i);
			    //send command to Select data register
			    I2C_MasterSendData(&I2C1Handle, &CommandCode[0], 1, SLAVE_ADDR,I2C_DISABLE_SR);

			    // Read 1 bytes of data
			    I2C_MasterReceiveData(&I2C1Handle, &data[i], 1, SLAVE_ADDR, I2C_DISABLE_SR);
			  }
			  // Convert the data
			  ch0 = (uint16_t)(data[1] * 256 + data[0]);
			  ch1 = (uint16_t)(data[3]  * 256) + data[2];

			  // Output data to serial monitor
			  printf("Full Spectrum(IR + Visible) : %u \r\n",ch0);
			  printf("Infrared Value : %u \r\n",ch1);
			  printf("Visible Value : %u \r\n",(ch0-ch1));

			  LuxValue = (ch1/ ch0);

			  TSL2561_GetLux(ch0,ch1,&LuxValue);
			  printf("After calculation : %f \r\n",LuxValue);



		}

}
