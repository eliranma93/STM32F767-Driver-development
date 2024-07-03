/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: 18 Oct 2023
 *      Author: Eliran.Malki
 */


#include "stm32f767xx.h"
#include <string.h>



/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

/*
* SPI pin numbers:
* SCK   13  // Serial Clock.
* MISO  12  // Master In Slave Out.
* MOSI  11  // Master Out Slave In.
* SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
*/


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t	SPI2Handle;

	SPI2Handle.pSPIx					= SPI2;
	SPI2Handle.SPI_Config.SPI_DeviceMode= SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 2MHz
	SPI2Handle.SPI_Config.SPI_DFF		= SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL		= SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA		= SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM		= SPI_SSM_DI;//hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

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

int main(void) {

	//char user_data[]="Hello world";
	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";

	GPIO_Button_Init();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	while(1)
	{
	//wait till button is pressed
	while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

	//to avoid button de-bouncing related issues 200ms of delay
	delay();
	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//first send length information
	uint8_t dataLen =strlen(user_data);
	SPI_SendData(SPI2,&dataLen, 1);

	//to send data
	SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));

	//confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//DIsable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	}
	return 0;
}
