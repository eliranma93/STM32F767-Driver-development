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

//Flag variable
uint8_t rxComplt = RESET;

#define MY_ADDR 0x61

#define SLAVE_ADDR  0x39

I2C_Handle_t I2C1Handle;

//buffer to get the data
uint8_t rcv_buff[4] ;
/*
-   SCL -> SCL
-   SDA -> SDA
-   VDD -> 3.3V
-   GND -> GND
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
	uint8_t CommandCode[2];



	GPIO_Button_Init();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//i2c IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//wait till button is pressed
	while(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
	printf("button pressed\r\n");
	//to avoid button de-bouncing related issues 200ms of delay
	delay();

	CommandCode[0]=0x00 | 0x80;
	CommandCode[1]=0x03;

	//Starts I2C communication, Select control register,Power ON mode
	while(I2C_MasterSendDataIT(&I2C1Handle, CommandCode, 2, SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY){};

	CommandCode[0]=0x01 | 0x80;
	CommandCode[1]=0x02;

	//Starts I2C communication, Select timing register,Stop I2C Transmission
	while(I2C_MasterSendDataIT(&I2C1Handle, CommandCode, 2, SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY){};
	delay();

		while(1)
		{

			for(int i = 0; i < 4; i++)
			  {
				//Starts I2C communication,Select data register
				CommandCode[0]=140 + i;
				while(I2C_MasterSendDataIT(&I2C1Handle, CommandCode, 1, SLAVE_ADDR,I2C_DISABLE_SR) != I2C_READY){};

			    // Request 1 byte of data,Read 1 bytes of data
				while(I2C_MasterReceiveDataIT(&I2C1Handle, &rcv_buff[i], 1, SLAVE_ADDR,I2C_DISABLE_SR)!= I2C_READY);

				while (rxComplt != SET){};

				 rxComplt = RESET;

			  }
			  // Convert the data
			  double ch0 = ((rcv_buff[1] & 0xFF) * 256) + (rcv_buff[0] & 0xFF);
			  double ch1 = ((rcv_buff[3] & 0xFF) * 256) + (rcv_buff[2] & 0xFF);

			  // Output data to serial monitor
			  printf("Full Spectrum(IR + Visible) : %d /n",(uint16_t)ch0);

			  printf("/n Infrared Value : %d /n",(uint16_t)ch1);


			  printf("/n Visible Value : %d /n",(uint16_t)(ch0-ch1));

		}
}

void I2C1_EV_IRQHandler(void)
{

	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{

	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed\n");
     }else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed\n");
    	 rxComplt = SET;
     }else if (AppEv == I2C_EV_NACK)
     {
    	 printf("Error : Nack failure\n");
    	 //in master ack failure happens when slave fails to send ack for the byte
    	 //sent from the master.
    	 I2C_CloseSendData(pI2CHandle);

    	 //Automatically generate the stop condition to release the bus

    	 //Hang in infinite loop
    	 while(1);
     }
}


