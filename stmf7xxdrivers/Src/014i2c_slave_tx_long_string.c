/*
 * 010i2c_master_rx_testing.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Eliran.Malki
 */

#include 	<stdint.h>
#include	<stdio.h>
#include	<string.h>
#include	"stm32f767xx.h"
#include 	"stdlib.h"



#define SLAVE_ADDR  0x68

#define MY_ADDR		SLAVE_ADDR

/*testing */
//#define SLAVE_ADDR  0x67

I2C_Handle_t I2C1Handle;

//buffer to transmit the data
uint8_t Tx_buff[] ="STM32 Slave mode testing.. of a very long string, \r\n My name is Eliran Malki,\r\n I'am doing this coarse to improve my abilities on Embedded,\r\n I am pretty sure that i will take the next coarse of fast bit embedded/r/n" ;

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
	I2C1Handle.Mode=I2C_MODE_SLAVE;
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

		//enable the i2c peripheral interrupts
		I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);



		while(1);
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
	static uint8_t CommandCode=0;
	static uint8_t CntStringChar=0;
	static uint8_t CntStringLen=0;
	static uint32_t LenString=0;

	if (AppEv==I2C_EV_DATA_REQ) {
		//Master wants some data. slave has to send it
		if (CommandCode==0x51) {

			//Send the 4 bytes length information to the master
			LenString=strlen((char*)Tx_buff);
			I2C_SlaveSendData(I2C1,(uint8_t)LenString>>(CntStringLen*8) );
			CntStringLen++;

		} else if(CommandCode==0x52){

			//send the contents of Tx_buff
			I2C_SlaveSendData(I2C1, Tx_buff[CntStringChar++]);
		}
	} else if(AppEv==I2C_EV_DATA_RCV){

		//Data is waiting for the slave to read . slave has to read it
		CommandCode = I2C_SlaveReceiveData(I2C1);

	}else if (AppEv == I2C_EV_NACK) {

        //This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		//if the current active code is 0x52 then dont invalidate
		if(! (CommandCode == 0x52))
			CommandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		CntStringLen = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(CntStringChar == (LenString))
		{
			CntStringChar=0;
			CommandCode = 0xff;
		}

	}else if (AppEv == I2C_EV_STOP) {

		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
		CntStringLen = 0;

	}




}


