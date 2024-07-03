/*
 * stm32f767xx_i2c_driver.c
 *
 *  Created on: 22 Oct 2023
 *      Author: Eliran.Malki
 */


#include "stm32f767xx.h"

/*********************************************************************************************
 *   Hard-coded I2C_TIMINGR timing settings depending on the I2C clock
 *      See tables in ref manual, section 33.4.9
 *     | SM_10kHz | SM_100kHz | FM_400kHz | FM+_1000kHz |
 *  source: https://github.com/MayaPosch/Nodate/blob/master/arch/stm32/cpp/core/src/i2c.cpp
 * Note: STM32CubeMX calculates and provides the I2C_TIMINGR content in the I2C Config. window.
 * *******************************************************************************************/
uint32_t i2c_timings_4[4]  = {0x004091F3, 0x00400D10, 0x00100002, 0x00000001};
uint32_t i2c_timings_8[4]  = {0x1042C3C7, 0x10420F13, 0x00310309, 0x00100306};
uint32_t i2c_timings_16[4] = {0x3042C3C7, 0x303D5B, 0x10320309, 0x00200204}; // SM_100k timing value from STM32CubeMX, for avoiding glitch (it works)
uint32_t i2c_timings_48[4] = {0xB042C3C7, 0xB0420F13, 0x50330309, 0x50100103};
uint32_t i2c_timings_54[4] = {0xD0417BFF, 0x40D32A31, 0x10A60D20, 0x00900916};




static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint16_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint16_t SlaveAddr);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);



/* @fn 					-I2C_ExecuteAddressPhaseWrite
*
* @brief				-helper function to generate the slave address and write operation
*
* @param1   			-pointer to i2c peripheral register address
* @param2   			-slave address
*
* @return				-  none
*
* @note				-  none
*
*/

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint16_t SlaveAddr)
{
	pI2Cx->CR2 |= SlaveAddr << 1;
	pI2Cx->CR2 &= ~(1<<I2C_CR2_RD_WRN); //SlaveAddr is r/nw bit=0 (write mode)
}

/* @fn 					-I2C_ExecuteAddressPhaseRead
*
* @brief				-helper function to generate the slave address and read operation
*
* @param1   			-pointer to i2c peripheral register address
* @param2   			-slave address
*
* @return				-  none
*
* @note				-  none
*
*/

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint16_t SlaveAddr)
{
	pI2Cx->CR2 |= SlaveAddr << 1; // Slave address bit 7:1 (master mode)
	pI2Cx->CR2 |=(1<<I2C_CR2_RD_WRN); // RD_WRN: read transfer direction (master mode)

}

/*****************************************************************
 * @fn 					-I2C_GenerateStartCondition
 *
 * @brief				-helper function to generate the start condition
 *
 * @param1   			-pointer to i2c peripheral register address
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= ( 1 << I2C_CR2_START);
}

/*****************************************************************
 * @fn 					-I2C_GenerateStopCondition
 *
 * @brief				-helper function to generate the stop condition
 *
 * @param1   			-pointer to i2c peripheral register address
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */

void	I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= ( 1 << I2C_CR2_STOP);
}

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn 					-I2C_Init
 *
 * @brief				-enable clock to i2c peripheral
 *
 * @param1   			-pointer to i2c peripheral register address
 * @param2				-ENABLE or DISABLE
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
		if (pI2Cx==I2C1) {
			I2C1_PCLK_EN();
		}
		else if(pI2Cx==I2C2) {
			I2C2_PCLK_EN();
		}
		else if(pI2Cx==I2C3) {
			I2C3_PCLK_EN();
		}
		else if(pI2Cx==I2C4) {
			I2C4_PCLK_EN();
		}
	} else {
		if (pI2Cx==I2C1) {
			I2C1_PCLK_EN();
		}
		else if(pI2Cx==I2C2) {
			I2C2_PCLK_EN();
		}
		else if(pI2Cx==I2C3) {
			SPI3_PCLK_EN();
		}
		else if(pI2Cx==I2C4) {
			I2C4_PCLK_EN();
		}
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn 					-I2C_Init
 *
 * @brief				-innate the peripheral throw the values given in handle structure
 *
 * @param1   			-handle structure for i2c peripheral
 * @param2				-
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;
   //program the device own address

	 //enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//reset the i2c peripheral
	I2C_PeripheralControl(pI2CHandle->pI2Cx,DISABLE);

	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	if (pI2CHandle->Mode==I2C_MODE_SLAVE) {
		tempreg |=(1<<I2C_MODE_SLAVE_ENABLE);
	}
	//tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

    // fill I2C_TIMINGR register using hard-coded timings values
    uint8_t mode = 0;
    mode = pI2CHandle->I2C_Config.I2C_SCLSpeed; // mode: SM10k, SM100k, FM or FMPLUS
    if (RCC_GetPCLK1Value() == 8000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_8[mode];
    }else if (RCC_GetPCLK1Value() == 16000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_16[mode];
    }else if (RCC_GetPCLK1Value() == 48000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_48[mode];
    }else if (RCC_GetPCLK1Value() == 54000000)
    {
        pI2CHandle->pI2Cx->TIMINGR = i2c_timings_54[mode];
    }
}




/*****************************************************************
 * @fn 					-I2C_DeInit
 *
 * @brief				-reset the i2c peripheral
 *
 * @param1   			-pointer to i2c register peripheral
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if (pI2Cx==I2C1) {
		I2C1_REG_RESET();
		}
		else if(pI2Cx==I2C2) {
			I2C2_REG_RESET();
		}
		else if(pI2Cx==I2C3) {
			I2C3_REG_RESET();
		}
		else if(pI2Cx==I2C4) {
			I2C4_REG_RESET();
		}
}

/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn 					-I2C_IRQInterruptConfig
 *
 * @brief				-configure the interrupt at the processor side
 *
 * @param1   			-IRQNumber same in the NVIC
 *  @param2				- ENABLE or DISABLE
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
			if (IRQNumber<=31) {
				//program ISER0 register
				*NVIC_ISER0 |=(1<<IRQNumber);

			} else if (IRQNumber > 31 &&IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				*NVIC_ISER1 |=(1<<IRQNumber%32);

			}else if(IRQNumber >= 64 &&IRQNumber < 96 ) //64 to 95
			{
				//program ISER2 register
				*NVIC_ISER2 |=(1<<IRQNumber%64);
			}else if(IRQNumber >= 96 &&IRQNumber < 128 ) //96 to 127
			{
				//program ISER3 register
				*NVIC_ISER3 |=(1<<IRQNumber%96);
			}

		} else {
			if (IRQNumber<=31) {
				//program ICER0 register
				*NVIC_ICER0 |=(1<<IRQNumber);
			} else if (IRQNumber > 31 &&IRQNumber < 64 ) {
				//program ICER1 register
				*NVIC_ICER1 |=(1<<IRQNumber%32);
			}else if(IRQNumber >= 64 &&IRQNumber < 96 ) {
				//program ICER2 register
				*NVIC_ICER2 |=(1<<IRQNumber%64);
			}else if(IRQNumber >= 96 &&IRQNumber < 128 )
			{
				//program ICER3 register
				*NVIC_ICER3 |=(1<<IRQNumber%96);
			}
		}
}



/*****************************************************************
 * @fn 					-  I2C_IRQPriorityConfig
 *
 * @brief				-
 *
 * @param1   			-IRQNumber same in the NVIC
 * @param2   			-priority of interrupt
 *
 * @return				-
 *
 * @note				-
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	//not all the bits applicable at this registers we use the first 4 bits
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	//setting the priority of processor register by dividing the irq number we get the first address for each register

	*(NVIC_PR_BASE_ADDR+(iprx))|= (IRQPriority<<(shift_amount));
}

/*****************************************************************
 * @fn 					-I2C_MasterSendData
 *
 * @brief				-performing sending data
 *
 * @param1   			-pointer to i2c handler peripheral
 * @param2				-pointer to buffer to the data we want to send
 * @param3				-length of the data to send
 * @param4				-salve address
 * @param5				-
 *
 * @return				-  none
 *
 * @note				-  This is blocking call
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

    // Set autoend to 0, and length of data to be transmitted
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
    pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be transmitted

	//configure 7-bit addressing
	I2C1->CR2&=~(1 <<I2C_CR2_ADD10);

	// Generates the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


    // Send the data until Len becomes 0
    while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->TXDR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

    // wait until TC flag is set (when NBYTES data are transferred)
    while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC)));

    // Generate STOP condition
    if(Sr==I2C_DISABLE_SR){
    	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }


    // clear STOPF flag in I2C_ICR and clear the I2C_CR2 register:
    pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
    pI2CHandle->pI2Cx->CR2 = 0x0;

}

/*****************************************************************
 * @fn 					-I2C_MasterReceiveData
 *
 * @brief				-performing sending data
 *
 * @param1   			-pointer to i2c handler peripheral
 * @param2				-pointer to buffer to the data we want to read
 * @param3				-length of the data to send
 * @param4				-salve address
 * @param5				-
 *
 * @return				-  none
 *
 * @note				-  This is blocking call
 *
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
		//Set the address of the slave with r/nw bit set to r(1) (total 8 bits )
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

		//configure the length
		pI2CHandle->pI2Cx->CR2|=(Len<<I2C_CR2_NBYTES);

		//configure the stop condition as software
		pI2CHandle->pI2Cx->CR2&=~(1<<I2C_CR2_AUTOEND);

		//Set the address of the slave with r/nw bit set to r(1) (total 8 bits )
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

		// Generates the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		if (Len==1)
		{
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) ); //Wait till RXNE is set
			*pRxBuffer = pI2CHandle->pI2Cx->RXDR; // read data into buffer
		}
		else {
		    // Receive the data until Len becomes 0
		    while(Len > 0)
			{
				while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) ); //Wait till RXNE is set
				*pRxBuffer = pI2CHandle->pI2Cx->RXDR; // read data into buffer
				pRxBuffer++;
				Len--;
			}
		}



	    // wait until TC flag is set (when NBYTES data are transferred)
	    while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TC))){};

	    // Generate STOP condition
	    if(Sr==I2C_DISABLE_SR){
	    	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	    }


	    // clear STOPF flag in I2C_ICR and clear the I2C_CR2 register:
	    pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
	    pI2CHandle->pI2Cx->CR2 = 0x0;

}



/*****************************************************************
 * @fn 					-I2C_MasterSendDataIT
 *
 * @brief				-performing sending data
 *
 * @param1   			-pointer to i2c handler peripheral
 * @param2				-pointer to buffer to the data we want to send
 * @param3				-length of the data to send
 * @param4				-salve address
 * @param5				-start repeated
 * @return				-  none
 * @note				-  This is non blocking call
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Mode = I2C_MODE_MASTER;
        pI2CHandle->Sr = Sr;

        // Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

        // Set autoend to 0, and length of data to be transmitted
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
        pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->TxLen << I2C_CR2_NBYTES);  //number of bytes to be transmitted

        //Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        //enable TXIE, TCIE, STOPIE,NACKIE Control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);
        //pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);//master is generate the stop condition not read it
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_NACKIE);

        //enable ERRIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);
	}

	return busystate;
}


/*****************************************************************
 * @fn 					-I2C_MasterReceiveDataIT
 *
 * @brief				-performing receiving data
 *
 * @param1   			-pointer to i2c handler peripheral
 * @param2				-pointer to buffer to the data we want to receive
 * @param3				-length of the data to send
 * @param4				-salve address
 * @param5				-start repeated
 * @return				-  none
 * @note				-  This is non blocking call
 */

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->Mode=I2C_MODE_MASTER;

        // Send the address of the slave with r/nw bit set to r(1) (total 8 bits )
	    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

        // Set autoend to 0, and length of data to be transmitted
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
        pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be transmitted

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


        //enable TXIE, TCIE, STOPIE ,NACKIE Control Bits
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE);
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_TCIE);
        //pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);//master is generate the stop condition not read it
        pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_NACKIE);

        //enable ERRIE Control Bit
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);


	}

	return busystate;
}


/*****************************************************************
 * @fn 					-I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief				-performing enabling or disabling interrupt as slave
 *
 * @param1   			-pointer to i2c register peripheral
 * @param2				-enable or disable

 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
        //Enable TXIE, STOPIE ,NACKIE Control Bits
		pI2Cx->CR1 |= ( 1 << I2C_CR1_TXIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_STOPIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_NACKIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ADDRIE);
        pI2Cx->CR1 |= ( 1 << I2C_CR1_RXIE);
        //Enable ERRIE Control Bit
//		pI2Cx->CR1 |= ( 1 << I2C_CR1_ERRIE);

	} else {
        //Disable TXIE, TCIE, STOPIE ,NACKIE Control Bits
		pI2Cx->CR1 &=~ ( 1 << I2C_CR1_TXIE);
        pI2Cx->CR1 &=~ ( 1 << I2C_CR1_STOPIE);
        pI2Cx->CR1 &=~ ( 1 << I2C_CR1_NACKIE);
        pI2Cx->CR1 &=~ ( 1 << I2C_CR1_ADDRIE);
        pI2Cx->CR1 &=~ ( 1 << I2C_CR1_RXIE);
        //Disable ERRIE Control Bit
//		pI2Cx->CR1 &=~ ( 1 << I2C_CR1_ERRIE);
	}






}
/*
 * Other Peripheral Control APIs
 */

/*****************************************************************
 * @fn 					-I2C_PeripheralControl
 *
 * @brief				-to enable the i2c peripheral
 *
 * @param1   			-pointer to i2c register peripheral
 * @param2				-enable or disable
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	} else {

		pI2Cx->CR1 &=~ (1<<I2C_CR1_PE);
	}

}

/*****************************************************************
 * @fn 					-I2C_GetFlagStatus
 *
 * @brief				-checks is the flag is set or not
 *
 * @param1   			-pointer to i2c register peripheral
 * @param2				-macro of flag identification
 *
 * @return				-  none
 * @note				-  '1' or zero if the flag is set
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
	if (pI2Cx->ISR & FlagName) {
		return SET;

	} else {
		return RESET;
	}

}

/*****************************************************************
 * @fn 					-I2C_CloseSendData
 *
 * @brief				-helper function to disable relevant registers of sending data
 *
 * @param1   			-pointer to i2c register peripheral
 *
 * @return				-  none
 * @note				-
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    //disable TXIE, STOPIE, TCIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;

    //pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
}

/*****************************************************************
 * @fn 					-I2C_CloseReceiveData
 *
 * @brief				-helper function to disable relevant registers of receiving data
 *
 * @param1   			-pointer to i2c handler peripheral
 * @return				-  none
 * @note				-
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    //disable RXIE, STOPIE, TCIE, NACKIE Control Bits
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_RXIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_STOPIE);
    pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_TCIE);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

    //pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
}



/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief				-function to send data as slave mode
 *
 * @param1   			-pointer to i2c register peripheral
 * @param   			-data to send
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->TXDR=data;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief				-function to receive data as slave mode
 *
 * @param1   			-pointer to i2c register peripheral
 */

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return	(uint8_t)pI2Cx->RXDR;
}

/*****************************************************************
 * @fn 					-I2C_EV_IRQHandling
 *
 * @brief				-helper function to handle the interrupt event
 *
 * @param1   			-pointer to i2c handler peripheral
 * @return				-  none
 * @note				-
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    // Handle for interrupt generated by ADDR event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_ADDRIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_ADDR);
    if (temp1 && temp2)
    {
        // clear flag by setting the ADDRCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_ADDRCF);
        /* Enable Address Acknowledge */
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_NACK);
    }

    // Handle for interrupt generated by received NACK event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_NACKIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_NACKF);
    if (temp1 && temp2)
    {
        // clear flag by setting the NACKCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_NACKCF);

        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_NACK);
    }


    // Handle for interrupt generated by TC event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TCIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TC);
    if (temp1 && temp2)
    {
            //1. generate the STOP condition
            if(pI2CHandle->Sr == I2C_DISABLE_SR)
            {
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }
            // 2. reset all the member elements of the handle structure.
            I2C_CloseSendData(pI2CHandle);
            I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
            pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register
    }

    // Handle for interrupt generated by STOPF event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_STOPIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_STOPF);
    if (temp1 && temp2)
    {
        // STOPF flag is set
        // - set by hardware when a Stop condition is detected.
        // - cleared by software by setting the STOPCF bit:
        pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);

        //Notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // Handle for interrupt generated by TXIS event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_TXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_TXIS);
    if (temp1 && temp2)
    {
        // TXIS flag is set

        //check for device mode
        if (pI2CHandle->Mode == I2C_MODE_MASTER)
        {
            // master: We have to do the data transmission
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                if (pI2CHandle->TxLen > 0)
                {
                    // 1. load the data in TXDR
                    pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
                    // 2. decrement TxLen
                    pI2CHandle->TxLen--;
                    // 3. increment buffer address
                    pI2CHandle->pTxBuffer++;

                }

            }
        }else
        {
            //slave
            // check that slave is in transmitter mode (DIR bit set in ISR register)
            if (pI2CHandle->pI2Cx->ISR & (1<<I2C_ISR_DIR))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }

        }


    }

    // Handle for interrupt generated by RXNE event
    temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR1_RXIE);
    temp2 = pI2CHandle->pI2Cx->ISR & (1 << I2C_ISR_RXNE);
    if (temp1 && temp2)
    {
        // RXNE flag is set

        //check for device mode
        if (pI2CHandle->Mode == I2C_MODE_MASTER)
        {
            // master mode
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                if (pI2CHandle->RxLen > 0)
                {
                    // 1. read RXDR data into buffer
                    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
                    // 2. decrement RxLen
                    pI2CHandle->RxLen--;
                    // 3. increment buffer address
                    pI2CHandle->pRxBuffer++;

                }


                if(pI2CHandle->RxLen == 0)
                {
                    //1. generate the stop condition
                    if(pI2CHandle->Sr == I2C_DISABLE_SR){
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    //2 . Close the I2C rx
                    I2C_CloseReceiveData(pI2CHandle);


                    //3. Notify the application
                    I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);

                    pI2CHandle->pI2Cx->CR2 = 0x0; // clear CR2 register


                }

            }
        }else
        {
            // slave mode
            // check that slave is in receiver mode (DIR bit reset in ISR register)
            if (!(pI2CHandle->pI2Cx->ISR & (1<<I2C_ISR_DIR)))
            {

                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);


            }
        }
    }

}





/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief				-helper function to handle the interrupt error
 *
 * @param1   			-pointer to i2c handler peripheral
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ERRIE control bit in the CR1
	temp2 = (pI2CHandle->pI2Cx->CR1) & ( 1 << I2C_CR1_ERRIE);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1<< I2C_ISR_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_BERRCF);

		//Implement the code to notify the application about the error
	   //I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->ICR) & ( 1 << I2C_ISR_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_ARLOCF);
		//Implement the code to notify the application about the error
		//I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1 << I2C_ISR_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_OVRCF);

		//Implement the code to notify the application about the error
		//I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->ISR) & ( 1 << I2C_ISR_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->ICR |= ( 1 << I2C_ICR_TIMEOUTCF);
		//Implement the code to notify the application about the error
		//I2C_ApplicationEventCallback(pI2CHandle,I2C_TIMEOUT_OVR);
	}

}


