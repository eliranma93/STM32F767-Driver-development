/*
 * stmf767xx_spi_driver.c
 *
 *  Created on: 11 Oct 2023
 *      Author: Eliran.Malki
 */

# include "stm32f767xx.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn 					-SPI_PeriClockControl
 *
 * @brief				-enable clock to spi register peripheral
 *
 * @param1   			-pointer to spi register
 * @param2				-ENABLE or DISABLE
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
		if (pSPIx==SPI1) {
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2) {
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3) {
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4) {
			SPI4_PCLK_EN();
		}
		else if(pSPIx==SPI5) {
			SPI5_PCLK_EN();
		}
		else if(pSPIx==SPI6) {
			SPI6_PCLK_EN();
		}
	} else {
		if (pSPIx==SPI1) {
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2) {
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3) {
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4) {
			SPI4_PCLK_EN();
		}
		else if(pSPIx==SPI5) {
			SPI5_PCLK_EN();
		}
		else if(pSPIx==SPI6) {
			SPI6_PCLK_EN();
		}
	}



}


/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn 					-SPI_Init
 *
 * @brief				-innate the peripheral throw the values given in handle structure
 *
 * @param1   			-handle structure for spi peripheral
 * @param2				-
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//configure the SPI_CR1 register

	uint32_t tempreg =0;
	//1.configure the device mode
	tempreg|=pSPIHandle->SPI_Config.SPI_DeviceMode<<SPI_CR1_MSTR;

	//2.configure the bus config
	if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_FD) {

		//bidi mode should be cleared

		tempreg&=~(1<<SPI_CR1_BIDI_MODE);

	} else if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempreg|=(1<<SPI_CR1_BIDI_MODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//bidi mode should be cleared
		//RXONLY must be set
		tempreg&=~(1<<SPI_CR1_BIDI_MODE);
		tempreg|=(1<<SPI_CR1_RX_ONLY);
	}
	//3.configure the clock speed(baud rate)
	tempreg|=pSPIHandle->SPI_Config.SPI_SclkSpeed<<SPI_CR1_BR ;

	//4.configure the clock polarity
	tempreg|=pSPIHandle->SPI_Config.SPI_CPOL<<SPI_CR1_CPOL;

	//5.configure the clock phase
	tempreg|=pSPIHandle->SPI_Config.SPI_CPHA<<SPI_CR1_CPHA;

	//6.configure the slave software management
	tempreg|=pSPIHandle->SPI_Config.SPI_SSM<<SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1=tempreg;

	//configure the SPI_CR2 register just for the data size
	pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS ); //clearing 4 bits
	pSPIHandle->pSPIx->CR2|=pSPIHandle->SPI_Config.SPI_DFF<<SPI_CR2_DS;

}

/*****************************************************************
 * @fn 					-SPI_DeInit
 *
 * @brief				-reset the spi peripheral
 *
 * @param1   			-pointer to spi register peripheral
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	if (pSPIx==SPI1) {
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2) {
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3) {
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4) {
		SPI4_REG_RESET();
	}
	else if(pSPIx==SPI5) {
		SPI5_REG_RESET();
	}
	else if(pSPIx==SPI6) {
		SPI6_REG_RESET();
	}
}


/*
 * Data send and receive
 */

/*****************************************************************
 * @fn 					-SPI_GetFlagStatus
 *
 * @brief				-checks is the flag of TxBuffer or RxBuffer is empty
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-macro of flag identification
 *
 * @return				-  none
 *
 * @note				-  '1' or zero if the flag is set
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if (pSPIx->SR&FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*****************************************************************
 * @fn 					-SPI_SendData
 *
 * @brief				-performing sending data
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-pointer to buffer to hold the data we got
 * @param3				-length of the data to send
 *
 * @return				-  none
 *
 * @note				-  This is blocking call
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len>0)
	{

		//1. wait until TXE is set(empty when the condition is false)
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);


		//2. check the DFF bit in CR2
		if ((pSPIx->CR2 & (SPI_DFF_16BITS<<SPI_CR2_DS))==(SPI_DFF_16BITS<<SPI_CR2_DS)) {
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR=*((uint16_t*)pTxBuffer);
			//2. decrease the Len value
			Len--;
			Len--;
			//3. increase the address of pointer for Tx value
			(uint16_t*)pTxBuffer++;
		}else if (pSPIx->CR2 & (SPI_DFF_8BITS<<SPI_CR2_DS)) {
			//8 bit DFF
			*((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer; // cf. data packing RM section 35.5.9
            //pSPIx->DR = *pTxBuffer; // just dereferencing the pointer (pointer if of type uint8_t)
			Len--;
			pTxBuffer++;
		}

        // clear OVR flag
        uint32_t tempreg = 0x00;
        tempreg = pSPIx->DR;    // read operation on DR register
        tempreg = pSPIx->SR;    // read operation on SR register
        (void)tempreg;          //typecast to void to avoid gcc/g++ warnings

	}

}

/*****************************************************************
 * @fn 					-SPI_ReceiveData
 *
 * @brief				-the function is used to receive data from another device
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-pointer to data to send
 * @param3				-length of the data to receive
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	//1. wait until RXNE is set(non empty when the condition is false)
	while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)==FLAG_RESET);

	//2. check the DFF bit in CR2
	if ((pSPIx->CR2 & (SPI_DFF_16BITS<<SPI_CR2_DS))==(SPI_DFF_16BITS<<SPI_CR2_DS)) {
		//16 bit DFF
		//1. load from the data DR to the RxBuffer addresses
		*((uint16_t*)pRxBuffer)=pSPIx->DR;
		//2. decrease the Len value
		Len--;
		Len--;
		//3. increase the address of pointer for Rxbuffer
		(uint16_t*)pRxBuffer++;
	}else if ((pSPIx->CR2 & (SPI_DFF_8BITS<<SPI_CR2_DS))==(SPI_DFF_8BITS<<SPI_CR2_DS)) {
		//8 bit DFF
		*(pRxBuffer)=*((volatile uint8_t *)&pSPIx->DR);
		Len--;
		pRxBuffer++;
	}

}






/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn 					-SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
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
 * @fn 					-  SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	//not all the bits applicable at this registers we use the first 4 bits
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	//setting the priority of processor register by dividing the irq number we get the first address for each register

	*(NVIC_PR_BASE_ADDR+(iprx))|= (IRQPriority<<(shift_amount));

}




/*
 * Other Peripheral Control APIs
 */

/*****************************************************************
 * @fn 					-SPI_PeripheralControl
 *
 * @brief				-to enable the spi peripheral
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-enable or disable
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{

	if (EnorDi==ENABLE) {
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	} else {

		pSPIx->CR1 &=~ (1<<SPI_CR1_SPE);
	}
}

/*****************************************************************
 * @fn 					-SPI_SSIConfig
 *
 * @brief				-to enable the ssm as 1 for management pin nss software
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-enable or disable
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
			pSPIx->CR1 |= (1<<SPI_CR1_SSI);
		} else {
			pSPIx->CR1 &=~ (1<<SPI_CR1_SSI);
}
}

/*****************************************************************
 * @fn 					-SPI_SSOEConfig
 *
 * @brief				-to enable the nss pin as mater mode to cancel the option of multimaster
configuration
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-enable or disable
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
			pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
		} else {
			pSPIx->CR2 &=~ (1<<SPI_CR2_SSOE);
}
}

/*********************************************************************
 * @fn      		  - SPI_FRXTHConfig
 *
 * @brief             - RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
 *
 * @param[1]         -pointer to spi register peripheral
 * @param[2]         - enable or disable

 *
 */
void SPI_FRXTHConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
        if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_FRXTH); // set FRXTH (bit #12)
    }else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH); // reset bit
    }
}


/*****************************************************************
 * @fn 					-SPI_SendDataIT
 *
 * @brief				-the function is saves the pointers, the length information , everything and just enable the TXEIE interrupt
 *
 * @param1   			-pointer to spi register peripheral
 * @param2				-pointer to buffer to hold the data we got
 * @param3				-length of the data to send
 *
 * @return				-  none
 *
 * @note				- this is non blocking API
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;



}
/*****************************************************************
 * @fn 					-SPI_ReceiveDataIT
 *
 * @brief				-the function is used to receive data from another device when get an interrupt from SPI peripheral
 *
 * @param1   			-handle structure for spi peripheral
 * @param2				-pointer to data to send
 * @param3				-length of the data to receive
 *
 * @return				-  none
 *
 * @note				-  this is non blocking API
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;


}

/*****************************************************************
 * @fn 					-SPI_IRQHandling
 *
 * @brief				-This function finds out from which state an interrupt came to the peripheral spi and performs the required function accordingly
 *
 * @param1   			-handle structure for spi peripheral
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHnadle)
{
	uint8_t temp1 , temp2;
		//first lets check for TXE
		temp1 = pHnadle->pSPIx->SR & ( 1 << SPI_SR_TXE);
		temp2 = pHnadle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

		if( temp1 && temp2)
		{
			//handle TXE
			spi_txe_interrupt_handle(pHnadle);
		}

		// check for RXNE
		temp1 = pHnadle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
		temp2 = pHnadle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

		if( temp1 && temp2)
		{
			//handle RXNE
			spi_rxne_interrupt_handle(pHnadle);
		}

		// check for ovr flag
		temp1 = pHnadle->pSPIx->SR & ( 1 << SPI_SR_OVR);
		temp2 = pHnadle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

		if( temp1 && temp2)
		{
			//handle ovr error
			spi_ovr_err_interrupt_handle(pHnadle);
		}



}
/*******************some helper functions*************************************/

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ((pSPIHandle->pSPIx->CR2 & (SPI_DFF_16BITS<<SPI_CR2_DS))==(SPI_DFF_16BITS<<SPI_CR2_DS)) {
			//16 bit DFF
			//1. load the data in to the DR
			pSPIHandle->pSPIx->DR=*((uint16_t*)pSPIHandle->pTxBuffer);
			//2. decrease the Len value
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			//3. increase the address of pointer for Tx value
			(uint16_t*)pSPIHandle->pTxBuffer++;
	}else if (pSPIHandle->pSPIx->CR2 & (SPI_DFF_8BITS<<SPI_CR2_DS)) {
				//8 bit DFF
			pSPIHandle->pSPIx->DR= *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ((0xF & (pSPIHandle->pSPIx->CR2 >> 8))==SPI_DFF_16BITS) {
		//16 bit DFF
		//1. load from the data DR to the RxBuffer addresses
		*((uint16_t*)pSPIHandle->pRxBuffer)=pSPIHandle->pSPIx->DR;
		//2. decrease the Len value
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		//3. increase the address of pointer for Rxbuffer
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else {
		//8 bit DFF
		*(pSPIHandle->pRxBuffer)=*((volatile uint8_t *)pSPIHandle->pSPIx->DR);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//RxLen is zero , so close the spi transmission and inform the application that
		//RX is over.

		//this prevents interrupts from setting up of RXNE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//To ignore errors we will use type cast because we have no use of a variable
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
