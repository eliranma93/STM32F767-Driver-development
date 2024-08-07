/*
 * stm32f767xx_usart_driver.c
 *
 *  Created on: 29 Oct 2023
 *      Author: Eliran.Malki
 */

#include "stm32f767xx.h"


/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn 					-  USART_PeriClockControl
 *
 * @brief				-  This function enables or disables peripheral clock for the given USART port
 *
 * @param1   			-   base address
 * @param2				-	ENABLE or DISABLE macros
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi==ENABLE) {
			if (pUSARTx==USART1) {
				USART1_PCLK_EN();
			}
			else if(pUSARTx==USART2) {
				USART2_PCLK_EN();
			}
			else if(pUSARTx==USART3) {
				USART3_PCLK_EN();
			}
			else if(pUSARTx==UART4) {
				UART4_PCLK_EN();
			}
			else if(pUSARTx==UART5) {
				UART5_PCLK_EN();
			}
			else if(pUSARTx==USART6) {
				USART6_PCLK_EN();
			}
		}
	else {
		if (pUSARTx==USART1) {
			USART1_PCLK_DI();
		}
		else if(pUSARTx==USART2) {
			USART2_PCLK_DI();
		}
		else if(pUSARTx==USART3) {
			USART3_PCLK_DI();
		}
		else if(pUSARTx==UART4) {
			UART4_PCLK_DI();
		}
		else if(pUSARTx==UART5) {
			UART5_PCLK_DI();
		}
		else if(pUSARTx==USART6) {
			USART6_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*****************************************************************
 * @fn 					-USART_Init
 *
 * @brief				-innate the peripheral throw the values given in handle structure
 *
 * @param1   			-handle structure for usart peripheral
 * @param2				-
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */

void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	// peripheral clock enable
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		// enable Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		// enable Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    // configure the Word length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_MO ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//EVEN parity is default

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		// enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    // enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	// configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

}




/*****************************************************************
 * @fn 					-  USART_DeInit
 *
 * @brief				-reset all the register of port
 *
 * @param1   			-peripheral definition
 *
 * @return				-
 *
 * @note				-
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx==USART1) {
		USART1_REG_RESET();
	}
	else if(pUSARTx==USART2) {
		USART2_REG_RESET();
	}
	else if(pUSARTx==USART3) {
		USART3_REG_RESET();
	}
	else if(pUSARTx==UART4) {
		UART4_REG_RESET();
	}
	else if(pUSARTx==UART5) {
		UART5_REG_RESET();
	}
	else if(pUSARTx==USART6) {
		USART6_REG_RESET();
	}


}

/*
 * Data Send and Receive
 */
/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief				-performing sending data
 *
 * @param1   			-pointer to usart register peripheral
 * @param2				-pointer to buffer to hold the data we got
 * @param3				-length of the data to send
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//wait until TXE flag is set in the ISR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//8-bit data transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);

			//increment the buffer address
			pTxBuffer++;
		}
	}

	//wait until TC flag is set in the ISR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief				-the function is used to receive data from another device
 *
 * @param1   			-pointer to usart register peripheral
 * @param2				-pointer to data to send
 * @param3				-length of the data to receive
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
					pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
                *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
            }

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief				-performing sending data with interrupt when the TXDR is empty
 *
 * @param1   			-pointer to usart  handler
 * @param2				-pointer to buffer to hold the data we want to send
 * @param3				-length of the data to send
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1=(1<<USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1=(1<<USART_CR1_TCIE);


	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief				-the function is used to receive data from another device when get interrupt from the RXNE
 *
 * @param1   			-pointer to usart handler
 * @param2				-pointer to data to send
 * @param3				-length of the data to receive
 *
 * @return            uint8_t 
 *
 * @Note              -

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |=(1<<USART_CR1_RXNEIE);

	}

	return rxstate;

}
/*
 * IRQ Configuration and ISR handling
 */

/*****************************************************************
 * @fn 					-USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn 					-  USART_IRQPriorityConfig
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	//not all the bits applicable at this registers we use the first 4 bits
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	//setting the priority of processor register by dividing the irq number we get the first address for each register

	*(NVIC_PR_BASE_ADDR+(iprx))|= (IRQPriority<<(shift_amount));

}


/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2;
	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << USART_ISR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->ICR |= ( 1 << USART_ICR_TCCF);

				//Implement the code to clear the TCIE control bit

				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer=NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen=0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << USART_ISR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer+=2;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen--;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~ ( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << USART_ISR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne

		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer+=2;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen--;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << USART_ISR_CTS);
	
	//check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	
	//check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	// temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->ICR |= ( 1 << USART_ICR_CTSCF);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << USART_ISR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		pUSARTHandle->pUSARTx->ICR |= ( 1 << USART_ICR_IDLECF);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 <<USART_ISR_ORE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 <<USART_CR1_RXNEIE);


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		pUSARTHandle->pUSARTx->ICR |= ( 1 << USART_ICR_ORECF);

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->ISR;
		if(temp1 & ( 1 << USART_ISR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_ISR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_ISR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}


}


/*
 * Other Peripheral Control APIs
 */

/*****************************************************************
 * @fn 					-USART_GetFlagStatus
 *
 * @brief				-checks is the flag is set or not
 *
 * @param1   			-pointer to usart register peripheral
 * @param2				-macro of flag identification
 *
 * @return				-  none
 *
 * @note				-  '1' or zero if the flag is set
 *
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if (pUSARTx->ISR & StatusFlagName) {
		return SET;

	} else {
		return RESET;
	}

}
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @fn 					-USART_PeripheralControl
 *
 * @brief				-to enable the usart peripheral
 *
 * @param1   			-pointer to usart register peripheral
 * @param2				-enable or disable
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi==ENABLE) {
		pUSARTx->CR1 |= (1<<USART_CR1_UE);
	} else {

		pUSARTx->CR1 &=~ (1<<USART_CR1_UE);
	}
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -pointer to usart register peripheral
 * @param[in]         -desired baud rate
 *

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
    //Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    uint32_t tempreg=0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pUSARTx == USART1 || pUSARTx == USART6)
    {
        //USART1 and USART6 are hanging on APB2 bus
        PCLKx = RCC_GetPCLK2Value();
    }else
    {
        //USART2,3 and UART4,5,7,8 are hanging on APB1 bus
        PCLKx = RCC_GetPCLK1Value();
    }

    //Check for OVER8 configuration bit
    if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
    {
        //OVER8 = 1 , over sampling by 8
        usartdiv = (((PCLKx * 2) + (BaudRate / 2)) / BaudRate);     // Rounded up.

        tempreg = (uint16_t)(usartdiv & 0xFFF0);                    // BRR[15:4] = USARTDIV[15:4]
        tempreg |= (uint16_t)((usartdiv & (uint16_t)0x000F) >> 1);  // BRR[2:0]  = USARTDIV[3:0] shifted 1 bit to the right.
    }
    else
    {
        //over sampling by 16
        tempreg = ((PCLKx + (BaudRate / 2)) / BaudRate); // BRR = USARTDIV. Rounded up.
    }

    //copy the value of tempreg into BRR register
    pUSARTx->BRR = tempreg;
}

/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);

