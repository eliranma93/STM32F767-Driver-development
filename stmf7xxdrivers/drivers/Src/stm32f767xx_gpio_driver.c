/*
 * stmf767xx_gpio_driver.c
 *
 *  Created on: Oct 3, 2023
 *      Author: Eliran.Malki
 */

#include <stm32f767xx_gpio_driver.h>


/*
 * Peripheral Clock setup
 */

/*****************************************************************
 * @fn 					-  GPIO_PeriClockControl
 *
 * @brief				-  This function enables or disables peripheral clock for the given GPIO port
 *
 * @param1   			-   base address
 * @param2				-	ENABLE or DISABLE macros
 *
 * @return				-  none
 *
 * @note				-  none
 *
 */

void delay(void){
  for (uint32_t i = 0; i < 200000; i++)
    ;
}

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi)
{
	if (EnorDi==ENABLE) {
		if (pGPIOx==GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF) {
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG) {
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH) {
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOI) {
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx==GPIOJ) {
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx==GPIOK) {
			GPIOK_PCLK_EN();
		}
	} else {
		if (pGPIOx==GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF) {
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG) {
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH) {
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx==GPIOI) {
			GPIOI_PCLK_DI();
		}
		else if(pGPIOx==GPIOJ) {
			GPIOJ_PCLK_DI();
		}
		else if(pGPIOx==GPIOK) {
			GPIOK_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn 					-  GPIO_Init
 *
 * @brief				-configure the mode of gpio pin
 * 						-configure the speed
 *						-configure the pupd settings
 *						-configure the alt functionality
 *						-
 * @param1   			-Handle structure GPIO pin
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;//temp register

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//1. configure the mode of gpio pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//reset value
		pGPIOHandle->pGPIOx->MODER|=temp;//setting

	}else
	{
		// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT) {

			//1.configure the FTSR
			EXTI->FTSR |=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clean the corresponding RTSR
			EXTI->RTSR &=~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){

			//1.configure the FTSR
			EXTI->RTSR |=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clean the corresponding RTSR
			EXTI->FTSR &=~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT ) {

			//1.configure both RTSR and FTSR
			EXTI->FTSR |=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=(portcode<<(temp2*4));


		//3.enable the exti interrupt delivery using IMR
		EXTI->IMR |=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;

	//2. configure the speed

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//reset value
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;//setting
	temp=0;

	//3. configure the pupd settings

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//reset value
	pGPIOHandle->pGPIOx->PUPDR|=temp;//setting
	temp=0;

	//4. configure the optype

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//reset value
	pGPIOHandle->pGPIOx->OTYPER|=temp;//setting
	temp=0;

	//5. configure the alt functionality

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALT_FN) {
		//configure the alt function registers.
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &=~ (0xF<<(4*temp2));//reset value
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));//setting
	}
}

/*****************************************************************
 * @fn 					-  GPIO_DeInit
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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx==GPIOA) {
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB) {
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC) {
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD) {
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE) {
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF) {
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG) {
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH) {
		GPIOH_REG_RESET();
	}
	else if(pGPIOx==GPIOI) {
		GPIOI_REG_RESET();
	}
	else if(pGPIOx==GPIOJ) {
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx==GPIOK) {
		GPIOK_REG_RESET();
}
}
/*
 * Data read and write
 */

/*****************************************************************
 * @fn 					-  GPIO_ReadFromInputPin
 *
 * @brief				-read pin position input
 *
 * @param1   			-peripheral definition
 * @param2				-PinNumber
 *
 * @return				-0 or 1
 *
 * @note				-
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR>>PinNumber)&0x00000001);
	return value;
}

/*****************************************************************
 * @fn 					-  GPIO_ReadFromInputPprt
 *
 * @brief				-read port position input
 *
 * @param1   			-peripheral definition
 *
 * @return				-0 to 11111111
 *
 * @note				-
 *
 */
uint16_t GPIO_ReadFromInputPprt(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*****************************************************************
 * @fn 					-  GPIO_WriteToOutputPin
 *
 * @brief				-by given pin number and value we write it out to the given peripheral
 *
 * @param1   			-peripheral definition
 * @param2				-pin number
 * @param3				-value out
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber,uint8_t Value)
{
	if (Value==GPIO_PIN_SET) {
		//Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1<<PinNumber);

	} else {
		//Write 0
		pGPIOx->ODR &=~ (Value<<PinNumber);
	}
}

/*****************************************************************
 * @fn 					-  GPIO_WriteToOutputPort
 *
 * @brief				-by given port peripheral and value we write it out
 *
 * @param1   			-peripheral definition
 * @param2				-value out
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/*****************************************************************
 * @fn 					-  GPIO_ToggleputPin
 *
 * @brief				-
 *
 * @param1   			-
 * @param2				-	s
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_ToggleputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}
/*
 * IRQ Configuration and ISR handling
 */

/*****************************************************************
 * @fn 					-  GPIO_IRQInterruptConfig
 *
 * @brief				-configure the interrupt at the processor side
 *
 * @param1   			-IRQNumber same in the NVIC
 *  @param2				- ENABLE or DISABLE
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
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
 * @fn 					-  GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
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
 * @fn 					-  GPIO_IRQHandling
 *
 * @brief				-reset the pending register to ready for the next interrupt
 *
 * @param1   			-PinNumber
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR&(1<<PinNumber))
	{
		//clear
		EXTI->PR|=(1<<PinNumber);
	}
}

