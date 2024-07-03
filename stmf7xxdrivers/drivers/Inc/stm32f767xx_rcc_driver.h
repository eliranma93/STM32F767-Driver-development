/*
 * stm32f767xx_rcc_driver.h
 *
 *  Created on: Nov 5, 2023
 *      Author: Eliran.Malki
 */

#ifndef INC_STM32F767XX_RCC_DRIVER_H_
#define INC_STM32F767XX_RCC_DRIVER_H_

/*********************************************************************************
 *                  APIs supported by this driver
 *      For more infos, check the function definitions
 *********************************************************************************/

uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F767XX_RCC_DRIVER_H_ */
