/*
 * stmf767xx.h
 *
 *  Created on: Oct 2, 2023
 *      Author: Eliran.Malki
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include <stddef.h>
#include <stdint.h>
#define __vo volatile
#define __weak __attribute__((weak))

/*************************************************START:Processor Specific Detalis**********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx(Interrupt Set-enable Registers) register Addresses
 */

# define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
# define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
# define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
# define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx(Interrupt Clear-enable Registers) register Addresses
 */

# define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
# define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
# define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
# define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED  4

/*
 * ARM Cortex Mx Coprocessor Access Control Register
 */
# define CPACR				((__vo uint32_t*)0xE000ED88)

/*
 * base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U // main memory
#define SRAM1_BASEADDR			0x20020000U //368KB
#define SRAM2_BASEADDR			0x2007C000U
#define ROM_BASEADDR			0x1FF00000U //system memory
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE 		PERIPH_BASE
#define APB2PERIPH_BASE 		0x40010000U
#define AHB1PERIPH_BASE 		0x40020000U
#define AHB2PERIPH_BASE 		0x50000000U
#define AHB3PERIPH_BASE 		0xA0000000U

/*
 * Base addresses of peripheral which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE+0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE+0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE+0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE+0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE+0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASE+0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASE+0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x3800)

/*
 * Base addresses of peripheral which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE+0x5C00)
#define I2C4_BASEADDR			(APB1PERIPH_BASE+0x6000)

#define SPI2_BASEADDR			(APB1PERIPH_BASE+0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE+0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE+0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE+0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE+0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE+0x5000)
#define UART7_BASEADDR			(APB1PERIPH_BASE+0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASE+0x7C00)


/*
 * Base addresses of peripheral which are hanging on APB2 bus
 */
#define EXTI_BASEADDR			(APB2PERIPH_BASE+0x3C00)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASE+0x3800)

#define SPI1_BASEADDR			(APB2PERIPH_BASE+0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE+0x3400)
#define SPI5_BASEADDR			(APB2PERIPH_BASE+0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASE+0x5400)

#define USART1_BASEADDR			(APB2PERIPH_BASE+0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE+0x1400)




/*************************************peripheral register definition structure***************************************/

//some elements(registers) may be high volatile in nature like input register so we configure all registers as volatile, we write volatile as __va


/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t MODER;	/*!GPIO port mode register												Address offset: 0x00*/
	__vo uint32_t OTYPER;	/*GPIO port output type register										Address offset: 0x04*/
	__vo uint32_t OSPEEDR;	/*GPIO port output speed register										Address offset: 0x08*/
	__vo uint32_t PUPDR;	/*GPIO port pull-up/pull-down register									Address offset: 0x0C*/
	__vo uint32_t IDR;		/*GPIO port input data register											Address offset: 0x10*/
	__vo uint32_t ODR;		/*GPIO port output data register										Address offset: 0x14*/
	__vo uint32_t BSRR;		/*GPIO port bit set/reset register										Address offset: 0x18*/
	__vo uint32_t LCKR;		/*GPIO port configuration lock register									Address offset: 0x1C*/
	__vo uint32_t AFR[2];	/*GPIO alternate function low register-AFR[0]->Address offset: 0x20 ,GPIO alternate function high register ->Address offset: 0x24*/

}GPIO_RegDef_t;



/*
 * peripheral register definition structure for SPI peripheral
 */

typedef struct
{
	__vo uint32_t CR1;		/*!SPI control register 1												Address offset: 0x00*/
	__vo uint32_t CR2;		/*!SPI control register 2												Address offset: 0x004*/
	__vo uint32_t SR;		/*SPI status register 													Address offset: 0x08*/
	__vo uint32_t DR;		/*SPI data register														Address offset: 0x0C*/
	__vo uint32_t CRCPR;	/*SPI CRC polynomial register											Address offset: 0x10*/
	__vo uint32_t RXCRCR;	/*SPI Rx CRC register													Address offset: 0x14*/
	__vo uint32_t TXCRCR;	/*SPI Tx CRC register												    Address offset: 0x18*/
	__vo uint32_t I2SCFGR;	/*SPIx_I2S configuration register										Address offset: 0x1C*/
	__vo uint32_t I2SPR;	/*SPIx_I2S prescaler register											Address offset: 0x20*/

}SPI_RegDef_t;



/*
 * peripheral register definition structure for I2C peripheral
 */

typedef struct
{
	__vo uint32_t CR1;		/*!I2C control register 1												Address offset: 0x00*/
	__vo uint32_t CR2;		/*!I2C control register 2												Address offset: 0x004*/
	__vo uint32_t OAR1;		/*Own address 1 register 												Address offset: 0x08*/
	__vo uint32_t OAR2;		/*Own address 2 register												Address offset: 0x0C*/
	__vo uint32_t TIMINGR;	/*Timing register														Address offset: 0x10*/
	__vo uint32_t TIMEOUTR;	/*Timeout register														Address offset: 0x14*/
	__vo uint32_t ISR;		/*Interrupt and status register										    Address offset: 0x18*/
	__vo uint32_t ICR;		/*Interrupt clear register												Address offset: 0x1C*/
	__vo uint32_t PECR;		/*PEC register															Address offset: 0x20*/
	__vo uint32_t RXDR;		/*Receive data register													Address offset: 0x24*/
	__vo uint32_t TXDR;		/*Transmit data register												Address offset: 0x28*/

}I2C_RegDef_t;


/*
 * peripheral register definition structure for USART
 */
typedef struct
{

	__vo uint32_t CR1;        /*!<     											Address offset: 0x00 */
	__vo uint32_t CR2;        /*!<      										Address offset: 0x04 */
	__vo uint32_t CR3;        /*!<      										Address offset: 0x08 */
	__vo uint32_t BRR;        /*!<      										Address offset: 0x0C */
	__vo uint32_t GTPR;       /*!<      										Address offset: 0x10 */
	__vo uint32_t RTOR;       /*!<     											Address offset: 0x14 */
	__vo uint32_t RQR;        /*!<      										Address offset: 0x18 */
	__vo uint32_t ISR;        /*!<      										Address offset: 0x1C */
	__vo uint32_t ICR;        /*!<      										Address offset: 0x20 */
	__vo uint32_t RDR;        /*!<      										Address offset: 0x24 */
	__vo uint32_t TDR;        /*!<      										Address offset: 0x28 */

} USART_RegDef_t;





/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t CR	;	/*!RCC clock control register											Address offset: 0x00*/
	__vo uint32_t PLLCFGR;	/*!RCC PLL configuration register										Address offset: 0x04*/
	__vo uint32_t CFGR;		/*!RCC clock configuration register										Address offset: 0x08*/
	__vo uint32_t CIR;		/*!RCC clock interrupt register											Address offset: 0x0C*/
	__vo uint32_t AHB1RSTR;	/*!RCC AHB1 peripheral reset register									Address offset: 0x10*/
	__vo uint32_t AHB2RSTR;	/*!RCC AHB2 peripheral reset register									Address offset: 0x14*/
	__vo uint32_t AHB3RSTR;	/*!RCC AHB3 peripheral reset register									Address offset: 0x18*/
	uint32_t	  RESERVED0;/*!Reserved, 0x1C																			*/
	__vo uint32_t APB1RSTR;	/*!RCC APB1 peripheral reset register									Address offset: 0x20*/
	__vo uint32_t APB2RSTR;	/*!RCC APB2 peripheral reset register									Address offset: 0x24*/
	uint32_t	  RESERVED1[2];/*Reserved, 0x28-0x2c   */
	__vo uint32_t AHB1ENR;	/*!RCC AHB1 peripheral clock register									Address offset: 0x30*/
	__vo uint32_t AHB2ENR;	/*!RCC AHB2 peripheral clock enable register							Address offset: 0x34*/
	__vo uint32_t AHB3ENR;	/*!RCC AHB3 peripheral clock enable register							Address offset: 0x38*/
	uint32_t	  RESERVED2;/*!Reserved, 0x3C																			*/
	__vo uint32_t APB1ENR;	/*!RCC APB1 peripheral clock enable register							Address offset: 0x40*/
	__vo uint32_t APB2ENR;	/*!RCC APB2 peripheral clock enable register							Address offset: 0x44*/
	uint32_t	  RESERVED3[2];/*Reserved, 0x48-0x4c   */
	__vo uint32_t AHB1LPENR;/*!RCC AHB1 peripheral clock enable in low-power mode register			Address offset: 0x50*/
	__vo uint32_t AHB2LPENR;/*!RCC AHB2 peripheral clock enable in low-power mode register			Address offset: 0x54*/
	__vo uint32_t AHB3LPENR;/*!RCC AHB3 peripheral clock enable in low-power mode register			Address offset: 0x58*/
	uint32_t	  RESERVED4;/*!Reserved, 0x5C																			*/
	__vo uint32_t APB1LPENR;/*!RCC APB1 peripheral clock enable in low-power mode register			Address offset: 0x60*/
	__vo uint32_t APB2LPENR;/*!RCC APB2 peripheral clock enabled in low-power mode register			Address offset: 0x64*/
	uint32_t	  RESERVED5[2];/*Reserved, 0x68-0x6c   */
	__vo uint32_t BDCR;		/*!RCC backup domain control register									Address offset: 0x70*/
	__vo uint32_t CSR;		/*!RCC clock control & status register									Address offset: 0x74*/
	uint32_t	  RESERVED6[2];/*Reserved, 0x78-0x7c   */
	__vo uint32_t SSCGR;	/*!RCC spread spectrum clock generation register						Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;/*!RCC PLLI2S configuration register									Address offset: 0x84*/
	__vo uint32_t PLLSAICFGR;/*!RCC PLLSAI configuration register									Address offset: 0x88*/
	__vo uint32_t RCC_DCKCFGR1;/*!RCC dedicated clocks configuration register						Address offset: 0x8C*/
	__vo uint32_t DCKCFGR2;		/*!RCC dedicated clocks configuration register						Address offset: 0x90*/
}RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;		/*!Interrupt mask register												Address offset: 0x00*/
	__vo uint32_t EMR;		/*Event mask register													Address offset: 0x04*/
	__vo uint32_t RTSR;		/*Rising trigger selection register										Address offset: 0x08*/
	__vo uint32_t FTSR;		/*Falling trigger selection register									Address offset: 0x0C*/
	__vo uint32_t SWIER;	/*Software interrupt event register										Address offset: 0x10*/
	__vo uint32_t PR;		/*Pending register														Address offset: 0x14*/

}EXTI_RegDef_t;



/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;	/*!SYSCFG memory remap register											Address offset: 0x00*/
	__vo uint32_t PMC;		/*peripheral mode configuration register								Address offset: 0x04*/
	__vo uint32_t EXTICR[4];/*SYSCFG external interrupt configuration register 				Address offset: 0x08 to 0x14*/
	uint32_t RESERVED;		/*SYSCFG external interrupt configuration register 2							  Reserved,0x18 */
	__vo uint32_t CBR;		/*Class B register														Address offset: 0x1c*/
	__vo uint32_t CMPCR;	/*Compensation cell control register									Address offset: 0x20*/

}SYSCFG_RegDef_t;



/*
 * peripheral definitions ( Peripheral base addresses typecasted to XXX_RegDef_t)
 */

#define GPIOA 			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 			((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 			((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 			((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5			((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6			((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4			((I2C_RegDef_t*)I2C4_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)


#define  RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define  EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define  SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |=(1<<8))
#define GPIOJ_PCLK_EN()	(RCC->AHB1ENR |=(1<<9))
#define GPIOK_PCLK_EN()	(RCC->AHB1ENR |=(1<<10))
/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |=(1<<23))
#define I2C4_PCLK_EN() (RCC->APB1ENR |=(1<<24))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |=(1<<13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |=(1<<20))
#define SPI6_PCLK_EN() (RCC->APB2ENR |=(1<<21))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()  (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN() (RCC->APB2ENR |=(1<<5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |=(1<<14))

/*
 * Clock Disable Macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &=~(1<<8))
#define GPIOJ_PCLK_DI()	(RCC->AHB1ENR &=~(1<<9))
#define GPIOK_PCLK_DI()	(RCC->AHB1ENR &=~(1<<10))
/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &=~(1<<23))
#define I2C4_PCLK_DI() (RCC->APB1ENR &=~(1<<24))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &=~(1<<13))
#define SPI5_PCLK_DI() (RCC->APB2ENR &=~(1<<20))
#define SPI6_PCLK_DI() (RCC->APB2ENR &=~(1<<21))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI() (RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &=~(1<<19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI() (RCC->APB2ENR &=~(1<<5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &=~(1<<14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<0));	(RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<1));	(RCC->AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<2));	(RCC->AHB1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<3));	(RCC->AHB1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<4));	(RCC->AHB1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<5));	(RCC->AHB1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<6));	(RCC->AHB1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<7));	(RCC->AHB1RSTR &=~(1<<7));}while(0)
#define GPIOI_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<8));	(RCC->AHB1RSTR &=~(1<<8));}while(0)
#define GPIOJ_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<9));	(RCC->AHB1RSTR &=~(1<<9));}while(0)
#define GPIOK_REG_RESET() 			do{(RCC->AHB1RSTR |=(1<<10));	(RCC->AHB1RSTR &=~(1<<10));}while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<12));	(RCC->APB2RSTR &=~(1<<12));}while(0)
#define SPI2_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<14));	(RCC->APB1RSTR &=~(1<<14));}while(0)
#define SPI3_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<15));	(RCC->APB1RSTR &=~(1<<15));}while(0)
#define SPI4_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<13));	(RCC->APB2RSTR &=~(1<<13));}while(0)
#define SPI5_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<20));	(RCC->APB2RSTR &=~(1<<20));}while(0)
#define SPI6_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<21));	(RCC->APB2RSTR &=~(1<<21));}while(0)

/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<21));	(RCC->APB2RSTR &=~(1<<21));}while(0)
#define I2C2_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<22));	(RCC->APB1RSTR &=~(1<<22));}while(0)
#define I2C3_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<23));	(RCC->APB1RSTR &=~(1<<23));}while(0)
#define I2C4_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<24));	(RCC->APB2RSTR &=~(1<<24));}while(0)
/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<4));	(RCC->APB2RSTR &=~(1<<4));}while(0)
#define USART2_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<17));	(RCC->APB1RSTR &=~(1<<17));}while(0)
#define USART3_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<18));	(RCC->APB1RSTR &=~(1<<18));}while(0)
#define UART4_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<19));	(RCC->APB1RSTR &=~(1<<19));}while(0)
#define UART5_REG_RESET() 			do{(RCC->APB1RSTR |=(1<<20));	(RCC->APB1RSTR &=~(1<<20));}while(0)
#define USART6_REG_RESET() 			do{(RCC->APB2RSTR |=(1<<5));	(RCC->APB2RSTR &=~(1<<5));}while(0)


/*
 * returns port code for given GPIOx base address
 */
/*
 * This macro returns a code(between 0 to 10) for a given GPIOx base address(x)
 */
//I did here a C conditional operation, "?"  is mean if its true then, ":" is mean else
#define GPIO_BASEADDR_TO_CODE(x)    ((x==GPIOA) ? 0 :\
									 (x==GPIOB) ? 1 :\
									 (x==GPIOC) ? 2 :\
									 (x==GPIOD) ? 3 :\
									 (x==GPIOE) ? 4 :\
									 (x==GPIOF) ? 5 :\
									 (x==GPIOG) ? 6 :\
									 (x==GPIOH) ? 7 :\
									 (x==GPIOI) ? 8 :\
									 (x==GPIOJ) ? 9 :\
									 (x==GPIOK) ? 10 :0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F767xx MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI4				91
#define IRQ_NO_SPI5				92
#define IRQ_NO_SPI6				93

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73
#define IRQ_NO_I2C4_EV			95
#define IRQ_NO_I2C4_ER			96

#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			93


/*
 * macros for all the possible IRQ priority levels
 */

#define NVIC_IRQ_PRIO			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI1O			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15



//some generic macros
#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


/*******************************************************************************************
 * Bit position definitions of SPI peripheral
 *******************************************************************************************/

/*
 * Bit position definitions for SPI_CR1
 */
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSB_FIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RX_ONLY				10
#define SPI_CR1_CRCL				11
#define SPI_CR1_CRC_NEXT			12
#define SPI_CR1_CRC_EN				13
#define SPI_CR1_BIDI_ON				14
#define SPI_CR1_BIDI_MODE			15


/*
 * Bit position definitions for SPI_CR2
 */
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_NSSP				3
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7
#define SPI_CR2_DS					8
#define SPI_CR2_FRXTH				12
#define SPI_CR2_LDMA_RX				13
#define SPI_CR2_LDMA_TX				14


/*
 * Bit position definitions for SPI_CSR
 */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRC_ERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8
#define SPI_SR_FRLVL				9
#define SPI_SR_FTLVL				11


/*******************************************************************************************
 * Bit position definitions of I2C peripheral
 *******************************************************************************************/
/*
 * Bit position definitions for I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_TXIE			1
#define I2C_CR1_RXIE			2
#define I2C_CR1_ADDRIE			3
#define I2C_CR1_NACKIE			4
#define I2C_CR1_STOPIE			5
#define I2C_CR1_TCIE			6
#define I2C_CR1_ERRIE			7
#define I2C_CR1_DNF				8
#define I2C_CR1_ANFOFF			12
#define I2C_CR1_TXDMAEN			14
#define I2C_CR1_RXDMAEN			15
#define I2C_CR1_SBC				16
#define I2C_CR1_NOSTRETCH		17
#define I2C_CR1_GCEN			19
#define I2C_CR1_SMBHEN			20
#define I2C_CR1_SMBDEN			21
#define I2C_CR1_ALERTEN			22
#define I2C_CR1_PECEN			23

/*
 * Bit position definitions for I2C_CR2
 */
#define I2C_CR2_SADD			0
#define I2C_CR2_RD_WRN			10
#define I2C_CR2_ADD10			11
#define I2C_CR2_HEAD10R			12
#define I2C_CR2_START			13
#define I2C_CR2_STOP			14
#define I2C_CR2_NACK			15
#define I2C_CR2_NBYTES			16
#define I2C_CR2_RELOAD			24
#define I2C_CR2_AUTOEND			25
#define I2C_CR2_PECBYTE			26


/*
 * Bit position definitions for I2C_ISR
 */
#define I2C_ISR_TXE				0
#define I2C_ISR_TXIS			1
#define I2C_ISR_RXNE			2
#define I2C_ISR_ADDR			3
#define I2C_ISR_NACKF			4
#define I2C_ISR_STOPF			5
#define I2C_ISR_TC				6
#define I2C_ISR_TCR				7
#define I2C_ISR_BERR			8
#define I2C_ISR_ARLO			9
#define I2C_ISR_OVR				10
#define I2C_ISR_PECERR			11
#define I2C_ISR_TIMEOUT			12
#define I2C_ISR_ALERT			13
#define I2C_ISR_BUSY			15
#define I2C_ISR_DIR				16
#define I2C_ISR_ADDCODE			17

/*
 * Bit position definitions for I2C_ICR
 */
#define I2C_ICR_ADDRCF      3
#define I2C_ICR_NACKCF      4
#define I2C_ICR_STOPCF      5
#define I2C_ICR_BERRCF      8
#define I2C_ICR_ARLOCF      9
#define I2C_ICR_OVRCF       10
#define I2C_ICR_PECCF       11
#define I2C_ICR_TIMEOUTCF   12
#define I2C_ICR_ALERTCF     13

/*
 * Bit position definitions for I2C_TIMINGR
 */
#define I2C_TIMINGR_SCLL			0
#define I2C_TIMINGR_SCLH			8
#define I2C_TIMINGR_SDADEL			16
#define I2C_TIMINGR_SCLDEL			20
#define I2C_TIMINGR_PRESC			28

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_UE					0
#define USART_CR1_UESM					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_MO 					12
#define USART_CR1_MME 					13
#define USART_CR1_CMIE					14
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */

#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14
#define USART_CR2_ADD   				24


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_ISR
 */

#define USART_ISR_PE       				0
#define USART_ISR_FE       				1
#define USART_ISR_NF       				2
#define USART_ISR_ORE      				3
#define USART_ISR_IDLE       			4
#define USART_ISR_RXNE        			5
#define USART_ISR_TC       				6
#define USART_ISR_TXE        			7
#define USART_ISR_LBDF        			8
#define USART_ISR_CTSIF        			9
#define USART_ISR_CTS					10
#define USART_ISR_RTOF					11


/*
 * Bit position definitions USART_ISR
 */


#define USART_ICR_PECF      0
#define USART_ICR_FECF      1
#define USART_ICR_NCF       2
#define USART_ICR_ORECF     3
#define USART_ICR_IDLECF    4
#define USART_ICR_TCCF      6
#define USART_ICR_TCBGTCF   7
#define USART_ICR_LBDCF     8
#define USART_ICR_CTSCF     9
#define USART_ICR_RTOCF     11
#define USART_ICR_EOBCF     12
#define USART_ICR_CMCF      17
#define USART_ICR_WUCF      20



#include "stm32f767xx_gpio_driver.h"
#include "stm32f767xx_spi_driver.h"
#include "stm32f767xx_i2c_driver.h"
#include "stm32f767xx_usart_driver.h"
#include "stm32f767xx_rcc_driver.h"

#endif /* INC_STM32F767XX_H_ */
