/*
 * stmf767xx_spi_driver.h
 *
 *  Created on: 11 Oct 2023
 *      Author: Eliran.Malki
 */

#ifndef INC_STM32F767XX_SPI_DRIVER_H_
#define INC_STM32F767XX_SPI_DRIVER_H_

/*
 *Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;			/*<!possible values from @Spi_DeviceMode 	>*/
	uint8_t	SPI_BusConfig;			/*<!possible values from @SPI_BusConfig     >*/
	uint8_t	SPI_SclkSpeed;			/*<!possible values from @SPI_SPEED		    >*/
	uint8_t	SPI_DFF;				/*<!possible values from @DATA_FRAME_FORMAT >*/
	uint8_t	SPI_CPOL;				/*<!possible values from @SPI_CLOCK_POLARITY>*/
	uint8_t	SPI_CPHA;				/*<!possible values from @SPI_CLOCK_PHASE   >*/
	uint8_t	SPI_SSM;				/*<!possible values from @GPIO_PIN_PO	    >*/
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t *pSPIx;	/*< This holds the base address of the SPIx(x:1,2,3,4,5,6) peripheral >*/
	SPI_Config_t SPI_Config;/*< This holds GPIO pin configuration settings 						  >*/
	uint8_t		 *pTxBuffer;/*< To store the app. Tx buffer address 							  >*/
	uint8_t		 *pRxBuffer;/*< To store the app. Rx buffer address 							  >*/
	uint32_t	 TxLen;		/*< To store Tx Len													  >*/
	uint32_t	 RxLen;		/*< To store Rx Len													  >*/
	uint32_t	 TxState;	/*< To store Tx State												  >*/
	uint32_t	 RxState;	/*< To store Rx State												  >*/

}SPI_Handle_t;


/*
 * @Spi_DeviceMode
 * configure the device as a slave or master
 */

#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1

/*
 * @SPI_SclkConfig
 * SPI speeds options
 */

#define  SPI_SCLK_SPEED_DIV2			0//Default
#define  SPI_SCLK_SPEED_DIV4			1
#define  SPI_SCLK_SPEED_DIV8			2
#define  SPI_SCLK_SPEED_DIV16			3
#define  SPI_SCLK_SPEED_DIV32			4
#define  SPI_SCLK_SPEED_DIV64			5
#define  SPI_SCLK_SPEED_DIV128			6
#define  SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_BusConfig
 * configure the communication between the devices
 */

#define SPI_BUS_CONFIG_FD				 0
#define SPI_BUS_CONFIG_HD				 1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	 2


/*
 * @DATA_FRAME_FORMAT(DFF)
 * configure the data as 8 or 16 bit
 */

#define SPI_DFF_8BITS					7
#define SPI_DFF_16BITS					15

/*
* @SPI_CLOCK_POLARITY
* configure the idling as '0' or '1'
*/

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
* @SPI_CLOCK_PHASE
* configure the idling as '0' or '1'
*/

#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

/*
* @SPI_SLAVE_MANAGEMENT
* When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
*/

#define SPI_SSM_DI						0
#define SPI_SSM_EN						1


/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG					(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1<<SPI_SR_BSY)

/*
 * SPI application state
 */

#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   			1
#define SPI_EVENT_RX_CMPLT   			2
#define SPI_EVENT_OVR_ERR    			3
#define SPI_EVENT_CRC_ERR    			4




/*************************************************************************************************************
*  											APIs supported by this driver
*  							For more information about this APIs check the function definitions
 **************************************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHnadle);



/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_FRXTHConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F767XX_SPI_DRIVER_H_ */
