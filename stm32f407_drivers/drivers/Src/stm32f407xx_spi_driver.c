/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Apr 16, 2025
 *      Author: ANAND S
 */

#include "stm32407xx_spi_driver.h"

/*Helper functions
 * *
 */

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//Enable the SPI peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//Configure SPI_CR1 register

	uint32_t tempreg=0;

	//configure the device mode

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configure the bus mode
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
				//bidi mode should be cleared
		tempreg &= ~(1 <<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
			//bidi mode should be set
		tempreg |= (1 <<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 <<15);
		//RXONLY bit must be set
		tempreg |= (1 <<10);
	}

	//3. Configure the SPI peripheral clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF

	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure  the CPOL

	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the  CPHA

	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //todo
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t  FlagName)
{
	if(pSPIx->SR & FlagName)
		return FLAG_SET;
	else

		return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}


}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
		}
}




/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)  //Polling based function code
{
	while(Len > 0)
	{
		//1. Wait until TXE is Set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG == FLAG_RESET));

		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRXBuffer,uint32_t Len)
{

	while(Len > 0)
		{
			//1. Wait until RXNE is Set
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG == FLAG_RESET));

			//2. Check the DFF bit in CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				//16 bit and load the data to DR from RX buffer
				 *((uint16_t*)pRXBuffer) = pSPIx->DR;
				Len -= 2;
				(uint16_t*)pRXBuffer++;
			}
			else
			{
				//8 bit  and load the data to DR from RX buffer
				 *((uint16_t*)pRXBuffer) = pSPIx->DR;
				Len--;
				pRXBuffer++;
			}
		}

}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}


}


uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pTXBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save the Tx buffer address and Length information in some global variables
	pSPIHandle->pTxBuffer = pTXBuffer;
	pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other piece of code can take over and use the same SPI until the operation is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3.  Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	//4. Data Transmission is handled by the the ISR code

	}
	return state;

}


uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pRXBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_TX)
		{
		//1. Save the Tx buffer address and Length information in some global variables
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other piece of code can take over and use the same SPI until the operation is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3.  Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data Receive is handled by the the ISR code

		}
		return state;

}

/*Implementation of helper function
 *
 */

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
			if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				//16 bit
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen -= 2;
		        pSPIHandle->pTxBuffer += 2; // Advance by 2 bytes (16 bits)
			}
			else
			{
				//8 bit
				pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}

			if(! pSPIHandle->TxLen)
				{
					//TxLen is zero , so close the spi transmission and inform the application that
					//TX is over.

					//this prevents interrupts from setting up of TXE flag
					SPI_CloseTransmisson(pSPIHandle);
					SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
				}
}

static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
				{
					//16 bit
					*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
					pSPIHandle->RxLen -= 2;
			        pSPIHandle->pRxBuffer += 2; // Advance by 2 bytes (16 bits)
				}
				else
				{
					//8 bit
					*(pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->DR;
					 pSPIHandle->RxLen--;
					 pSPIHandle->pRxBuffer++;
				}

	if(! pSPIHandle->RxLen)
		{
			//reception is complete
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
		(void)temp;
		//2. inform the application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
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


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // Default weak implementation
}
