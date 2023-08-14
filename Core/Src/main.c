/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32f10x_flash.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
typedef void(*pFunction)(void);
void jump_to_start(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  SPI_CS_PIN  									LL_GPIO_PIN_12
#define  SPI_CS_PORT 									GPIOB
#define  SPI_HANDLE 									SPI2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
pFunction Jump_To_Application;
uint32_t  JumpAddress;
uint16_t  app_main_checksum=0;
uint16_t  app_new_checksum=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** DISABLE: JTAG-DP Disabled and SW-DP Disabled
  */
  LL_GPIO_AF_DisableRemap_SWJ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// each firmware on the Flash has a CRC that calculated in programming
		// also has a ONE BYTE signature for validity checking
		// if CRC are different and signeture byte = 63 then new firmware detected!
		app_main_checksum=(FEE_ReadDataByte_abs(Flash_lower_half_end-2-255)<<8)|FEE_ReadDataByte_abs(Flash_lower_half_end-1-255);
		
		//Read External NOR first SECTOR
		
		//if there is problem with Reading the NOR flash , just keep going with MCU main firmware
		if()
		{
			//Read New Firmware Checksum
			app_new_checksum=//Get firmware from external NOR flash.
		
			//Read New Firmware File Size

			//for both upgrade and downgrade just compare new and old firmware 
			//there is no need to use extra register. after update, firmware CRC will writen at end of FLASH memory
			if(app_main_checksum!=app_new_checksum)//so a new firmware exist in memory to update
			{
					//copy new firmware to lower flash location
					FEE_Init();
					FEE_Erase();
					Page_Copy(Flash_lower_half_start,Flash_uppper_half_start,Flash_uppper_half_end);
					FLASH_LockBank1();
			}
		}
		jump_to_start();
		while(1)
		{
			for(JumpAddress=0;JumpAddress<0x000FFFFF;JumpAddress++)
			{
			}
		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(NOR_CS_GPIO_Port, NOR_CS_Pin);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NOR_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(NOR_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void jump_to_start(void)
{
	
	/* If Program has been written */
	if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
		{
			
			//Make sure, the CPU is in privileged mode.
			//if( CONTROL_nPRIV_Msk & __get_CONTROL( ) )
			//{  /* not in privileged mode */
			//	EnablePrivilegedMode();
			//}			
			//Disable all enabled interrupts in NVIC.
			
			NVIC->ICER[ 0 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 1 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 2 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 3 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 4 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 5 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 6 ] = 0xFFFFFFFF ;
			NVIC->ICER[ 7 ] = 0xFFFFFFFF ;
			
			//Clear all pending interrupt requests in NVIC.
			
			NVIC->ICPR[ 0 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 1 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 2 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 3 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 4 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 5 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 6 ] = 0xFFFFFFFF ;
			NVIC->ICPR[ 7 ] = 0xFFFFFFFF ;
			
			//Disable SysTick and clear its exception pending bit,
			
			//__disable_irq();//EEEEERRRRRRROOOORRRR
			//__DSB();//EEEEERRRRRRROOOORRRR
			//__ISB();//EEEEERRRRRRROOOORRRR
		 
			SysTick->CTRL=0;		
			SysTick->LOAD=0;
			SysTick->VAL=0;
			///////////////////////__set_PRIMASK(1);/////////////////////////////EEEEERRRRRRROOOORRRR
			
			SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
			
			//Disable individual fault handlers if the bootloader used them.
			SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk );
			//Activate the MSP, if the core is found to currently run with the PSP.
			if(CONTROL_SPSEL_Msk& __get_CONTROL())
			{  /* MSP is not active */
				__set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk );
			}
			
			//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
			
			/* Set system control register SCR->VTOR  */
			//SCB->VTOR = NVIC_VectTab_FLASH | ((ApplicationAddress & 0x0000FFFF) & (uint32_t)0x1FFFFF80);
			
			NVIC_SetVectorTable(NVIC_VectTab_FLASH, ApplicationAddress & 0x0000FFFF);			
			
			//__DSB();
			//__ISB();
			
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			
			Jump_To_Application = (void (*)(void)) (*((uint32_t *) ((ApplicationAddress + 4))));
			Jump_To_Application(); 
		}
}


//###################################################################################################################
uint8_t W25qxx_Spi(uint8_t Data)
{
	uint8_t ret;
	//HAL_SPI_TransmitReceive(&_W25QXX_SPI, &Data, &ret, 1, 100);
	LL_SPI_TransmitData8(SPI_HANDLE, Data);
  while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE));
  
  while (LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE))
	{
		ret=LL_SPI_ReceiveData8(SPI_HANDLE);
	}
	return ret;
}

/*
uint8_t Init_SPI_W25Qxx(void)
{
  uint8_t ret = 1;
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  // MOSI 
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // MISO
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // SCK
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  // NSS
  GPIO_InitStruct.Pin = SPI_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  LL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);  
  LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
  // SPI
  LL_SPI_InitTypeDef spi;
  spi.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi.Mode = LL_SPI_MODE_MASTER;
  spi.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi.ClockPolarity = LL_SPI_POLARITY_LOW;
  spi.ClockPhase = LL_SPI_PHASE_1EDGE;
  spi.NSS = LL_SPI_NSS_SOFT;
  spi.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  spi.BitOrder = LL_SPI_MSB_FIRST;
  spi.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	
	//Enable SPI2 Clock - Note for Other SPI interfaces you must change this.
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	
  LL_SPI_Init(SPI_HANDLE, &spi);
  LL_SPI_Enable(SPI_HANDLE);

  // DMA Receive from SPI
  LL_DMA_InitTypeDef dma_spi_rx;
  dma_spi_rx.PeriphOrM2MSrcAddress = (uint32_t)&SPI_HANDLE->DR;
  dma_spi_rx.MemoryOrM2MDstAddress = (uint32_t)&dataBuffer;
  dma_spi_rx.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_spi_rx.Mode = LL_DMA_MODE_NORMAL;
  dma_spi_rx.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_spi_rx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_spi_rx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_spi_rx.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_spi_rx.Priority = LL_DMA_PRIORITY_HIGH;
   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_4, &dma_spi_rx); 

  // DMA Transmit to SPI
  LL_DMA_InitTypeDef dma_spi_tx;
  dma_spi_tx.PeriphOrM2MSrcAddress = (uint32_t)&SPI_HANDLE->DR;
  dma_spi_tx.MemoryOrM2MDstAddress = (uint32_t)&dataBuffer;
  dma_spi_tx.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_spi_tx.Mode = LL_DMA_MODE_NORMAL;
  dma_spi_tx.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_spi_tx.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_spi_tx.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_spi_tx.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_spi_tx.Priority = LL_DMA_PRIORITY_HIGH;
  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_5, &dma_spi_tx);

  ret = 0;
  return ret;
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
