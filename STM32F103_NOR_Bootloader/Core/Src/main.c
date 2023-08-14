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
#include "string.h"
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
#define WriteEnable             0x06
#define WtiteDisable            0x04
#define Read_JedecID            0x9f
#define Read_UniqueID           0x4b
#define Read_ManufatureID       0x90
#define Read_DeviceID           0xab
#define Read_StatusRegister_1   0x05
#define Read_StatusRegister_2   0x35
#define Read_StatusRegister_3   0x15
#define Write_StatusRegister    0x01
#define Write_StatusRegister_1  0x01
#define Write_StatusRegister_2  0x31
#define Write_StatusRegister_3  0x11
#define PageProgram             0x02
#define PageProgram_256         0x12
#define ReadData                0x03
#define FastRead                0x0b
#define FastRead_256            0x0c
#define Erase_Chip              0xc7
#define Erase_Block_64_256      0xdc
#define Erase_Block_64          0xd8
#define Erase_Block_32          0x52
#define Erase_Sector            0x20
#define Erase_Sector_256        0x21
#define Erase_Suspend           0x75
#define Erase_Resume            0x7h
#define PowerDown               0xb9
#define PowerDown_Release       0xab
#define Reset_Enable						0x66
#define Reset_Device						0x99
#define Global_Unlock						0x98
#define SR_Unlock								0x50

#define SPI_CS_PIN  						LL_GPIO_PIN_12
#define SPI_CS_PORT 						GPIOB
#define SPI_HANDLE 							SPI2
#define SPI_TIMEOUT 						1000
#define SPI_RETRY   						10

#define W25qxx_Delay(delay) 		LL_mDelay(delay)
#define w25qxx_SectorSize				0x1000  // 4096
#define w25qxx_PageSize     		0x0100  // 256
#define settings_sector					1

#define FLASH_PAGE_SIZE								1024
#define FLASH_Base_Address						0x8000000												// Bootloader start address
#define Main_Program_Base_Address   	FLASH_Base_Address + 0x00001000 // after 4KB for bootloader space
#define Main_Program_End_Address			FLASH_Base_Address + 0x00010000 // after 64KB flash size for STM32F103C8
#define ApplicationAddress    				Main_Program_Base_Address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
pFunction Jump_To_Application;
uint32_t  JumpAddress;
uint32_t  app_new_size=0;
uint32_t  app_new_checksum=0;
uint8_t 	w25qxx_Lock = 0;
uint8_t 	dataBuffer[w25qxx_SectorSize]; //4096 Byte Buffer
uint32_t  current_sector=0;
uint32_t  current_base_byte=0;
uint32_t  current_byte=0;
uint32_t  current_range_byte=0;
uint32_t  crc32=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int W25qxx_Init(void);
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t CalculateCRC32(uint32_t* inputData, uint32_t size)
{
	// CRC32 format is similar to CRC-32/MPEG-2
	// STM32F1 just do CRC32 without any polynomial settings
	// size must be : sizeof(inputData)/sizeof(uint32_t)
	uint32_t crcValue;
	///////////////Enable Hardware CRC unit//////////////////
  // Enable CRC clock
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
  // Reset CRC calculation unit
  LL_CRC_ResetCRCCalculationUnit(CRC);
  // Calculate CRC for the input data
  for (int i = 0; i < size; i++)
  {
    LL_CRC_FeedData32(CRC,inputData[i]);
  }
  // Get the calculated CRC value
  crcValue = LL_CRC_ReadData32(CRC);
	///////////////Disable Hardware CRC unit//////////////////
	LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CRC);
	
	return crcValue;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t tmp32;
	uint32_t crc32;
	uint32_t sectors;
	uint32_t Flash_Address;
	int i;
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
  //LL_GPIO_AF_DisableRemap_SWJ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
	LL_SPI_Enable(SPI_HANDLE);
  /* USER CODE BEGIN 2 */
	

	if(W25qxx_Init()== 1)
	{
		W25qxx_ReadSector(dataBuffer,settings_sector,0,w25qxx_PageSize);
		memcpy(&app_new_checksum,&dataBuffer[252],4); // check end double words of first page for new checksu
		//check checksum is not 0xFFFFFFFF to do not update again if it set high
		//if update finished , main program will set this bytes at first to prevent updating again 
		if(app_new_checksum!=0xFFFFFFFF)
		{		
			//read new firmware size
			memcpy(&app_new_size,&dataBuffer[248],4);
			//check size is valid
			if(app_new_size > 0 && app_new_size != 0xFFFFFFFF)
			{
				//calculate how many 4KB sectors to read
				sectors = (app_new_size / 4096);
				if(app_new_size%4096>0){sectors++;}
				sectors = sectors + 2; // add sector offset
				//
				
				//Enable & Reset CRC
				LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
				LL_CRC_ResetCRCCalculationUnit(CRC);
				
				for(tmp32 = 2;tmp32 < sectors;tmp32++)
				{
						//Read 4KB Sector
						W25qxx_ReadSector(dataBuffer,sectors,0,w25qxx_SectorSize);
						
						//calculate CRC
					  for (i = 0; i < sizeof(dataBuffer) / sizeof(uint32_t); i++)
						{
							LL_CRC_FeedData32(CRC,dataBuffer[i]);
						}
				}
				
				// Get the calculated CRC value
				crc32 = LL_CRC_ReadData32(CRC);
				///////////////Disable Hardware CRC unit//////////////////
				LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_CRC);		

				//compare calculated CRC with saved CRC in Settings Sector
				if(crc32 == app_new_checksum)
				{
					//Unlock the FLASH
					FEE_Init();
					
					//Erase the FLASH
					FLASH_Erase(Main_Program_Base_Address,Main_Program_End_Address);
					
					//do update with 1KB pages
					Flash_Address = Main_Program_Base_Address;
					for(tmp32 = 2;tmp32 < sectors;tmp32++)
					{
							//Read 4KB Sector
							W25qxx_ReadSector(dataBuffer,sectors,0,w25qxx_SectorSize);
							
							//Write on the Flash
							for (i = 0; i < 4; i++)
							{
								Flash_WriteMultiple(dataBuffer,Flash_Address,&dataBuffer[(i*1024)],1024);
								//check for exit in case of reach to firmware size
								Flash_Address+= 1024; 
								if((Flash_Address-Main_Program_Base_Address) >= app_new_size)
								{
									tmp32 = sectors + 1; // make a condition to exit first for loop
									break;
								}
							}
					}
					
					//Lock the Flash
					FLASH_Locked();
				}
			}
		}
	}

	// Jump into main code
	jump_to_start();	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
	
	// If Program has been written 
	if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
		{
			
			//Make sure, the CPU is in privileged mode.
			//if( CONTROL_nPRIV_Msk & __get_CONTROL( ) )
			//{  // not in privileged mode 
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
			{  // MSP is not active 
				__set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk );
			}
			
			//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
			
			// Set system control register SCR->VTOR  
			//SCB->VTOR = NVIC_VectTab_FLASH | ((ApplicationAddress & 0x0000FFFF) & (uint32_t)0x1FFFFF80);
			
			SCB->VTOR = ApplicationAddress & 0xFFFF;//check this
			
			//__DSB();
			//__ISB();
			
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			
			Jump_To_Application = (void (*)(void)) (*((uint32_t *) ((ApplicationAddress + 4))));
			Jump_To_Application(); 
		}
}


//###################### W25QXX NOR READ FUNCTIONS ############################################################################
int SPI_TRANSFER(uint8_t __command,uint8_t *__result)
{
	static uint16_t timeout=0;
	
	LL_SPI_TransmitData8(SPI_HANDLE, __command);
	while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
	
	*__result = LL_SPI_ReceiveData8(SPI_HANDLE);
	timeout = 0;
  while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE))
	{
		timeout++;
		if(timeout>=SPI_TIMEOUT){return -1;}
	}
	
	
	*__result = LL_SPI_ReceiveData8(SPI_HANDLE);
	return 0;
}


int W25Qxx_MultiByteReadSPI(uint8_t *buffer,uint32_t bytes, uint32_t offset, uint8_t command) {
    uint16_t timeout = 0;
		// Send Read command if needed
		if (command == Read_UniqueID || command == Read_JedecID) {
				LL_SPI_TransmitData8(SPI_HANDLE, command);
				timeout = 0;
				while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
						timeout++;
						if (timeout >= SPI_TIMEOUT) {
								// Return an appropriate error code or handle the timeout error
								return -1;
						}
				}
		}

		// Receive data bytes using SPI
		for (uint32_t i = 0; i < bytes; i++) {
				LL_SPI_TransmitData8(SPI_HANDLE, 0x00);
				timeout = 0;
				while (!LL_SPI_IsActiveFlag_TXE(SPI_HANDLE)) {
						timeout++;
						if (timeout >= SPI_TIMEOUT) {
								// Return an appropriate error code or handle the timeout error
								return -1;
						}
				}

				timeout = 0;
				while (!LL_SPI_IsActiveFlag_RXNE(SPI_HANDLE)) {
						timeout++;
						if (timeout >= SPI_TIMEOUT) {
								// Return an appropriate error code or handle the timeout error
								return -1;
						}
				}

				buffer[offset + i] = LL_SPI_ReceiveData8(SPI_HANDLE);
		}
    

    // Wait for the SPI transmission to complete
    timeout = 0;
    while (LL_SPI_IsActiveFlag_BSY(SPI_HANDLE)) {
        timeout++;
        if (timeout >= SPI_TIMEOUT) {
            // Return an appropriate error code or handle the timeout error
            return -1;
        }
    }

    return 0;
}

int W25Qxx_ReadSPI(uint8_t command,uint8_t *buffer, int32_t address, uint16_t bytes, uint32_t offset)
{
    int result = 0;
    uint8_t spiResponse = 0;

    LL_GPIO_ResetOutputPin(SPI_CS_PORT, SPI_CS_PIN);

    result = SPI_TRANSFER(command, &spiResponse);
    if (result == -1)
    {
        LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
        return -1; // Timeout Error
    }

    if (address >= 0)
    {
        for (int i = 3; i > 0; i--)
        {
            result = SPI_TRANSFER((address >> (8 * i)) & 0xFF, &spiResponse);
            if (result == -1)
            {
                LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
                return -1; // Timeout Error
            }
        }
    }
		
    if (command == FastRead)
    {
        for (int i = 1; i > 0; i--)
        {
            result = SPI_TRANSFER(0, &spiResponse);
            if (result == -1)
            {
                LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
                return -1; // Timeout Error
            }
        }
    }
		
    if (bytes > 1)
    {
				#ifdef W25QXX_USE_DMA
        result = W25Qxx_TransferDMASPI(buffer, bytes, dir, offset, command);
				#else
				result = W25Qxx_MultiByteReadSPI(buffer,bytes, offset, command);
				#endif
			
        if (result == -1)
        {
            LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
            return -1; // Timeout Error
        }
    }
    else if (bytes == 1)
    {
        result = SPI_TRANSFER(buffer[0], &spiResponse);
        if (result == -1)
        {
            LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
            return -1; // Timeout Error
        }
    }

    LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
    return spiResponse;
}

uint32_t W25qxx_ReadID(void)
{
	static int result,retry;
	uint32_t Temp = 0xFFFFFFFF;
	while(retry<SPI_RETRY)
	{
		retry++;	
		result = W25Qxx_ReadSPI(Read_JedecID,dataBuffer,-1,4,0);
		if(result != -1)
		{		
			Temp = (dataBuffer[0] << 16) | (dataBuffer[1] << 8) | dataBuffer[2];
			break;
		}
		W25qxx_Delay(1);
	}
	return Temp;
}

int W25qxx_Init(void)
{
	w25qxx_Lock = 1;
	
	LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
	W25qxx_Delay(1);
	
	if(W25qxx_ReadID() == 0xFFFFFFFF) // W25Q64 No responded
	{		
			w25qxx_Lock = 0;
			return 0;
	}	
	w25qxx_Lock = 0;
	return 1;
}

void W25qxx_WaitForWriteEnd(void)
{
	int result,retry;
	do
	{
		retry = 0;
		while(retry<SPI_RETRY)
		{		
			retry++;
			result = W25Qxx_ReadSPI(Read_StatusRegister_1,dataBuffer,-1,1,0);
			if(result!=-1){break;}
		}		
		//W25qxx_Delay(100);
	} while ((result & 0x01) == 0x01);
}

uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
	return (SectorAddress * w25qxx_SectorSize) / w25qxx_PageSize;
}

void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
	while (w25qxx_Lock == 1)
	W25qxx_Delay(1);
	w25qxx_Lock = 1;
	if ((NumByteToRead_up_to_PageSize > w25qxx_PageSize) || (NumByteToRead_up_to_PageSize == 0))
		NumByteToRead_up_to_PageSize = w25qxx_PageSize;
	if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx_PageSize)
		NumByteToRead_up_to_PageSize = w25qxx_PageSize - OffsetInByte;

	W25qxx_WaitForWriteEnd();
	Page_Address = Page_Address * w25qxx_PageSize + OffsetInByte;
	W25Qxx_ReadSPI(FastRead,pBuffer,Page_Address,NumByteToRead_up_to_PageSize,0);
	
	w25qxx_Lock = 0;
}
	
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
	if ((NumByteToRead_up_to_SectorSize > w25qxx_SectorSize) || (NumByteToRead_up_to_SectorSize == 0))
		NumByteToRead_up_to_SectorSize = w25qxx_SectorSize;

	if (OffsetInByte >= w25qxx_SectorSize)
	{
		return;
	}
	
	uint32_t StartPage;
	int32_t BytesToRead;
	uint32_t LocalOffset;
	if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx_SectorSize)
		BytesToRead = w25qxx_SectorSize - OffsetInByte;
	else
		BytesToRead = NumByteToRead_up_to_SectorSize;
	StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx_PageSize);
	LocalOffset = OffsetInByte % w25qxx_PageSize;
	do
	{
		W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, w25qxx_PageSize);//BytesToRead);
		StartPage++;
		BytesToRead -= w25qxx_PageSize - LocalOffset;
		pBuffer += w25qxx_PageSize - LocalOffset;
		LocalOffset = 0;
	} while (BytesToRead > 0);
}
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
