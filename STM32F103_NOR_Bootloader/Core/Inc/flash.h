//-----------------------------------------------------------------------------
// Copyright:  TESLA ELECTRONIC COMPANY    
// Author:  Mohammad Ali Nekouie       
// Remarks:  M.A.N.      
// known Problems: !!!
// Version:  0.1     
// Description:  This Lib create for FLASH AS EEPROM
//-----------------------------------------------------------------------------
	

#ifndef UC_MEMORY_H
#define UC_MEMORY_H

	
	
	
	// CAN BE CHANGED
	#define FLASH_DENSITY_PAGES				1	      // how many pages are used 
	#define FLASH_PAGE_SIZE						1024	    // can be 1k or 2k check manual for used device
	#define FLASH_PAGE_BASE_ADDRESS 	0x08000000  // choose last location for the first EEPROMPage address on the top of flash
//////////////Flash Space Definitions////////////////////////
//BANK1
	#define Flash_uppper_half_end	 	0x0801FFFF
	#define Flash_uppper_half_start	0x08010800
	#define Flash_lower_half_end		0x080107FF
	#define Flash_lower_half_start	0x08001000
	#define Flash_BOOTLOADER_end		0x08000FFF
	#define Flash_BOOTLOADER_start	0x08000000

	// DONT CHANGE
	#define FLASH_DENSITY_BYTES				((FLASH_PAGE_SIZE / 2) * FLASH_DENSITY_PAGES - 1)
	#define FLASH_LAST_PAGE_ADDRESS 	(FLASH_PAGE_BASE_ADDRESS + (FLASH_PAGE_SIZE * FLASH_DENSITY_PAGES))
	#define FLASH_EMPTY_WORD					((uint16_t)0xFFFF)

	// use this function to initialize the functionality

	void FLASH_Locked(void);
	uint16_t FEE_Init(void);
	void FEE_Erase (void);
	uint8_t FEE_ReadDataByte (uint32_t Address);
  uint8_t FEE_ReadDataByte_abs (uint32_t BASE_ADDRESS);
	void FEE_ReadMultiple (uint32_t Address,uint8_t * Dest,uint16_t len);
	uint16_t FEE_WriteMultiple(uint32_t Address,uint8_t * Value,uint16_t len);
	void FEE_LoadPageThenChange(char * Buffer,uint16_t Buffer_length,uint16_t var_Address,char * Value,uint16_t var_Lenght);
	void Page_Copy(uint8_t *DataBuf,uint32_t dest_Page_start_address,uint32_t source_Page_start_address,uint32_t source_Page_end_address);
	uint16_t Flash_WriteMultiple(uint8_t *DataBuf,uint32_t Address,uint8_t * Value,uint16_t len);
	void FLASH_Erase (uint32_t start_address,uint32_t end_address);
	
#endif



