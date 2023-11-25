
#include <Flash.h>
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"

#define FLASHWORD		8

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  /* BANK 1 */
  if((Address >= 0x08000000) && (Address < 0x08020000))
  {
    sector = FLASH_SECTOR_0;
  }

  else if((Address >= 0x08020000) && (Address < 0x08040000))
  {
    sector = FLASH_SECTOR_1;
  }

  else if((Address >= 0x08040000) && (Address < 0x08060000))
  {
    sector = FLASH_SECTOR_2;
  }

  else if((Address >= 0x08060000) && (Address < 0x08080000))
  {
    sector = FLASH_SECTOR_3;
  }

  else if((Address >= 0x08080000) && (Address < 0x080A0000))
  {
    sector = FLASH_SECTOR_4;
  }

  else if((Address >= 0x080A0000) && (Address < 0x080C0000))
  {
    sector = FLASH_SECTOR_5;
  }

  else if((Address >= 0x080C0000) && (Address < 0x080E0000))
  {
    sector = FLASH_SECTOR_6;
  }

  else if((Address >= 0x080E0000) && (Address < 0x08100000))
  {
    sector = FLASH_SECTOR_7;
  }


  /* BANK 2 */
  if((Address >= 0x08100000) && (Address < 0x08120000))
  {
    sector = FLASH_SECTOR_0;
  }

  else if((Address >= 0x08120000) && (Address < 0x08140000))
  {
    sector = FLASH_SECTOR_1;
  }

  else if((Address >= 0x08140000) && (Address < 0x08160000))
  {
    sector = FLASH_SECTOR_2;
  }

  else if((Address >= 0x08160000) && (Address < 0x08180000))
  {
    sector = FLASH_SECTOR_3;
  }

  else if((Address >= 0x08180000) && (Address < 0x081A0000))
  {
    sector = FLASH_SECTOR_4;
  }

  else if((Address >= 0x081A0000) && (Address < 0x081C0000))
  {
    sector = FLASH_SECTOR_5;
  }

  else if((Address >= 0x081C0000) && (Address < 0x081E0000))
  {
    sector = FLASH_SECTOR_6;
  }

  else if((Address >= 0x081E0000) && (Address < 0x08200000))
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}

uint8_t bytes_temp[4];

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar=0;

	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	  uint32_t StartSector = GetSector(StartSectorAddress);
	  uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
	  uint32_t EndSector = GetSector(EndSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;

	  // The the proper BANK to erase the Sector
	  if (StartSectorAddress < 0x08100000)
		  EraseInitStruct.Banks     = 0x01U;
	  else EraseInitStruct.Banks    = 0x02U;

	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;


	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
		  return HAL_FLASH_GetError ();
	  }

	  /* Program the user Flash area 8 WORDS at a time
	   * (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, StartSectorAddress, (uint32_t ) &data[sofar]) == HAL_OK)
	     {
	    	 StartSectorAddress += 4*FLASHWORD;  //
	    	 sofar+=FLASHWORD;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}


void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *data, uint16_t numberofwords)
{
	while (1)
	{

		*data = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		data++;
		if (!(numberofwords--)) break;
	}
}

void Convert_To_Str (uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data)/4) + ((strlen((char *)Data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8*(i%4));
	}
}


void Flash_Write_NUM (uint32_t StartSectorAddress, float Num)
{

	float2Bytes(bytes_temp, Num);

	Flash_Write_Data (StartSectorAddress, (uint32_t *)bytes_temp, 1);
}


float Flash_Read_NUM (uint32_t StartSectorAddress)
{
	uint8_t buffer[4];
	float value;

	Flash_Read_Data(StartSectorAddress, (uint32_t *)buffer, 1);
	value = Bytes2float(buffer);
	return value;
}
