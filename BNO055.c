/*
 * BNO055.c
 *
 *  Created on: Jun 22, 2023
 *      Author: Andrei
 */

#include "BNO055.h"
#include <stdio.h>
#include <string.h>
#include "Flash.h"
/*
 * Scriere Registri
 */
void BNO055_Write_Reg(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart, uint8_t Register_Address, uint8_t* Transmitted_Data, uint8_t Module_Address,uint8_t Numar_Bytes)
{
	HAL_StatusTypeDef Error;
	uint8_t buf[12];
	Error = HAL_I2C_Mem_Write(hi2c,(Module_Address << 1),Register_Address,1,Transmitted_Data,Numar_Bytes,HAL_MAX_DELAY);
	if (Error != HAL_OK)
		{
			strcpy((char*)buf,"Error! \n\r");
			HAL_Delay(500);
			HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
		}
}

/*
 * Citire Registri
 */
void BNO055_Read_Reg(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Register_Address, uint8_t* Valoare_Registru,uint8_t Module_Address, uint8_t Numar_Bytes) // Citire dintr-un anumit registru
{
	HAL_StatusTypeDef Error;
	uint8_t buf[12];
	Error = HAL_I2C_Mem_Read(hi2c,(Module_Address << 1),Register_Address,1,Valoare_Registru,Numar_Bytes,HAL_MAX_DELAY);
	if (Error != HAL_OK)
		{
			strcpy((char*)buf,"Error! \n\r");
			HAL_Delay(500);
			HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
		}
}

/*
 * Setare Pagina(Bank-uri)
 */
void BNO055_Set_Page(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Register_Address, uint8_t Page,uint8_t Module_Address)
{
	BNO055_Write_Reg(hi2c,huart,Register_Address,&Page,Module_Address,1);
}

/*
 * Testare Comunicatie cu Senzorul si Citire Registri de Tip ID
 */
void BNO055_Test(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address) // Functie testare valoarea registrului WHO_AM_I
{
	// Setare Pagina 0
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	// Testare Valoare CHIP_ID
	HAL_StatusTypeDef Error;
	uint8_t buf[34];
	uint8_t Valoare_Registru = BNO055_CHIP_ID;
	Error = HAL_I2C_Master_Transmit(hi2c,(Module_Address << 1),&Valoare_Registru,sizeof(Valoare_Registru),HAL_MAX_DELAY);
	if (Error != HAL_OK)
	{
		strcpy((char*)buf,"Error Tx!\n\r");
	}
	else
	{
			Error = HAL_I2C_Master_Receive(hi2c,(Module_Address << 1),&Valoare_Registru,1,HAL_MAX_DELAY);
			if(Error != HAL_OK)
				strcpy((char*)buf,"Error Rx!\n\r");
	}
	if(Valoare_Registru != CHIP_ID)
	{
		strcpy((char*)buf,"Eroare Transmisie CHIP!\n\r");
		HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
	}

	// Testare Valoare ACC_ID
	Valoare_Registru = BNO055_ACC_ID;
	Error = HAL_I2C_Master_Transmit(hi2c,(Module_Address << 1),&Valoare_Registru,sizeof(Valoare_Registru),HAL_MAX_DELAY);
		if (Error != HAL_OK)
		{
			strcpy((char*)buf,"Error Tx!\n\r");
		}
		else
		{
				Error = HAL_I2C_Master_Receive(hi2c,(Module_Address << 1),&Valoare_Registru,1,HAL_MAX_DELAY);
				if(Error != HAL_OK)
					strcpy((char*)buf,"Error Rx!\n\r");
		}
		if(Valoare_Registru != ACC_ID)
		{
			strcpy((char*)buf,"Eroare Transmisie ACC!\n\r");
			HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
		}

	// Testare Valoare MAG_ID
	Valoare_Registru = BNO055_MAG_ID;
	Error = HAL_I2C_Master_Transmit(hi2c,(Module_Address << 1),&Valoare_Registru,sizeof(Valoare_Registru),HAL_MAX_DELAY);
		if (Error != HAL_OK)
		{
		strcpy((char*)buf,"Error Tx!\n\r");
		}
		else
		{
				Error = HAL_I2C_Master_Receive(hi2c,(Module_Address << 1),&Valoare_Registru,1,HAL_MAX_DELAY);
				if(Error != HAL_OK)
				strcpy((char*)buf,"Error Rx!\n\r");
		}
		if(Valoare_Registru != MAG_ID)
		{
			strcpy((char*)buf,"Eroare Transmisie MAG!\n\r");
			HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
		}

	// Testare Valoare GYR_ID
	Valoare_Registru = BNO055_GYR_ID;
	Error = HAL_I2C_Master_Transmit(hi2c,(Module_Address << 1),&Valoare_Registru,sizeof(Valoare_Registru),HAL_MAX_DELAY);
		if (Error != HAL_OK)
		{
			strcpy((char*)buf,"Error Tx!\n\r");
		}
		else
		{
			Error = HAL_I2C_Master_Receive(hi2c,(Module_Address << 1),&Valoare_Registru,1,HAL_MAX_DELAY);
			if(Error != HAL_OK)
			strcpy((char*)buf,"Error Rx!\n\r");
		}
		if(Valoare_Registru != GYR_ID)
		{
			strcpy((char*)buf,"Eroare Transmisie GYR!\n\r");
			HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
		}

	// Transmisie Mesaj Final
	strcpy((char*)buf,"Testarea a avut loc cu succes!\r\n");
	HAL_UART_Transmit(huart,buf,strlen((char*)buf),HAL_MAX_DELAY);
}

/*
 * Citire Mod de Operare
 */
int BNO055_Get_Operation_Mode(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address)
{
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);	// Comutam pe pagina 0 de memorie;
	uint8_t Mod_Operare = 0;										// Initializam o variabila mod_operare;
	BNO055_Read_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_Operare,Module_Address,1);	// Verificam ce mod de operare prezinta senzorul.
	return Mod_Operare;
}

/*
 * Setare Mod Nou de Operare
 */
void BNO055_Set_Operation_Mode(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,uint8_t Mod_Nou_Operare)
{
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);	// Comutam pe pagina 0 de memorie;
	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_CONFIG = CONFIGMODE;
	if(Mod_Operare != CONFIGMODE)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_CONFIG,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(7);
	}
	BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_Nou_Operare,Module_Address,1); // Configurarea senzorului in nou mod de operare;
	HAL_Delay(19);
}

/*
 * Initializare Senzor
 */
void BNO055_Initialise(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address)
{
	// Setare Pagina 0;
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	// Setare Mod Configurare NDOF (Fuzionare cu 9 grade de libertate)
	uint8_t Mod_Configurare_NDOF = NDOF;
	BNO055_Set_Operation_Mode(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,Mod_Configurare_NDOF);

	// Setare Mod Configurare Power
	uint8_t Mod_Configurare_Power = NORMAL_MODE;
	BNO055_Write_Reg(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_PWR_MODE,&Mod_Configurare_Power,BNO055_MODULE_1_ADDRESS,1);

	// ADAUGA SI IESIREA DIN MODUL SLEEP!!! DOAR PENTRU A FI PRECAUTI.
}


/*
 * Citire Date Cuaternioni
 */
void BNO055_Read_Quaternion_W(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_NDOF = NDOF;
	if(Mod_Operare != Mod_NDOF)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(19);
	}

	uint8_t Buffer[2];
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_W_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_W_MSB,&Buffer[1],BNO055_MODULE_1_ADDRESS,1);

	int16_t w_quat = 0;
	const double scale = (1.0/(1<<14));	// Echivalent cu 1/(2^14)

	w_quat = (((int16_t)Buffer[0]) | ((int16_t)Buffer[1]<<8));
	IMU_Data -> w = w_quat*scale;
}

void BNO055_Read_Quaternion_X(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_NDOF = NDOF;
	if(Mod_Operare != Mod_NDOF)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(19);
	}

	uint8_t Buffer[2];
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_X_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_X_MSB,&Buffer[1],BNO055_MODULE_1_ADDRESS,1);

	int16_t x_quat = 0;
	const double scale = (1.0/(1<<14));	// Echivalent cu 1/(2^14)

	x_quat = (((int16_t)Buffer[0]) | ((int16_t)Buffer[1]<<8));
	IMU_Data -> x = x_quat*scale;
}

void BNO055_Read_Quaternion_Y(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_NDOF = NDOF;
	if(Mod_Operare != Mod_NDOF)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(19);
	}

	uint8_t Buffer[2];
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Y_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Y_MSB,&Buffer[1],BNO055_MODULE_1_ADDRESS,1);

	int16_t y_quat = 0;
	const double scale = (1.0/(1<<14));	// Echivalent cu 1/(2^14)

	y_quat = (((int16_t)Buffer[0]) | ((int16_t)Buffer[1]<<8));
	IMU_Data -> y = y_quat*scale;
}

void BNO055_Read_Quaternion_Z(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_NDOF = NDOF;
	if(Mod_Operare != Mod_NDOF)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(19);
	}

	uint8_t Buffer[2];
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Z_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Z_MSB,&Buffer[1],BNO055_MODULE_1_ADDRESS,1);

	int16_t z_quat = 0;
	const double scale = (1.0/(1<<14));	// Echivalent cu 1/(2^14)

	z_quat = (((int16_t)Buffer[0]) | ((int16_t)Buffer[1]<<8));
	IMU_Data -> z = z_quat*scale;
}

void BNO055_Read_Quaternion_WXYZ(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	// Setare Pagina 0;
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_NDOF = NDOF;
	if(Mod_Operare != Mod_NDOF)	// Daca nu este in config, pentru a seta alt mod de operare, trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(19);
	}

	uint8_t Buffer[8];		// Avem 4 cuaternioni, cate 2 bytes pentru fiecare, deci vom incarca toate datele intr-un buffer cu 8 bytes

	// Citim datele din fiecare registru in parte:
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_W_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_W_MSB,&Buffer[1],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_X_LSB,&Buffer[2],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_X_MSB,&Buffer[3],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Y_LSB,&Buffer[4],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Y_MSB,&Buffer[5],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Z_LSB,&Buffer[6],BNO055_MODULE_1_ADDRESS,1);
	BNO055_Read_Reg(hi2c,huart,BNO055_QUA_DATA_Z_MSB,&Buffer[7],BNO055_MODULE_1_ADDRESS,1);

	// Le vom salva in variabile pe 16 biti, pe care le vom crea, deplasand cu 8 biti la stanga MSB-ul
	int16_t w_quat,x_quat,y_quat,z_quat;
	w_quat = x_quat = y_quat = z_quat = 0;

	w_quat = (((uint16_t)Buffer[1])<<8 | ((uint16_t)Buffer[0]));
	x_quat = (((uint16_t)Buffer[3])<<8 | ((uint16_t)Buffer[2]));
	y_quat = (((uint16_t)Buffer[5])<<8 | ((uint16_t)Buffer[4]));
	z_quat = (((uint16_t)Buffer[7])<<8 | ((uint16_t)Buffer[6]));

	const double scale = (1.0/(1<<14));	// Echivalent cu 1/(2^14)
	// Pentru a transforma dintr-un numar pe 16 biti in cuaternioni, trebuie sa impartim valoarea la 2^14, conform DATASHEET PG.35
	IMU_Data -> w = w_quat*scale;
	IMU_Data -> x = x_quat*scale;
	IMU_Data -> y = y_quat*scale;
	IMU_Data -> z = z_quat*scale;
}

void BNO055_Convert_Quat_to_Euler(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	float pi = 3.14159265359;
	IMU_Data -> Roll = (atan2(2*((IMU_Data -> w)*(IMU_Data -> x)+(IMU_Data -> y)*(IMU_Data -> z)),(1-2*((IMU_Data -> x)*(IMU_Data -> x)+(IMU_Data -> y)*(IMU_Data -> y)))))*180/pi;
	IMU_Data -> Pitch = (asin(2*((IMU_Data -> w)*(IMU_Data -> y)-(IMU_Data -> x)*(IMU_Data -> z))))*180/pi;
	IMU_Data -> Yaw = (atan2(2*((IMU_Data -> w)*(IMU_Data -> z)+(IMU_Data -> x)*(IMU_Data -> y)),(1-2*((IMU_Data -> y)*(IMU_Data -> y)+(IMU_Data -> z)*(IMU_Data -> z)))))*180/pi;
}

/*
 * Citire Date Calibrare
 */

void BNO055_Read_Calib_Stat(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	// Setare Pagina 0;
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	IMU_Data -> cal_state_sys = IMU_Data -> cal_state_mag = IMU_Data -> cal_state_gyr = IMU_Data -> cal_state_acc = 0;

	uint8_t Buffer;
	BNO055_Read_Reg(hi2c,huart,BNO055_CALIB_STAT,&Buffer,BNO055_MODULE_1_ADDRESS,1);

	IMU_Data -> cal_state_sys = (Buffer >> 6) & 0x03;
	IMU_Data -> cal_state_gyr = (Buffer >> 4) & 0x03;
	IMU_Data -> cal_state_acc = (Buffer >> 2) & 0x03;
	IMU_Data -> cal_state_mag = Buffer & 0x03;
}

void BNO055_Read_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	// Setare Pagina 0;
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_CONFIG = CONFIGMODE;
	if(Mod_Operare != CONFIGMODE)	// Trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_CONFIG,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(7);
	}

	uint8_t Buffer[22];
	BNO055_Read_Reg(hi2c,huart,BNO055_ACC_OFFSET_X_LSB,&Buffer[0],BNO055_MODULE_1_ADDRESS,22);

  	IMU_Data -> cal_offset_x_acc = (((uint16_t)Buffer[1]) << 8) | ((uint16_t)Buffer[0]);
	IMU_Data -> cal_offset_y_acc = (((uint16_t)Buffer[3]) << 8) | ((uint16_t)Buffer[2]);
	IMU_Data -> cal_offset_z_acc = (((uint16_t)Buffer[5]) << 8) | ((uint16_t)Buffer[4]);
	IMU_Data -> cal_offset_x_mag = (((uint16_t)Buffer[7]) << 8) | ((uint16_t)Buffer[6]);
	IMU_Data -> cal_offset_y_mag = (((uint16_t)Buffer[9]) << 8) | ((uint16_t)Buffer[8]);
	IMU_Data -> cal_offset_z_mag = (((uint16_t)Buffer[11]) << 8) | ((uint16_t)Buffer[10]);
	IMU_Data -> cal_offset_x_gyr = (((uint16_t)Buffer[13]) << 8) | ((uint16_t)Buffer[12]);
	IMU_Data -> cal_offset_y_gyr = (((uint16_t)Buffer[15]) << 8) | ((uint16_t)Buffer[14]);
	IMU_Data -> cal_offset_z_gyr = (((uint16_t)Buffer[17]) << 8) | ((uint16_t)Buffer[16]);
	IMU_Data -> cal_radius_acc = (((uint16_t)Buffer[19]) << 8) | ((uint16_t)Buffer[18]);
	IMU_Data -> cal_radius_mag = (((uint16_t)Buffer[21]) << 8) | ((uint16_t)Buffer[20]);

	for(int i=0;i<=21;i++)
	{
		IMU_Data -> cal_buffer[i] = Buffer[i];
	}
}

void BNO055_Set_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	// Setare Pagina 0;
	BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

	uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
	uint8_t Mod_CONFIG = CONFIGMODE;
	if(Mod_Operare != CONFIGMODE)	// Trebuie sa fie in config intai;
	{
		BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_CONFIG,Module_Address,1);	// Configuram senzorul in modul CONFIG;
		HAL_Delay(7);
	}

	BNO055_Write_Reg(hi2c,huart,BNO055_ACC_OFFSET_X_LSB,&(IMU_Data -> cal_buffer[0]),Module_Address,22);
	uint8_t Mod_NDOF = NDOF;
	BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_NDOF,Module_Address,1);
	HAL_Delay(19);
}

/*
 * Transfer Date Registru Calibrare
 */

// Aceasta Functie Incarca in Struct Datele Primite de la Memoria Flash.
extern uint32_t Date_Memorie_Flash[11];
void BNO055_Read_Calib_Data_From_Flash(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	IMU_Data -> cal_offset_x_mag_flash = Date_Memorie_Flash[0];
	IMU_Data -> cal_offset_y_mag_flash = Date_Memorie_Flash[1];
	IMU_Data -> cal_offset_z_mag_flash = Date_Memorie_Flash[2];
	IMU_Data -> cal_offset_x_gyr_flash = Date_Memorie_Flash[3];
	IMU_Data -> cal_offset_y_gyr_flash = Date_Memorie_Flash[4];
	IMU_Data -> cal_offset_z_gyr_flash = Date_Memorie_Flash[5];
	IMU_Data -> cal_offset_x_acc_flash = Date_Memorie_Flash[6];
	IMU_Data -> cal_offset_y_acc_flash = Date_Memorie_Flash[7];
	IMU_Data -> cal_offset_z_acc_flash = Date_Memorie_Flash[8];
	IMU_Data -> cal_radius_acc_flash = Date_Memorie_Flash[9];
	IMU_Data -> cal_radius_mag_flash = Date_Memorie_Flash[10];

}

// Aceasta Functie Incarca Datele Calibrarii in Senzor.
void BNO055_Transfer_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data)
{
	// Setare Pagina 0;
		BNO055_Set_Page(hi2c,huart,BNO055_PAGE_ID,0,Module_Address);

		uint8_t Mod_Operare = BNO055_Get_Operation_Mode(hi2c,huart,Module_Address);	// Obtinem modul actual de operare al senzorului.
		uint8_t Mod_CONFIG = CONFIGMODE;
		if(Mod_Operare != CONFIGMODE)	// Trebuie sa fie in config intai;
		{
			BNO055_Write_Reg(hi2c,huart,BNO055_OPR_MODE,&Mod_CONFIG,Module_Address,1);	// Configuram senzorul in modul CONFIG;
			HAL_Delay(7);
		}

		BNO055_Write_Reg(hi2c,huart,BNO055_ACC_OFFSET_X_LSB,&(IMU_Data -> cal_offset_x_acc_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_ACC_OFFSET_Y_LSB,&(IMU_Data -> cal_offset_y_acc_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_ACC_OFFSET_Z_LSB,&(IMU_Data -> cal_offset_z_acc_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_MAG_OFFSET_X_LSB,&(IMU_Data -> cal_offset_x_mag_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_MAG_OFFSET_Y_LSB,&(IMU_Data -> cal_offset_y_mag_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_MAG_OFFSET_Z_LSB,&(IMU_Data -> cal_offset_z_mag_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_GYR_OFFSET_X_LSB,&(IMU_Data -> cal_offset_x_gyr_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_GYR_OFFSET_Y_LSB,&(IMU_Data -> cal_offset_y_gyr_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_GYR_OFFSET_Z_LSB,&(IMU_Data -> cal_offset_z_gyr_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_ACC_RADIUS_LSB,&(IMU_Data -> cal_radius_acc_flash),Module_Address,2);
		BNO055_Write_Reg(hi2c,huart,BNO055_MAG_RADIUS_LSB,&(IMU_Data -> cal_radius_mag_flash),Module_Address,2);
}
