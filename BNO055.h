/*
 * BNO055.h
 *
 *  Created on: Jun 22, 2023
 *      Author: Andrei
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stm32h7xx_hal.h"

// DEFINIRE ADRESE - PAGINA 0
#define BNO055_MODULE_1_ADDRESS		0x28
#define BNO055_MODULE_2_ADDRESS		0x29
#define BNO055_MODULE_3_ADDRESS		0x28
#define BNO055_MODULE_4_ADDRESS		0x29

#define BNO055_CHIP_ID				0x00
#define BNO055_ACC_ID				0x01
#define BNO055_MAG_ID				0x02
#define BNO055_GYR_ID				0x03
#define BNO055_SW_REV_ID_L_SB		0x04
#define BNO055_SW_REV_ID_M_SB		0x05
#define BNO055_PAGE_ID				0x07
#define BNO055_ACC_DATA_X_LSB		0x08
#define BNO055_ACC_DATA_X_MSB		0x09
#define BNO055_ACC_DATA_Y_LSB		0x0A
#define BNO055_ACC_DATA_Y_MSB		0x0B
#define BNO055_ACC_DATA_Z_LSB		0x0C
#define BNO055_ACC_DATA_Z_MSB		0x0D
#define BNO055_MAG_DATA_X_LSB		0x0E
#define BNO055_MAG_DATA_X_MSB		0x0F
#define BNO055_MAG_DATA_Y_LSB		0x10
#define BNO055_MAG_DATA_Y_MSB		0x11
#define BNO055_MAG_DATA_Z_LSB		0x12
#define BNO055_MAG_DATA_Z_MSB		0x13
#define BNO055_GYR_DATA_X_LSB		0x14
#define BNO055_GYR_DATA_X_MSB		0x15
#define BNO055_GYR_DATA_Y_LSB		0x16
#define BNO055_GYR_DATA_Y_MSB		0x17
#define BNO055_GYR_DATA_Z_LSB		0x18
#define BNO055_GYR_DATA_Z_MSB		0x19
#define BNO055_EUL_HEADING_LSB		0x1A
#define BNO055_EUL_HEADING_MSB		0x1B
#define BNO055_EUL_ROLL_LSB			0x1C
#define BNO055_EUL_ROLL_MSB			0x1D
#define BNO055_EUL_PITCH_LSB		0x1E
#define BNO055_EUL_PITCH_MSB		0x1F
#define BNO055_QUA_DATA_W_LSB		0x20
#define BNO055_QUA_DATA_W_MSB		0x21
#define BNO055_QUA_DATA_X_LSB		0x22
#define BNO055_QUA_DATA_X_MSB		0x23
#define BNO055_QUA_DATA_Y_LSB		0x24
#define BNO055_QUA_DATA_Y_MSB		0x25
#define BNO055_QUA_DATA_Z_LSB		0x26
#define BNO055_QUA_DATA_Z_MSB		0x27
#define BNO055_LIA_DATA_X_LSB		0x28
#define BNO055_LIA_DATA_X_MSB		0x29
#define BNO055_LIA_DATA_Y_LSB		0x2A
#define BNO055_LIA_DATA_Y_MSB		0x2B
#define BNO055_LIA_DATA_Z_LSB		0x2C
#define BNO055_LIA_DATA_Z_MSB		0x2D
#define BNO055_GRV_DATA_X_LSB		0x2E
#define BNO055_GRV_DATA_X_MSB		0x2F
#define BNO055_GRV_DATA_Y_LSB		0x30
#define BNO055_GRV_DATA_Y_MSB		0x31
#define BNO055_GRV_DATA_Z_LSB		0x32
#define BNO055_GRV_DATA_Z_MSB		0x33
#define BNO055_TEMP					0x34
#define BNO055_CALIB_STAT			0x35
#define BNO055_ST_RESULT			0x36
#define BNO055_INT_STA				0x37
#define BNO055_SYS_CLK_STATUS		0x38
#define BNO055_SYS_STATUS			0x39
#define BNO055_SYS_ERR				0x3A
#define BNO055_UNIT_SEL				0x3B
#define BNO055_OPR_MODE				0x3D
#define BNO055_PWR_MODE				0x3E
#define BNO055_SYS_TRIGGER			0x3F
#define BNO055_TEMP_SOURCE			0x40
#define BNO055_AXIS_MAP_CONFIG		0x41
#define BNO055_AXIS_MAP_SIGN		0x42
#define BNO055_ACC_OFFSET_X_LSB		0x55
#define BNO055_ACC_OFFSET_X_MSB		0x56
#define BNO055_ACC_OFFSET_Y_LSB		0x57
#define BNO055_ACC_OFFSET_Y_MSB		0x58
#define BNO055_ACC_OFFSET_Z_LSB		0x59
#define BNO055_ACC_OFFSET_Z_MSB		0x5A
#define BNO055_MAG_OFFSET_X_LSB		0x5B
#define BNO055_MAG_OFFSET_X_MSB		0x5C
#define BNO055_MAG_OFFSET_Y_LSB		0x5D
#define BNO055_MAG_OFFSET_Y_MSB		0x5E
#define BNO055_MAG_OFFSET_Z_LSB		0x5F
#define BNO055_MAG_OFFSET_Z_MSB		0x60
#define BNO055_GYR_OFFSET_X_LSB		0x61
#define BNO055_GYR_OFFSET_X_MSB		0x62
#define BNO055_GYR_OFFSET_Y_LSB		0x63
#define BNO055_GYR_OFFSET_Y_MSB		0x64
#define BNO055_GYR_OFFSET_Z_LSB		0x65
#define BNO055_GYR_OFFSET_Z_MSB		0x66
#define BNO055_ACC_RADIUS_LSB		0x67
#define BNO055_ACC_RADIUS_MSB		0x68
#define BNO055_MAG_RADIUS_LSB		0x69
#define BNO055_MAG_RADIUS_MSB		0x6A

// DEFINIRE ADRESE - PAGINA 1
#define BNO055_PAGE_ID				0x07
#define BNO055_ACC_CONFIG			0x08
#define BNO055_MAG_CONFIG			0x09
#define BNO055_GYR_CONFIG_0			0x0A
#define BNO055_GYR_CONFIG_1			0x0B
#define BNO055_ACC_SLEEP_CONFIG		0x0C
#define BNO055_GYR_SLEEP_CONFIG		0x0D
#define BNO055_INT_MSK				0x0F
#define BNO055_INT_EN				0x10
#define BNO055_ACC_AM_THRES			0x11
#define BNO055_ACC_INT_SETTINGS		0x12
#define BNO055_ACC_HG_DURATION		0x13
#define	BNO055_ACC_HG_THRES			0x14
#define BNO055_ACC_NM_THRES			0x15
#define BNO055_ACC_NM_SET			0x16
#define BNO055_GYR_INT_SETTING		0x17
#define BNO055_GYR_HR_X_SET			0x18
#define BNO055_GYR_DUR_X			0x19
#define BNO055_GYR_HR_Y_SET			0x1A
#define BNO055_GYR_DUR_Y			0x1B
#define BNO055_GYR_HR_Z_SET			0x1C
#define BNO055_GYR_DUR_Z			0x1D
#define BNO055_GYR_AM_THRES			0x1E
#define BNO055_GYR_AM_SET			0x1F

/*
 * DEFINIRE VALORI CONSTANTE
 */

// Valori Registrii ID
#define CHIP_ID						0xA0
#define ACC_ID						0xFB
#define MAG_ID						0x32
#define GYR_ID						0x0F
// Valori Configurare Mod Operare - NON FUSION
#define CONFIGMODE 					0x00	// Mod Configurare
#define ACCONLY 					0x01	// Functioneaza doar Accelerometrul
#define MAGONLY 					0x02	// Functioneaza doar Magnetometrul
#define GYROONLY 					0x03	// Functioneaza doar Giroscopul
#define ACCMAG	 					0x04	// Functioneaza ACC+MAG
#define ACCGYRO						0x05	// Functioneaza ACC+GYRO
#define MAGGYRO						0x06	// Functioneaza MAG+GYRO
#define AMG							0x07	// Functioneaza MAG+GYRO+ACC
// Valori Configurare Mod Operare - FUSION
#define IMU 						0x08	// Se fuzioneaza datele primite de la ACC+GYRO
#define COMPASS	 					0x09	// Masoara Campul Magnetic al Pamantului de la MAG
#define M4G							0x0A 	// Similar cu IMU, doar ca se folosesc MAG+GYRO
#define NDOF_FMC_OFF				0x0B	// ORIENTARE ABSOLUTA, DAR FARA CALIBRARE RAPIDA A MAG
#define	NDOF						0x0C	// ORIENTARE ABSOLUTA CU CALIBRARE RAPIDA A MAG
// Valori Configurare Mod Power
#define NORMAL_MODE					0x00
#define LOW_POWER_MODE				0x01
#define SUSPEND_MODE				0x02	// Sistemul ia o pauza si se dezactiveaza toti senzorii. Nu se vor modifica valorile niciunui registru in acest mod!

// Definire Interfete de Comunicatie cu Senzorii
extern I2C_HandleTypeDef hi2c2;
#define IMU_1_2_I2C						hi2c2
extern I2C_HandleTypeDef hi2c1;
#define IMU_3_4_I2C						hi2c1

// Definire Interfete de Comunicatie cu Interfata Grafica
extern UART_HandleTypeDef huart3;
#define Serial_Plot_UART				huart3

// Definire Struct pentru stocarea datelor primite de la senzor
typedef struct{

	double w,x,y,z;

	uint8_t cal_state_sys,cal_state_mag,cal_state_gyr,cal_state_acc;

	uint16_t cal_offset_x_mag,cal_offset_y_mag,cal_offset_z_mag;
	uint16_t cal_offset_x_gyr,cal_offset_y_gyr,cal_offset_z_gyr;
	uint16_t cal_offset_x_acc,cal_offset_y_acc,cal_offset_z_acc;
	uint16_t cal_radius_acc,cal_radius_mag;

	uint16_t cal_offset_x_mag_flash,cal_offset_y_mag_flash,cal_offset_z_mag_flash;
	uint16_t cal_offset_x_gyr_flash,cal_offset_y_gyr_flash,cal_offset_z_gyr_flash;
	uint16_t cal_offset_x_acc_flash,cal_offset_y_acc_flash,cal_offset_z_acc_flash;
	uint16_t cal_radius_acc_flash,cal_radius_mag_flash;
	//uint16_t Buffer_Flash[11];

	uint8_t cal_buffer[22];

	double Roll,Pitch,Yaw;

} BNO055_Data;

// Definire Functii in Header
void BNO055_Test(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address);
void BNO055_Set_Page(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Register_Address, uint8_t Page,uint8_t Module_Address);
void BNO055_Read_Reg(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Register_Address, uint8_t* Valoare_Registru,uint8_t Module_Address,uint8_t Numar_Bytes);
void BNO055_Write_Reg(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Register_Address, uint8_t* Transmitted_Data,uint8_t Module_Address,uint8_t Numar_Bytes);
void BNO055_Initialise(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address);
void BNO055_Read_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,BNO055_Data* IMU_Data,uint8_t Module_Address);
void BNO055_Calculate_Processed_Data_From_Raw(BNO055_Data* IMU_Data);
void BNO055_Set_Operation_Mode(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,uint8_t Mod_Nou_Operare);
void BNO055_Read_Quaternion_W(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Quaternion_X(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Quaternion_Y(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Quaternion_Z(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Quaternion_WXYZ(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Calib_Stat(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Set_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Convert_Quat_to_Euler(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Read_Calib_Data_From_Flash(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);
void BNO055_Transfer_Calib_Data(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address,BNO055_Data* IMU_Data);

int BNO055_Get_Operation_Mode(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart,uint8_t Module_Address);

#endif /* INC_BNO055_H_ */
