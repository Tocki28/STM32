/*
 * NRF24L01.c
 *
 *  Created on: Sep 16, 2023
 *      Author: Andrei
 */

#include "stm32h7xx_hal.h"
#include "stdio.h"
#include "NRF24L01.h"


extern SPI_HandleTypeDef hspi3;
#define NRF24_SPI &hspi3

#define NRF24_CE_PORT   GPIOD
#define NRF24_CE_PIN    GPIO_PIN_0

#define NRF24_CSN_PORT   GPIOA
#define NRF24_CSN_PIN    GPIO_PIN_15


void Delay_us(uint16_t Micros, TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim,0);
	while(__HAL_TIM_GET_COUNTER(htim) < Micros);
	// Maximum 65535 microseconds = 65.535 miliseconds
	// Minimum 1 microsecond
}

void CS_Select (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect (void)
{
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}


void CE_Enable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable (void)
{
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}


// write a single byte to the particular register
void NRF24L01_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t Buffer[2];
	Buffer[0] = Reg|(1<<5);
	Buffer[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, Buffer, 2, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

void NRF24L01_WriteRegMulti (uint8_t Reg, uint8_t *Data, uint8_t Size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);			// Transmite registrul in care se doreste a scrie;
	HAL_SPI_Transmit(NRF24_SPI, Data, Size, 1000);		// Se transmite intregul set de date.

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

uint8_t NRF24L01_ReadReg (uint8_t Reg)
{
	uint8_t Data = 0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &Data, 1, 100);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return Data;
}

/* Read multiple bytes from the register */
void NRF24L01_ReadRegMulti (uint8_t Reg, uint8_t *Data, uint8_t Size)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, Data, Size, 1000);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


// send the command to the NRF
void NRF24L01_SendCommand (uint8_t Command)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Command, 1, 100);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

void NRF24L01_Reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		NRF24L01_WriteReg(STATUS, 0b01111110);
		//NRF24L01_WriteReg(STATUS, 0x0E);
	}

	else if (REG == FIFO_STATUS)
	{
		NRF24L01_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
		NRF24L01_WriteReg(CONFIG, 0x08);
		NRF24L01_WriteReg(EN_AA, 0x3F);
		NRF24L01_WriteReg(EN_RXADDR, 0x03);
		NRF24L01_WriteReg(SETUP_AW, 0x03);
		NRF24L01_WriteReg(SETUP_RETR, 0x03);
		NRF24L01_WriteReg(RF_CH, 0x02);
		NRF24L01_WriteReg(RF_SETUP, 0x0F);
		NRF24L01_WriteReg(STATUS, 0b01111110);
		NRF24L01_WriteReg(OBSERVE_TX, 0x00);
		NRF24L01_WriteReg(CD, 0x00);
		uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF24L01_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
		uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
		NRF24L01_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
		NRF24L01_WriteReg(RX_ADDR_P2, 0xC3);
		NRF24L01_WriteReg(RX_ADDR_P3, 0xC4);
		NRF24L01_WriteReg(RX_ADDR_P4, 0xC5);
		NRF24L01_WriteReg(RX_ADDR_P5, 0xC6);
		uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF24L01_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
		NRF24L01_WriteReg(RX_PW_P0, 0x00);
		NRF24L01_WriteReg(RX_PW_P1, 0x00);
		NRF24L01_WriteReg(RX_PW_P2, 0x00);
		NRF24L01_WriteReg(RX_PW_P3, 0x00);
		NRF24L01_WriteReg(RX_PW_P4, 0x00);
		NRF24L01_WriteReg(RX_PW_P5, 0x00);
		NRF24L01_WriteReg(FIFO_STATUS, 0x11);
		NRF24L01_WriteReg(DYNPD, 0x00);
		NRF24L01_WriteReg(FEATURE, 0x00);
	}
}

void NRF24L01_Init_TxAA (TIM_HandleTypeDef *htim)
{
	// disable the chip before configuring the device
	CE_Disable();

	// reset everything
	NRF24L01_Reset (0);

	NRF24L01_WriteReg(CONFIG, 0b01001110);  // Configure conform caietel;

	NRF24L01_WriteReg(EN_AA, 0b00000001);  // Auto - Acknowledgement bit pe Pipe 0;

	NRF24L01_WriteReg(EN_RXADDR, 0b00000001);  // Seteaza Rx pe Pipe 0;

	NRF24L01_WriteReg(SETUP_AW, 0b00000011);  // Seteaza marimea adresei Tx/Rx - 5 octeti

	NRF24L01_WriteReg(SETUP_RETR, 0b00011111);   // Delay de 500 + 86us pana la urmatoarea transmisie de date + 15 retransmisii

	NRF24L01_WriteReg(RF_CH, 0b00000010);  // Seteaza frecventa canalului pe care functioneaza transmitatorul, i-am dat reset value;

	NRF24L01_WriteReg(RF_SETUP, 0b00000110);   // Power= 0db, data rate = 1Mbps, No LNA

	/*
	 * Seteaza Dynamic Payload pentru Pipe 0 !!!
	 */
	NRF24L01_WriteReg(FEATURE,0b00000110);
	NRF24L01_WriteReg(DYNPD,0b00000001);

	// Clear STATUS Flags
	NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????
	//NRF24L01_WriteReg(STATUS, 0x0E);

	uint8_t Tx_Address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	NRF24L01_WriteRegMulti(TX_ADDR, Tx_Address,5);

	//Set the Pipe 0 RxAddress the same as Tx for Ack byte that is to come;
	NRF24L01_WriteRegMulti(RX_ADDR_P0, Tx_Address,5);

	/*
	 * CE - 1L pentru minim 10us pentru a incepe transmisia sau pentru a intra in Standby Mode II
	 */
	// Daca FIFO - empty si CE - 1L, atunci modulul intra Standby Mode II -> Transmisia urmatorului pachet va avea loc
	// imediat ce se vor incarca date prin SPI si se va aduce CS - 1L.
	CE_Enable();
	Delay_us(50,htim);
}

void NRF24L01_Transmit (uint8_t *Tx_Data)
{
	uint8_t Sending_Command = 0;

	/*
	 * Select the device
	 */
	CS_Select();

	// Sending the PD Command
	Sending_Command = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &Sending_Command, 1, 100);

	// Sending the PD
	HAL_SPI_Transmit(NRF24_SPI, Tx_Data, 5, 1000);

	/*
	 * Deselect the device
	 */
	CS_UnSelect();

	// In mom in care CS - 1L, incepe transmisia, pana cand se va goli FIFO
}

void NRF24L01_Check_IRQ(uint8_t *Tx_Data)
{
	/*
	 * Citim Registrul STATUS pentru a vedea ce intrerupere a avut loc:
	 */
	uint8_t Status = NRF24L01_ReadReg(STATUS);
	/*
	 * Tratam tipul de intrerupere:
	 */
	if( ((Status&(1<<5)) == 0b00100000) && ((Status&(1<<4)) == 0b00000000)) // Daca TX_DS == 1, ACK a fost receptionat cum trebuie
	{
		NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????
		//NRF24L01_WriteReg(STATUS, 0x0E);

		NRF24L01_Transmit (Tx_Data);
	}
	if( ((Status&(1<<4)) == 0b00010000) && (((Status&(1<<5)) == 0b00000000))) // Daca MAX_RT == 1, ACK NU a fost receptionat cum trebuie dupa 15 incercari
	{
		NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????
		NRF24L01_SendCommand(FLUSH_TX);
		NRF24L01_Transmit (Tx_Data);
		//NRF24L01_WriteReg(STATUS, 0x0E);

		/*
		* Pentru a retransmite aceleasi date, se va reseta flag-ul.
		*/
	}
}

void NRF24L01_Init_RxAA(TIM_HandleTypeDef *htim)
{
	// disable the chip before configuring the device
	CE_Disable();

	// reset everything
	NRF24L01_Reset (0);

	NRF24L01_WriteReg(CONFIG, 0b00111111);  // Configure conform caietel;

	NRF24L01_WriteReg(EN_AA, 0b00000010);  // Auto - Acknowledgement bit pe Pipe 0;

	NRF24L01_WriteReg(EN_RXADDR, 0b00000010);  // Seteaza Rx pe Pipe 0;

	NRF24L01_WriteReg(SETUP_AW, 0b00000011);  // Seteaza marimea adresei Tx/Rx - 5 octeti

	NRF24L01_WriteReg(RX_PW_P1, 0b00000110);  // Seteaza Payload Width - 3 octeti

	NRF24L01_WriteReg(SETUP_RETR, 0b00011111);   // Delay de 500 + 86us pana la urmatoarea transmisie de date + 15 retransmisii

	NRF24L01_WriteReg(RF_CH, 0b0000010);  // Seteaza frecventa canalului pe care functioneaza transmitatorul, i-am dat reset value;

	NRF24L01_WriteReg(RF_SETUP, 0b00000110);   // Power= 0db, data rate = 1Mbps, No LNA

	/*
	 * Seteaza Dynamic Payload pentru Pipe 1 !!!
	 */
	NRF24L01_WriteReg(FEATURE,0b00000110);
	NRF24L01_WriteReg(DYNPD,0b00000010);

	// Clear STATUS Flags
	NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????

	uint8_t Tx_Address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

	//Set the Pipe 0 RxAddress the same as Tx for Ack byte that is to come;
	NRF24L01_WriteRegMulti(RX_ADDR_P1, Tx_Address,5);

	/*
	 * CE - 1L pentru minim 10us pentru a incepe transmisia sau pentru a intra in Standby Mode II
	 */
	// Daca FIFO - empty si CE - 1L, atunci modulul intra Standby Mode II -> Transmisia urmatorului pachet va avea loc
	// imediat ce se vor incarca date prin SPI si se va aduce CS - 1L.
	CE_Enable();
	HAL_Delay(10);
}

void NRF24L01_Receive (uint8_t *Rx_Data,uint8_t* Date_Wireless)
{

	uint8_t Status = NRF24L01_ReadReg(FIFO_STATUS);
	if( ((Status&(1<<0)) == 0b00000000) )
	{
		CE_Disable();

		uint8_t Sending_Command = 0;

	/*
	 * Select the device
	 */
		CS_Select();

	// Sending the PD Command
		Sending_Command = R_RX_PAYLOAD;
		HAL_SPI_Transmit(NRF24_SPI, &Sending_Command, 1, 100);

	// Sending the PD
		HAL_SPI_Receive(NRF24_SPI, Rx_Data, 5, 1000);

	/*
	 * Deselect the device
	 */
		CS_UnSelect();
		if(Rx_Data[0] == 255 && Rx_Data[3] == 0 && Rx_Data[1] == 50)
		{
			Sending_Command = FLUSH_RX;
			NRF24L01_SendCommand(Sending_Command);

			NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????
			CE_Enable();
			Date_Wireless[0] = Rx_Data[1];
			Date_Wireless[1] = Rx_Data[2];

		}
		else
		{
			Sending_Command = FLUSH_RX;
			NRF24L01_SendCommand(Sending_Command);

			NRF24L01_WriteReg(STATUS, 0b01111110);	// Write 1L to make FLAGS 0L????

			CE_Enable();
		}
	}

}
