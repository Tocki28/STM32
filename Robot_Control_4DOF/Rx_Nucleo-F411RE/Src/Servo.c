/*
 * Servo.c
 *
 *  Created on: Jul 8, 2023
 *      Author: Andrei
 */

#include <Servo.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <PID.h>
#include <BNO055.h>
#include <main.h>
/*
 * S-a setat Frecventa TMR (APB1 TIMER CLOCKS) la 64MHz;
 * In functie de aceasta frecventa, Prescaler si Registrul ARR, se va calcula frecventa de la iesirea canalelor, iar in cazul motoarelor Servo se foloseste frecventa 50Hz, deci:
 * 									TIMER FREQUENCY = (CLOCK FREQUENCY)/(PRESCALER * ARR);
 *
 * 	Pentru controlul motorului Servo se vor genera impulsuri PWM cu durata:
 * 				1ms = (Pozitionare la 0 grade);
 * 				1.5ms = (Pozitionare la 90 de grade);
 * 				2ms = (Pozitionare la 180 de grade).
 * 	Iar pentru o frecventa de 50Hz (T = 1/F = 1/50 = 20ms), in cazul in care ARR este incarcat cu valoarea de 20000, cand CCR = 20000, vom avea Duty Cycle = 100%.
 * 	Analog, pentru o durata a impulsului de 1ms, va trebui incarcat CCR = 450;
 * 	Analog, pentru o durata a impulsului de 2ms, va trebui incarcat CCR = 2450;
 *
 * 	Insa, intrucat motoarele sunt de proasta calitate (Fabricate in China) - pentru 0 grade, impulsul necesar nu va fi de 1ms, ci va fi mai mic sau mai mare. Prin incercari succesive,
 * 	se poate observa ca:
 * 				CCR = Valori diferite in functie de constructia fiecarui motor - (Pozitionare la 0 grade);
 * 				CCR = Valori diferite in functie de constructia fiecarui motor - (Pozitionare la 180 grade);
 */

void Servo_Initialise(Servo_Control *Servo)
{
	Servo -> CCR_Rotire_Baza_Target = 0.0f;
	Servo -> CCR_Rotire_Baza_Update = 90.0f;

	Servo -> CCR_Rotire_Gheara_Target = 0.0f;
	Servo -> CCR_Rotire_Gheara_Update = 0.0f;

	Servo -> Balansare_Gheara_Target = 0.0f;
	Servo -> Balansare_Gheara_Update = 0.0f;

	Servo -> Balansare_Baza_Target = 0.0f;
	Servo -> Balansare_Baza_Update = 0.0f;

	Servo -> Angle_Balansare_Baza_Old = 0;
	Servo -> Angle_Balansare_Gheara_Old = 0;
}
extern uint8_t Rx_Data[5];
void Servo_Set_Angle_Rotire_Baza(uint8_t Angle,PID_Controller *PID, Servo_Control *Servo,BNO055_Data *Feedback)
{
	PID -> K_Proportional = 0.04;
	BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,Feedback);
	BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,Feedback);
	//Tx[5] = 0;
	//Tx[4] = (uint8_t)Feedback -> Yaw;
	//Angle = Angle + 90;
	/*if(Angle >= 180)
	{
		Angle = 180;
	}
	else if(Angle <=0)
	{
		Angle = 0;
	}*/
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (2450-450);
	//				Angle	  ..... CCR = x;
	Servo -> CCR_Rotire_Baza_Target = (Rx_Data[1]*(2600-600))/180+600;
	while(abs(Servo -> CCR_Rotire_Baza_Target - Servo -> CCR_Rotire_Baza_Update) > 20)
	{
		Servo -> CCR_Rotire_Baza_Target = (Rx_Data[1]*(2600-600))/180+600;
		Servo -> CCR_Rotire_Baza_Update = Servo -> CCR_Rotire_Baza_Update + PID_Update(PID,Servo -> CCR_Rotire_Baza_Target,Servo -> CCR_Rotire_Baza_Update);
		__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Rotire_Baza,(uint16_t)Servo -> CCR_Rotire_Baza_Update);
		HAL_Delay(5);
		BNO055_Read_Quaternion_WXYZ(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,Feedback);
		BNO055_Convert_Quat_to_Euler(&IMU_1_2_I2C,&Serial_Plot_UART,BNO055_MODULE_1_ADDRESS,Feedback);
	//Rx_Data[0] = (uint8_t)Servo -> CCR_Rotire_Baza_Update;
	///Rx_Data[1] = (uint16_t)Servo -> CCR_Rotire_Baza_Update >> 8;
	//Rx_Data[2] = (uint8_t)Servo -> CCR_Rotire_Baza_Target;
	///Rx_Data[3] = (uint16_t)Servo -> CCR_Rotire_Baza_Target >> 8;
	//Rx_Data[4] = (uint8_t)Feedback -> Yaw;
	//Rx_Data[5] = 0;
	}
	HAL_Delay(20);
	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Rotire_Baza,0);
	PID -> K_Proportional = 0.04;
}


void Servo_Set_Angle_Balansare_Baza_Stanga(uint8_t Angle, Servo_Control *Servo)
{
	//Angle = Angle + 90;
	Angle = 180-Angle;
	if(Angle >= 140)
	{
		Angle = 140;
	}
	else if(Angle <= 15)
	{
		Angle = 15;
	}
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (105-20);
	//				Angle	  ..... CCR = x;
	// La valoarea finala se adauga 20 pentru a elimina offsetul (0 grade -> CCR = 20)
	float Reg_Value = (Angle*(2800-700))/180+700;

	/*
	 *	Deoarece Servo-urile nu sunt extrem de calitative, este ft greu sa le corelezi in oglinda, iar in momentul in care
	 *	robotul balanseaza gheara cu o valoare redusa a unghiului, un singur servo face fata, abia cand unghiul sare de 20
	 *	de grade intervine si cel de al doilea pentru a ajuta, oprindu-se la 3ms dupa.
	 */
	if(abs(Angle - Servo -> Angle_Balansare_Baza_Old) > 20)
	{
	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Balansare_Baza_Stanga,Reg_Value);
	HAL_Delay(10);
	}

	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Balansare_Baza_Stanga,0);
	HAL_Delay(3);
	Servo -> Angle_Balansare_Baza_Old = Angle;
}
void Servo_Set_Angle_Balansare_Baza_Dreapta(uint8_t Angle)
{
	//Angle = Angle + 90;
	//Angle = 180-Angle;	// De ce se pune acest Offset? Deoarece motoarele sunt in oglinda, deci ce reprezinta 180 de grade pentru unul, va reprezenta 0 grade pentru celalalt!
	if(Angle >= 140)
	{
		Angle = 140;
	}
	else if(Angle <= 10)
	{
		Angle = 10;
	}
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (120-45);
	//				Angle	  ..... CCR = x;
	// La valoarea finala se adauga 45 pentru a elimina offsetul (0 grade -> CCR = 45)
	float Reg_Value = (Angle*(2600-700))/180+700;
	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Balansare_Baza_Dreapta,Reg_Value);
	HAL_Delay(3);
}
void Servo_Set_Angle_Rotire_Gheara(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID)
{
	//Angle = Angle + 90;
	if(Angle >= 180)
	{
		Angle = 180;
	}
	else if(Angle <=0)
	{
		Angle = 0;
	}
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (135-30);
	//				Angle	  ..... CCR = x;
	// La valoarea finala se adauga 30 pentru a elimina offsetul (0 grade -> CCR = 30)
	uint16_t CCR_Servo;
	CCR_Servo = (Angle*(2450-450))/180+450;
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Rotire_Gheara,CCR_Servo);
	HAL_Delay(5);
}

void Servo_Set_Angle_Balansare_Gheara_Stanga(uint8_t Angle,Servo_Control *Servo)
{
	if(Angle >= 180)
	{
		Angle = 180;
	}
	else if(Angle <=0)
	{
		Angle = 0;
	}
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (130-28);
	//				Angle	  ..... CCR = x;
	// La valoarea finala se adauga 28 pentru a elimina offsetul (0 grade -> CCR = 28)
	float Reg_Value = (Angle*(1450-450))/180+450;
	//if(abs(Angle - Servo -> Angle_Balansare_Gheara_Old) > 20)
	//{
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Balansare_Gheara_Stanga,Reg_Value);
	HAL_Delay(3);
	//}
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Balansare_Gheara_Stanga,0);
	HAL_Delay(2);
	//Servo -> Angle_Balansare_Gheara_Old = Angle;
}
void Servo_Set_Angle_Balansare_Gheara_Dreapta(uint8_t Angle)
{
	Angle = 180-Angle;
	if(Angle >= 180)
	{
		Angle = 180;
	}
	else if(Angle <=0)
	{
		Angle = 0;
	}
	// Prin regula de 3 simpla se obtine:
	//				180 grade ..... CCR = (130-28);
	//				Angle	  ..... CCR = x;
	// La valoarea finala se adauga 28 pentru a elimina offsetul (0 grade -> CCR = 28)
	float Reg_Value = (Angle*(2600-1600))/180+1600;
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Balansare_Gheara_Dreapta,Reg_Value);
	HAL_Delay(3);
}
void Servo_Set_Angle_Balansare_Baza(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID)
{
	//Servo -> CCR_Rotire_Gheara_Target = (Angle*(2450-450))/180+450;
	//PID -> K_Proportional = 0.08;
	if(Angle >= 180)
	{
		Angle = 180;
	}
	else if(Angle <= 10)
	{
		Angle = 10;
	}
	while(abs(Angle - Servo -> Balansare_Baza_Update) > 10)
	{
		Servo -> Balansare_Baza_Update = Servo -> Balansare_Baza_Update + PID_Update(PID,Rx_Data[3],Servo -> Balansare_Baza_Update);
		Servo_Set_Angle_Balansare_Baza_Stanga(Servo -> Balansare_Baza_Update,Servo);
		Servo_Set_Angle_Balansare_Baza_Dreapta(Servo -> Balansare_Baza_Update);
		HAL_Delay(10);
	}
	PID -> K_Proportional = 0.04;

}
void Servo_Set_Angle_Balansare_Gheara(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID)
{
	PID -> K_Proportional = 0.08;
	while(abs(Rx_Data[2] - Servo -> Balansare_Gheara_Update) > 10)
	{
		Servo -> Balansare_Gheara_Update = Servo -> Balansare_Gheara_Update + PID_Update(PID,Rx_Data[2],Servo -> Balansare_Gheara_Update);
		Servo_Set_Angle_Balansare_Gheara_Stanga(Servo -> Balansare_Gheara_Update,Servo);
		Servo_Set_Angle_Balansare_Gheara_Dreapta(Servo -> Balansare_Gheara_Update);
	}
	PID -> K_Proportional = 0.04;
}
void Setare_Pozitie_Initiala(Servo_Control *Servo,UART_HandleTypeDef *huart,PID_Controller *PID,BNO055_Data *Feedback)
{
	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Rotire_Baza,1600);
	__HAL_TIM_SET_COMPARE(&Corp_Brat,Motor_Balansare_Baza_Stanga,2800);
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Balansare_Gheara_Stanga,1450);
	__HAL_TIM_SET_COMPARE(&Gheara,Motor_Rotire_Gheara,1450);

}
