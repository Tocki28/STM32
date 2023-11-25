/*
 * Servo.h
 *
 *  Created on: Jul 8, 2023
 *      Author: Andrei
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "BNO055.h"
#include "main.h"
/*
 * Redenumire Canale si TIM, conform cu motoarele, pentru usurinta:
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define Gheara							htim3
#define Corp_Brat						htim2

#define Motor_Rotire_Baza				TIM_CHANNEL_1
#define Motor_Balansare_Baza_Stanga		TIM_CHANNEL_2
#define Motor_Balansare_Baza_Dreapta	TIM_CHANNEL_3
#define Motor_Rotire_Gheara				TIM_CHANNEL_3
#define Motor_Balansare_Gheara_Stanga	TIM_CHANNEL_1
#define Motor_Balansare_Gheara_Dreapta	TIM_CHANNEL_2

typedef struct
{

	float Angle_Balansare_Baza_Old;
	float Angle_Balansare_Gheara_Old;

	float CCR_Rotire_Baza_Target;
	float CCR_Rotire_Baza_Update;

	float CCR_Rotire_Gheara_Target;
	float CCR_Rotire_Gheara_Update;

	float Balansare_Baza_Target;
	float Balansare_Baza_Update;

	float Balansare_Gheara_Target;
	float Balansare_Gheara_Update;

} Servo_Control;

void Servo_Initialise(Servo_Control *Servo);
void Servo_Set_Angle_Rotire_Baza(uint8_t Angle,PID_Controller *PID,Servo_Control *Servo,BNO055_Data *Feedback);
void Servo_Set_Angle_Balansare_Baza_Stanga(uint8_t Angle,Servo_Control *Servo);
void Servo_Set_Angle_Balansare_Baza_Dreapta(uint8_t Angle);
void Servo_Set_Angle_Rotire_Gheara(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID);
void Servo_Set_Angle_Balansare_Gheara_Stanga(uint8_t Angle,Servo_Control *Servo);
void Servo_Set_Angle_Balansare_Gheara_Dreapta(uint8_t Angle);
void Servo_Set_Angle_Balansare_Baza(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID);
void Servo_Set_Angle_Balansare_Gheara(uint8_t Angle, Servo_Control *Servo, PID_Controller *PID);
void Setare_Pozitie_Initiala(Servo_Control *Servo,UART_HandleTypeDef *huart,PID_Controller *PID,BNO055_Data *Feedback);

#endif /* INC_SERVO_H_ */
