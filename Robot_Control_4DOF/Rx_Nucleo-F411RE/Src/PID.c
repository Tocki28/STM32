/*
 * PID.c
 *
 *  Created on: Jul 31, 2023
 *      Author: Andrei
 */
#include "PID.h"
#include "stm32f4xx_hal.h"

void PID_Initialise(PID_Controller *PID)
{
	PID -> Integrator = 0.0f;
	PID -> Prev_Error = 0.0f;
	PID -> Differentiator = 0.0f;
	PID -> Prev_Measurement = 0.0f;
	PID -> Output = 0.0f;
	PID -> Change_In_Error = 0.0f;
	PID -> Change_In_Time = 0.0f;
	PID -> Error_Slope = 0.0f;
	PID -> Error_Area = 0.0f;
	PID -> System_Time_Old = 0.0f;
	PID -> System_Time_New = 0.0f;
	/*
	 * Initial, pentru a calibra controlerul PID, se vor seta Ki si Kd = 0, pentru a seta coeficientul proportional. Se urmareste ca iesirea sistemului sa nu prezinte oscilatii.
	 */
	PID -> K_Proportional = 0.02f;
	PID -> K_Integral = 0.0f;
	PID -> K_Derivative = 0.0f;
}

float PID_Update(PID_Controller *PID,float Target,float Measurement)
{
	PID -> System_Time_Old = PID -> System_Time_New;
	PID -> System_Time_New = HAL_GetTick();

	PID -> Change_In_Time = PID -> System_Time_New - PID -> System_Time_Old;

	float Error = Target - Measurement;

	PID -> Change_In_Error = Error - PID -> Prev_Error;
	PID -> Error_Slope = (PID -> Change_In_Error)/(PID -> Change_In_Time);

	PID -> Error_Area = PID -> Error_Area + Error*(PID -> Change_In_Time);

	/*
	 * Eroarea Sistemului:
	 */


	/*
	 * Termenul Proportional:
	 */
	float Proportional = PID -> K_Proportional * Error;

	/*
	 * Termenul Integrator:
	 */
	PID -> Integrator = (PID -> K_Integral) * (PID -> Error_Area);

	/*if(PID -> Integrator > PID -> Int_Max_Limit)
	{
		PID -> Integrator = PID -> Int_Max_Limit;
	}
	else if(PID -> Integrator < PID -> Int_Min_Limit)
	{
		PID -> Integrator = PID -> Int_Min_Limit;
	}
*/
	/*
	* Termenul Derivator:
	*/
	PID -> Differentiator = (PID -> K_Derivative) * (PID -> Error_Slope);

	/*
	 * Calculul Output-ului + Limitarea Acestuia:
	 */
	PID -> Output = Proportional + PID -> Integrator + PID -> Differentiator;

	/*
	 * Aceasta este limitarea pe care am decis sa o fac in masurarea unghiurilor pentru optimizare, NU in PID.
	 */
	/*if((PID -> Output > 180) && (PID -> Output < 270))
	{
		PID -> Output = 180;
	}
	if((PID -> Output < 0) || (PID -> Output > 270))
	{
		PID -> Output = 0;
	}
	*/
	PID -> Prev_Error = Error;

	PID -> Prev_Measurement = Measurement;
	return PID -> Output;
}


