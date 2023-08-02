/*
 * PID.c
 *
 *  Created on: Jul 31, 2023
 *      Author: Andrei
 */
#include "PID.h"

void PID_Initialise(PID_Controller *PID)
{
	PID -> Integrator = 0.0f;
	PID -> Prev_Error = 0.0f;
	PID -> Differentiator = 0.0f;
	PID -> Prev_Measurement = 0.0f;
	PID -> Output = 0.0f;

	PID -> K_Proportional = 0.4f;
	PID -> K_Integral = 2.0f;
	PID -> K_Derivative = 0.00001f;
	PID -> FTJ_Time_Constant = 0.02f;
	PID -> Min_Limit = -90.0f;
	PID -> Max_Limit = 90.0f;
	PID -> Int_Min_Limit = -10.0f;
	PID -> Int_Max_Limit = 10.0f;
	PID -> Sample_Time = 0.01f;
}

float PID_Update(PID_Controller *PID,float Target,float Measurement)
{
	/*
	 * Eroarea Sistemului:
	 */
	float Error = Target - Measurement;

	/*
	 * Termenul Proportional:
	 */
	float Proportional = PID -> K_Proportional * Error;

	/*
	 * Termenul Integrator:
	 */
	PID -> Integrator = PID -> Integrator + 0.5f * PID -> K_Integral * PID -> Sample_Time * (Error + PID -> Prev_Error);

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
	PID -> Differentiator = -(2.0f * PID -> K_Derivative * (Measurement - PID -> Prev_Measurement) + (2.0f * PID -> FTJ_Time_Constant - PID -> Sample_Time) * PID -> Differentiator) / (2.0f * PID -> FTJ_Time_Constant + PID -> Sample_Time);

	/*
	 * Calculul Output-ului + Limitarea Acestuia:
	 */
	PID -> Output = Proportional + PID -> Integrator + PID -> Differentiator;

	if(PID -> Output > PID -> Max_Limit)
	{
		PID -> Output = PID -> Max_Limit;
	}
	else if(PID -> Output < PID -> Min_Limit)
	{
		PID -> Output = PID -> Min_Limit;
	}

	PID -> Prev_Error = Error;
	PID -> Prev_Measurement = Measurement;

	return PID -> Output;
}
