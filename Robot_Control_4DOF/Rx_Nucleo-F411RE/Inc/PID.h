/*
 * PID.h
 *
 *  Created on: Jul 31, 2023
 *      Author: Andrei
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct
{
	/*
	 * Constante de proportionalitate Controller:
	 */
	float K_Proportional;
	float K_Integral;
	float K_Derivative;

	/*
	 * Constanta de Timp a FTJ, de care avem nevoie pentru a atenua zgomotul introdus de termenul Derivative:
	 */
	float FTJ_Time_Constant;

	/*
	 * Limitele Iesirii Controllerului:
	 */
	float Min_Limit;
	float Max_Limit;

	/*
	 * Limite Integrator:
	 */
	float Int_Min_Limit;
	float Int_Max_Limit;

	/*
	 * Perioada de Esantionare:
	 */
	float Sample_Time;

	/*
	 * Date Controller:
	 */
	float System_Time_Old;
	float System_Time_New;
	int Change_In_Time;

	float Integrator;
	float Prev_Error;
	float Differentiator;
	float Prev_Measurement;
	float Change_In_Error;
	float Error_Slope;
	float Error_Area;
	/*
	 * Output
	 */
	float Output;
} PID_Controller;

void PID_Initialise(PID_Controller *PID);
float PID_Update(PID_Controller *PID,float Target,float Measurement);

#endif /* INC_PID_H_ */
