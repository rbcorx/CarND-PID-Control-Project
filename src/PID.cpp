#include <iostream>
#include <limits>
#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp; // Proportionnal term
	this->Ki = Ki; // Integral term 
	this->Kd = Kd; // Differential term

	// Proportionnal error init to high value as car away from the CTE initially
	p_error = numeric_limits<double>::max();  

	// integral term init to 0 as initial total area (Integral definition) btw car pos & CTE is NULL 
	// as car yet to start
	i_error = 0.0;

	// differential error init ti 0 as initial car pos is assumed to away from CTE
	d_error = 0.0; 
}

void PID::UpdateError(double cte) {
	if (p_error == numeric_limits<double>::max())
		p_error = cte;
	d_error = cte - p_error; 
	p_error = cte;
	i_error += cte; 
}

double PID::TotalError() {
	// Using: -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
	//return -Kp * p_error - Kd * d_error - Ki * i_error;
	return Kp * p_error + Kd * d_error + Ki * i_error;
}

