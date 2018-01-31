/*
* File: quantization.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This file is responsable to implement a way to quantize some sensors data
*/


#include <math.h>    

class quantization
{
	public: double action(double meas, double max,double min, double Nbits)
	{
		double step = (max - min)/Nbits;
		return(meas - fmod (meas,step)); 
	}
};
