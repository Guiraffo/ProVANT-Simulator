
#include <math.h>    

class quantization
{
	public: double action(double meas, double max,double min, double Nbits)
	{
		double step = (max - min)/Nbits;
		return(meas - fmod (meas,step)); 
	}
};
