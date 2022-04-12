#include <iostream>
//#include <Eigen/Eigen>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::MatrixXd SkewSymmetricMatrix(Eigen::VectorXd Vector)
{
	//Place Vet in the Skew Symmetric matrix S
	Eigen::MatrixXd SkewMatrix(3,3);
	SkewMatrix <<            0,      -Vector(2),       Vector(1),
			 Vector(2),               0,      -Vector(0),
			-Vector(1),       Vector(0),               0;
      
	return SkewMatrix;
}
