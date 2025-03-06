#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstring>
#include <emmintrin.h>
//#include <Math.h>

#define pi 3.1415926535
#define R 0.2261
#define sign(x) (x>=0) ? 1.0 : -1.0
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;


struct kinematicOut
{
	VectorXd wheelsSpeed;
	Vector4d targSteerAngle;
	Vector4d deltaPhiNew;
	Vector3d robotCoord;
	Vector3d robotSpeed;
};


struct kinematicOut inversKinematic(Vector3d targRobotSpeed, Vector3d robotParam, Vector4d currentSteerAngle, Vector4d steeringSpeedVector)
{
	double a = robotParam(0) / 2;
	double b = robotParam(1) / 2;
	double d = robotParam(2) / 2;

	MatrixXd wheelModulePosition{
		{1, 0, -b},
		{0, 1,  a},
		{1, 0, -b},
		{0, 1, -a},
		{1, 0,  b},
		{0, 1, -a},
		{1, 0,  b},
		{0, 1,  a}
	};
		
	MatrixXd wheelModuleSpeedVector = wheelModulePosition * targRobotSpeed;
	Vector4d SteerAngleRanged;
	Vector4d steerAngleTemp;
	for (int8_t i = 1; i < 8; i=i+2)
	{
		if (wheelModuleSpeedVector(i - 1) == 0) {
			if (wheelModuleSpeedVector(i) == 0)
				SteerAngleRanged(int(floor(i / 2))) = 0;
			else if (wheelModuleSpeedVector(i) > 0)
				SteerAngleRanged(int(floor(i / 2))) = pi / 2;
			else if (wheelModuleSpeedVector(i) < 0)
				SteerAngleRanged(int(floor(i / 2))) = 3 / 2 * pi;
		}
		else if (wheelModuleSpeedVector(i) == 0) {
			if (wheelModuleSpeedVector(i - 1) > 0)
				SteerAngleRanged(int(floor(i / 2))) = 0;
			else
				SteerAngleRanged(int(floor(i / 2))) = pi;
		}
		else if (wheelModuleSpeedVector(i - 1) > 0)
			if (wheelModuleSpeedVector(i) > 0)
				SteerAngleRanged(int(floor(i / 2))) = atan2f(wheelModuleSpeedVector(i), wheelModuleSpeedVector(i - 1));
			else 
				SteerAngleRanged(int(floor(i / 2))) = atan2f(wheelModuleSpeedVector(i), wheelModuleSpeedVector(i - 1)) + 2*pi;
		else
			if(wheelModuleSpeedVector(i)>0)
				SteerAngleRanged(int(floor(i / 2))) = atan2f(wheelModuleSpeedVector(i), wheelModuleSpeedVector(i - 1));
			else
				SteerAngleRanged(int(floor(i / 2))) = atan2f(wheelModuleSpeedVector(i), wheelModuleSpeedVector(i - 1)) + 2 * pi;


        //steerAngleTemp(i-1)=SteerAngleRanged(i-1);
	}

	Vector4d deltaPhi;
	Vector4d PhiRanged;
	Vector4d PhiCloser;
	Vector4d deltaPhiNew;
	Vector4d deltaPhiTemp;
	Vector4d SpeedSign;


	for (int8_t i = 0; i < 4; i++)
	{
		deltaPhi(i) = SteerAngleRanged(i) - currentSteerAngle(i);
		PhiRanged(i) = deltaPhi(i) - trunc(fabs(deltaPhi(i)) / (pi + 0.0000000001)) * 2 * pi;
		PhiCloser(i) = PhiRanged(i) + trunc(fabs(PhiRanged(i)) / (pi / 2 + 0.0000000001)) * pi;
		deltaPhiTemp(i) = PhiCloser(i) - trunc(abs(PhiCloser(i)) / (pi + 0.0000000001)) * 2 * pi;
		//SpeedSign(i) = float (pow(signbit(-1 * abs(deltaPhiTemp(i) - PhiRanged(i)) / pi), int8_t (signbit(-1 * abs(deltaPhiTemp(i) - PhiRanged(i)) / pi))));
		SpeedSign(i) = sign(-1 * abs(deltaPhiTemp(i) - PhiRanged(i)) / pi);
		deltaPhiNew(i) = deltaPhiTemp(i);
		steerAngleTemp(i) = deltaPhiNew(i) + currentSteerAngle(i);

		if (steerAngleTemp(i) < (0))
			steerAngleTemp(i) = steerAngleTemp(i) + 2 * pi;
		else if (steerAngleTemp(i) >= (2 * pi))
			steerAngleTemp(i) = steerAngleTemp(i) - 2 * pi;

	}



	MatrixXd KinematicMatrix1{
		{1 , 0 ,  d * sin(currentSteerAngle(0)) + a},
		{0 , 1 , -d * cos(currentSteerAngle(0)) + b},
		{1 , 0 , -d * sin(currentSteerAngle(0)) + a},
		{0 , 1 ,  d * cos(currentSteerAngle(0)) + b},
		{1 , 0 ,  d * sin(currentSteerAngle(1)) - a},
		{0 , 1 , -d * cos(currentSteerAngle(1)) + b},
		{1 , 0 , -d * sin(currentSteerAngle(1)) - a},
		{0 , 1 ,  d * cos(currentSteerAngle(1)) + b},
		{1 , 0 ,  d * sin(currentSteerAngle(2)) - a},
		{0 , 1 , -d * cos(currentSteerAngle(2)) - b},
		{1 , 0 , -d * sin(currentSteerAngle(2)) - a},
		{0 , 1 ,  d * cos(currentSteerAngle(2)) - b},
		{1 , 0 ,  d * sin(currentSteerAngle(3)) + a},
		{0 , 1 , -d * cos(currentSteerAngle(3)) - b},
		{1 , 0 , -d * sin(currentSteerAngle(3)) + a},
		{0 , 1 ,  d * cos(currentSteerAngle(3)) - b}
	};

	MatrixXd KinematicMatrix2{
		{ d * cos(currentSteerAngle(0)),					0,								   0,								   0},
		{ d * sin(currentSteerAngle(0)),					0,								   0,								   0},
		{-d * cos(currentSteerAngle(0)),					0,								   0,								   0},
		{-d * sin(currentSteerAngle(0)),					0,								   0,								   0},
		{				 0,					 d * cos(currentSteerAngle(1)),					   0,								   0},
		{				 0,					 d * sin(currentSteerAngle(1)),					   0,								   0},
		{				 0,					-d * cos(currentSteerAngle(1)),					   0,								   0},
		{				 0,					-d * sin(currentSteerAngle(1)),					   0,								   0},
		{				 0,									0,					d * cos(currentSteerAngle(2)),					   0},
		{				 0,									0,				   -d * cos(currentSteerAngle(2)),					   0},
		{				 0,									0,				   -d * sin(currentSteerAngle(2)),					   0},
		{				 0,									0,					d * sin(currentSteerAngle(2)),					   0},
		{				 0,									0,								   0,					d * cos(currentSteerAngle(3))},
		{				 0,									0,								   0,					d * sin(currentSteerAngle(3))},
		{				 0,									0,								   0,				   -d * cos(currentSteerAngle(3))},
		{				 0,									0,								   0,				   -d * sin(currentSteerAngle(3))}
	};


	MatrixXd rotSign{
		{double(sign(steeringSpeedVector(0))),  0, 0, 0, 0, 0, 0, 0},
		{0, -double(sign(steeringSpeedVector(0))), 0, 0, 0, 0, 0, 0},
		{0, 0, double(sign(steeringSpeedVector(1))),  0, 0, 0, 0, 0},
		{0, 0, 0, -double(sign(steeringSpeedVector(1))), 0, 0, 0, 0},
		{0, 0, 0, 0, double(sign(steeringSpeedVector(2))), 0, 0, 0},
		{0, 0, 0, 0, 0, -double(sign(steeringSpeedVector(2))), 0, 0},
		{0, 0, 0, 0, 0, 0, double(sign(steeringSpeedVector(3))),  0},
		{0, 0, 0, 0, 0, 0, 0, -double(sign(steeringSpeedVector(3)))}
	};
	MatrixXd SignSpeedMatrix{
		{SpeedSign(0), 0, 0, 0, 0, 0, 0, 0},
		{0, SpeedSign(0), 0, 0, 0, 0, 0, 0},
		{0, 0, SpeedSign(1), 0, 0, 0, 0, 0},
		{0, 0, 0, SpeedSign(1), 0, 0, 0, 0},
		{0, 0, 0, 0, SpeedSign(2), 0, 0, 0},
		{0, 0, 0, 0, 0, SpeedSign(2), 0, 0},
		{0, 0, 0, 0, 0, 0, SpeedSign(3), 0},
		{0, 0, 0, 0, 0, 0, 0, SpeedSign(3)}
	};

	VectorXd A = KinematicMatrix1 * targRobotSpeed;
	VectorXd B = KinematicMatrix2 * steeringSpeedVector;

	VectorXd WheelsSpeedLinear(8);
	VectorXd WheelsSteeringSpeed(8);
	
	for (int8_t i = 1; i < 16; i = i + 2)
	{
		double Ax = pow(A(i-1),2);
		double Ay = pow(A(i), 2);
		double Bx = pow(B(i - 1), 2);
		double By = pow(B(i), 2);
		uint8_t j = floor(i / 2);
		WheelsSpeedLinear(j) = sqrtf (Ax + Ay);
		WheelsSteeringSpeed(j) = sqrtf (Bx+By);
	}
	VectorXd C = SignSpeedMatrix * WheelsSpeedLinear;
	VectorXd E = rotSign * WheelsSteeringSpeed;

	VectorXd wheelsSpeedMperS = C - E;
	
	MatrixXd radiusmatrix{
		{1 / R, 0, 0, 0, 0, 0, 0, 0},
		{0, 1 / R, 0, 0, 0, 0, 0, 0},
		{0, 0, 1 / R, 0, 0, 0, 0, 0},
		{0, 0, 0, 1 / R, 0, 0, 0, 0},
		{0, 0, 0, 0, 1 / R, 0, 0, 0},
		{0, 0, 0, 0, 0, 1 / R, 0, 0},
		{0, 0, 0, 0, 0, 0, 1 / R, 0},
		{0, 0, 0, 0, 0, 0, 0, 1 / R}
	};

	VectorXd wheelsSpeed = radiusmatrix * wheelsSpeedMperS;

	struct kinematicOut out;
	out.wheelsSpeed = wheelsSpeed;
	out.deltaPhiNew = deltaPhiNew;
	//out.deltaPhiNew = steerAngleTemp;
	out.targSteerAngle = steerAngleTemp;

	return out;
}

struct kinematicOut forwardKinematic(VectorXd motorSpeed, Vector4d currentSteerAngle, Vector3d robotParam)
{
	double a = robotParam(0) / 2;
	double b = robotParam(1) / 2;
	double d = robotParam(2) / 2;

	MatrixXd kinemMatrix{
		{													cos(currentSteerAngle(0)) / 8,																								cos(currentSteerAngle(0)) / 8,																										cos(currentSteerAngle(1)) / 8,																 cos(currentSteerAngle(1)) / 8,																																			cos(currentSteerAngle(2)) / 8,																											cos(currentSteerAngle(2)) / 8,																									cos(currentSteerAngle(3)) / 8,																									cos(currentSteerAngle(3)) / 8},
		{													sin(currentSteerAngle(0)) / 8,																								sin(currentSteerAngle(0)) / 8,																										sin(currentSteerAngle(1)) / 8,																 sin(currentSteerAngle(1)) / 8,																																			sin(currentSteerAngle(2)) / 8,																											sin(currentSteerAngle(2)) / 8,																									sin(currentSteerAngle(3)) / 8,																									sin(currentSteerAngle(3)) / 8},
		{double((a * sin(currentSteerAngle(0)) + d - b * cos(currentSteerAngle(0))) / (8 * pow(d,2) + 8 * pow(b,2) + 8 * pow(a, 2))),    double((a * sin(currentSteerAngle(1)) - d - b * cos(currentSteerAngle(0))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),     double((-a * sin(currentSteerAngle(1)) + d - b * cos(currentSteerAngle(1))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),    double((-a * sin(currentSteerAngle(1)) - d - b * cos(currentSteerAngle(1))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),    double((-a * sin(currentSteerAngle(2)) + d + b * cos(currentSteerAngle(2))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),        double((-a * sin(currentSteerAngle(2)) - d + b * cos(currentSteerAngle(2))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),   double((a * sin(currentSteerAngle(3)) + d + b * cos(currentSteerAngle(3))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2))),     double((a * sin(currentSteerAngle(3)) - d + b * cos(currentSteerAngle(3))) / (8 * pow(d, 2) + 8 * pow(b, 2) + 8 * pow(a, 2)))}
	};
	MatrixXd radiusmatrix{
		{R, 0, 0, 0, 0, 0, 0, 0},
		{ 0, R, 0, 0, 0, 0, 0, 0},
		{ 0, 0,R, 0, 0, 0, 0, 0},
		{ 0, 0, 0, R, 0, 0, 0, 0},
		{ 0, 0, 0, 0,R, 0, 0, 0},
		{ 0, 0, 0, 0, 0, R, 0, 0},
		{ 0, 0, 0, 0, 0, 0,R, 0},
		{ 0, 0, 0, 0, 0, 0, 0, R}
	};
	VectorXd MS = radiusmatrix * motorSpeed;

	struct kinematicOut out;
	out.robotSpeed = kinemMatrix * MS;
	return out;
}

//int main()
//{
//	Vector3f RS{0,0,0 };
//	Vector3f RP{ 1.57, 2.1, 0.237 };
//	Vector4f CSA{ 0, 0, 0, 0 };
//	Vector4f SSV{ 0, 0, 0 , 0 };
//	struct kinematicOut test = inversKinematic(RS, RP, CSA, SSV);
//	struct kinematicOut test2 = forwardKinematic(test.wheelsSpeed, test.targSteerAngle, RP);
//	std::cout << "WS" << std::endl << test.wheelsSpeed << std::endl;
//	std::cout << "DPhi" << std::endl << test.deltaPhiNew << std::endl;
//	std::cout << "SA" << std::endl << test.targSteerAngle << std::endl;
//	std::cout << "RS" << std::endl << test2.robotSpeed << std::endl;
//	std::cin.get();
//}