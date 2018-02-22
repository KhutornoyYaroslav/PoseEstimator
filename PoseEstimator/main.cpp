#include "P4pf.h"

#include <stdio.h>
#include <vector>
#include <math.h>
#include <complex>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#include "common.h"

#define PI 3.1415926535
#define deg (180/3.1415926535)

#define XYZ 0 //
#define XZY 1 //
#define YXZ 2 //
#define YZX 3
#define ZYX 4 //
#define ZXY 5

void GetRotationFronAngles(double rotate, double slope, double roll, Eigen::Matrix3d* R, int type = 0) {

	Eigen::Matrix3d Rx;
	Eigen::Matrix3d Ry;
	Eigen::Matrix3d Rz;

	slope = slope / deg;
	rotate = rotate / deg;
	roll = roll / deg;

	//Rx matrix
	Rx <<	1,				 0,				  0,
			0,		cos(slope),		-sin(slope),
			0,		sin(slope),		 cos(slope);

	//Ry matrix
	Ry <<	 cos(rotate),		0,		sin(rotate),
					   0,		1,				  0,
			-sin(rotate),		0,		cos(rotate);

	//Rz matrix
	Rz <<	cos(roll),	-sin(roll),		0,
			sin(roll),	 cos(roll),		0,
					0,			 0,		1;

	switch(type) {

	case XYZ:
		*R = Rx * Ry * Rz;
		break;

	case XZY:
		*R = Rx * Rz * Ry;
		break;

	case YXZ:
		*R = Ry * Rx * Rz;
		break;

	case YZX:
		*R = Ry * Rz * Rx;
		break;

	case ZXY:
		*R = Rz * Rx * Ry;
		break;

	case ZYX:
		*R = Rz * Ry * Rx;
		break;

	default:
		*R = Rx * Ry * Rz;
		break;
	}
	
}

void GetAnglesFromXYZ(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	//https://gamedev.stackexchange.com/questions/50963/how-to-extract-euler-angles-from-transformation-matrix
	//
	// (X -> Y -> Z) Верно для пешеходного перехода
	//
	double rotXangle = atan2(-rotateMatrixs(1, 2), rotateMatrixs(2, 2));
	double cosYangle = sqrt(pow(rotateMatrixs(0, 0), 2) + pow(rotateMatrixs(0, 1), 2));
	double rotYangle = atan2(rotateMatrixs(0, 2), cosYangle);

	double sinXangle = sin(rotXangle);
	double cosXangle = cos(rotXangle);
	double rotZangle = atan2(cosXangle * rotateMatrixs(1, 0) + sinXangle * rotateMatrixs(2, 0),
		cosXangle * rotateMatrixs(1, 1) + sinXangle * rotateMatrixs(2, 1));

	printf("***** XYZ *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

void GetAnglesFromZYX(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	//https://www.learnopencv.com/rotation-matrix-to-euler-angles/
	//
	// (Z -> Y -> X) 
	//
	double cosYangle = sqrt(rotateMatrixs(0, 0) * rotateMatrixs(0, 0) + rotateMatrixs(1, 0) * rotateMatrixs(1, 0));

	double rotXangle = atan2(rotateMatrixs(2, 1), rotateMatrixs(2, 2));
	double rotZangle = atan2(rotateMatrixs(1, 0), rotateMatrixs(0, 0));
	double rotYangle = atan2(-rotateMatrixs(2, 0), cosYangle);

	printf("***** ZYX *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

void GetAnglesFromXZY(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	//http://quabr.com/22709671/the-uniqueness-of-rotation-matrix
	//
	// X -> Z -> Y
	//
	double rotXangle = atan2(rotateMatrixs(2, 1), rotateMatrixs(1, 1)); // A
	double rotYangle = atan2(rotateMatrixs(0, 2), rotateMatrixs(0, 0)); // B
	double rotZangle = asin(-rotateMatrixs(0, 1)); // C

	printf("***** XZY *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

void GetAnglesFromYXZ(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	// [Cy*Cz + Sx * Sy*Sz,		Cz*Sx*Sy - Cy * Sz,			Cx*Sy]
	// [Cx*Sz,					Cx*Cz,						  -Sx]
	// [Cy*Sx*Sz - Cz * Sy,		Sy*Sz + Cy * Cz*Sx,			Cx*Cy]

	double rotXangle = asin(-rotateMatrixs(1, 2));
	double rotYangle = atan2(rotateMatrixs(0, 2), rotateMatrixs(2, 2));
	double rotZangle = atan2(rotateMatrixs(1, 0), rotateMatrixs(1, 1));

	printf("***** YXZ *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

void GetAnglesFromYZX(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	// [Cy*Cz,		Sx*Sy - Cx * Cy*Sz,		Cx*Sy + Cy * Sx*Sz]
	// [Sz,			Cx*Cz,					-Cz * Sx]
	// [-Cz * Sy,	Cy*Sx + Cx * Sy*Sz,		Cx*Cy - Sx * Sy*Sz]

	double rotXangle = atan2(-rotateMatrixs(1, 2), rotateMatrixs(1, 1));
	double rotYangle = atan2(-rotateMatrixs(2, 0), rotateMatrixs(0, 0));
	double rotZangle = asin(rotateMatrixs(1, 0));

	printf("***** YZX *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

void GetAnglesFromZXY(Eigen::Matrix3d rotateMatrixs, double* rotate, double* slope, double* roll) {

	// [Cy*Cz - Sx * Sy*Sz,		 -Cx * Sz,		Cz*Sy + Cy * Sx*Sz]
	// [Cy*Sz + Cz * Sx*Sy,			Cx*Cz,		Sy*Sz - Cy * Cz*Sx]
	// [-Cx * Sy,					   Sx,				     Cx*Cy]

	double rotXangle = asin(rotateMatrixs(2, 1));
	double rotYangle = atan2(-rotateMatrixs(2, 0), rotateMatrixs(2, 2));
	double rotZangle = atan2(-rotateMatrixs(0, 1), rotateMatrixs(1, 1));

	printf("***** ZXY *****\n");
	printf("ay = %.2f | ", deg * rotYangle);
	printf("ax = %.2f | ", deg * rotXangle);
	printf("az = %.2f \n ", deg * rotZangle);

	*rotate = deg * rotYangle;
	*slope = deg * rotXangle;
	*roll = deg * rotZangle;
}

int ReadIpFromFile(const char* filename, std::vector<Eigen::Matrix<double, 2, 4>>* ip) {

	FILE* f;
	int value;
	Eigen::Matrix<double, 2, 4> tmpMatrix;

	if (fopen_s(&f, filename, "r") < 0)
		return -1;

	while (!feof(f)) {

		value = 0;

		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 4; j++) {

				fscanf_s(f, "%i", &value);
				tmpMatrix(i, j) = static_cast<double>(value);
			}
		}

		ip->push_back(tmpMatrix);
	}

	fclose(f);
	return 0;
}

double inline angle(Eigen::Vector3d v, Eigen::Vector3d u) {

	return deg * acos(v.dot(u)/(v.norm() * u.norm()));
}

double ComputeError(Eigen::Matrix3d rotateMatrix, Eigen::Vector3d translateVector, Eigen::Matrix<double, 2, 4> ip, Eigen::Matrix<double, 3, 4> op) {

	// ДОДЕЛАТЬ!

	Eigen::Matrix<double, 3, 4> P;
	Eigen::Matrix<double, 3, 4> u;
	Eigen::Matrix<double, 2, 4> u_u;
	Eigen::Matrix<double, 4, 4> U;
	Eigen::Matrix<double, 3, 4> delta;
	double error = 0.0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			P(i, j) = rotateMatrix(i, j);
		}

		P(i, 3) = translateVector[2-i];
	}
	
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {

			U(i, j) = op(i, j);
			U(3, j) = 1;
		}		
	}

	Eigen::Matrix <double, 2, 1> ones = { 1, 1 };
	
	u = P * U;

	//for (int i = 0; i < 2; i++) {
	//	for (int j = 0; j < 4; j++) {
	//	
	//		u_u(i, j) = u(i, j) / u(2, j);
	//	}
	//}

	delta = op - u;


	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 4; j++) {
			printf("ip[%i][%i] = %.2f\n", i, j, ip(i, j));
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			printf("u[%i][%i] = %.2f\n", i, j, u(i, j));
		}
	}
	

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {

			error = error + pow(delta(i, j), 2);
		}
	}
	
	error = sqrt(error);

	return error;
}

int EstimatePose(Eigen::Matrix<double, 2, 4> ip, Eigen::Matrix<double, 3, 4> op, PoseResult* result, int frameWidth, int frameHeight) {

	P4pf estimator;
	std::vector<double> focals;
	std::vector<Eigen::Matrix3d> rotateMatrixs;
	std::vector<Eigen::Vector3d> translateVectors;

	for (int i = 0; i < 4; i++) {

		//ip(0, i) = ((frameWidth - ip(0, i)) / frameWidth) - 0.5;
		//ip(1, i) = ((frameHeight - ip(1, i)) / frameHeight) - 0.5;

		ip(0, i) = (ip(0, i) / frameWidth) - 0.5;
		ip(1, i) = (ip(1, i) / frameHeight) - 0.5;

		
	}

	//for (int i = 0; i < 4; i++) {

	//	printf("ip[%i][%i] = %.2f\n", 0, i, ip(0,i));
	//	printf("ip[%i][%i] = %.2f\n", 1, i, ip(1, i));
	//}

	int res = estimator.P4Pf(ip, op, &focals, &rotateMatrixs, &translateVectors);
	if (res < 0)
		return -1;

	result->rotate = -1;
	result->slope = -1;
	result->roll = -100;
	result->height = -1;
	result->focal = -1;
	result->x = -1;
	result->y = -1;
	result->z = -1;

	printf("EP. size = %i\n", focals.size());

	for (int i = 0; i < focals.size(); i++) {

		Eigen::Matrix3d tmp = rotateMatrixs[i];
		Eigen::Matrix3d tmp1 = tmp.transpose();
		tmp1 = -1 * tmp1;
		Eigen::Vector3d C = tmp1 * translateVectors[i];

		// ?!?!?!?!
		//Eigen::Matrix3d tmp2 = tmp.transpose();
		//rotateMatrixs[i] = tmp2;
		// ?!?!?!?!

		double x = C[0];
		double y = C[1];
		double z = C[2];

		double slope = 0.0;
		double rotate = 0.0;
		double roll = 0.0;
		//-----------------
		//slope = deg * -atan2(rotateMatrixs[i](2, 1), sqrt(rotateMatrixs[i](2, 0) * rotateMatrixs[i](2, 0)   +   rotateMatrixs[i](2,2) * rotateMatrixs[i](2,2)));
		//rotate = deg * atan2(rotateMatrixs[i](2, 0), rotateMatrixs[i](2,2)); // 2 0

		//double slope = deg * -atan2(rotateMatrixs[i](1, 2), sqrt(rotateMatrixs[i](0, 2) * rotateMatrixs[i](0, 2) + rotateMatrixs[i](2, 2) * rotateMatrixs[i](2, 2)));
		//rotate = deg * atan2(rotateMatrixs[i](0, 2), rotateMatrixs[i](2, 2)); // 0 2


		//rotate = deg * -asin(rotateMatrixs[i](0,2));        /* Считаем ось Y */
		
		//printf("SLOPE = %.2f!!!!!!\n", slope);
		//printf("ROTATE = %.2f!!!!!!\n", rotate);

		//rotate = deg * atan2(-rotateMatrixs[i](2, 0), sqrt(rotateMatrixs[i](2, 1) * rotateMatrixs[i](2, 1) + rotateMatrixs[i](2, 2) * rotateMatrixs[i](2, 2)));
		//slope = deg * atan2(rotateMatrixs[i](2, 1), rotateMatrixs[i](2, 2));
	
		//double cc = cos(-asin(rotateMatrixs[i](0, 2)));
		//double trX = (rotateMatrixs[i](2, 2)) / cc;           /* Нет, так что находим угол по X */
		//double trY = -(rotateMatrixs[i](1, 2)) / cc;

		//slope = atan2(trY, trX) * deg;


	

		printf("\n***************\n");
		GetAnglesFromXYZ(rotateMatrixs[i], &rotate, &slope, &roll);
		GetAnglesFromXZY(rotateMatrixs[i], &rotate, &slope, &roll);	
		GetAnglesFromYXZ(rotateMatrixs[i], &rotate, &slope, &roll);
		GetAnglesFromYZX(rotateMatrixs[i], &rotate, &slope, &roll);
		GetAnglesFromZXY(rotateMatrixs[i], &rotate, &slope, &roll);
		GetAnglesFromZYX(rotateMatrixs[i], &rotate, &slope, &roll);

		
/*
		printf("\n\n***** Original rotation matrix *****\n");
		for (int j = 0; j < 3; j++) {

			printf("R[%i][%i] = %.4f, ", 0, j, rotateMatrixs[i](0, j));
			printf("R[%i][%i] = %.4f, ", 1, j, rotateMatrixs[i](1, j));
			printf("R[%i][%i] = %.4f\n", 2, j, rotateMatrixs[i](2, j));
		}*/
		
		//Eigen::Matrix3d ComputeR;
		//GetRotationFronAngles(rotate, slope, roll, &ComputeR, ZXY);

		//printf("***** Compute rotation matrix *****\n");
		//for (int j = 0; j < 3; j++) {

		//	printf("R[%i][%i] = %.4f, ", 0, j, ComputeR(0, j));
		//	printf("R[%i][%i] = %.4f, ", 1, j, ComputeR(1, j));
		//	printf("R[%i][%i] = %.4f\n", 2, j, ComputeR(2, j));
		//}

		//printf("***** Error rotation matrix *****\n");
		//for (int j = 0; j < 3; j++) {

		//	printf("E[%i][%i] = %.4f, ", 0, j, ComputeR(0, j) - rotateMatrixs[i](0, j));
		//	printf("E[%i][%i] = %.4f, ", 1, j, ComputeR(1, j) - rotateMatrixs[i](1, j));
		//	printf("E[%i][%i] = %.4f\n", 2, j, ComputeR(2, j) - rotateMatrixs[i](2, j));
		//}




		//if(abs(roll) <= 10.0) {

			
				result->rotate = rotate;
				result->slope = slope;
				result->roll = roll;

				result->focal = focals[i];

				result->height = y;
				result->x = C[0];
				result->y = C[1];
				result->z = C[2];
			//}
		//}

				/*if (abs(rotate) > 90)
					rotate = abs(rotate) - 90;

				if (abs(slope) > 90)
					rotate = abs(slope) - 90;

				if (abs(rotate) > 180)
					rotate = abs(rotate) - 180;

				if (abs(slope) > 180)
					rotate = abs(slope) - 180;

				printf("h = %.2f | ", y);
				printf("ror = %.1f | ", rotate);
				printf("slo = %.1f | ", slope);
				printf("roll = %.1f | ", roll);
				printf("f = %.2f | ", focals[i]);
				printf("x = %.2f | ", C[0]);
				printf("z = %.2f\n", C[2]);*/


	}

	return 0;
}


int PrintHist(std::vector<PoseResult> res) {

	int size = res.size();
	
	for (int i = 0; i < size; i++) {

		//if (abs(res[i].roll) < 10.0) {//&& res[i].height > 0) {
		//if (abs(res[i].roll) <= 165.0 && abs(res[i].roll) >= 145.0) {

			printf("h = %.2f | ", res[i].height);
			printf("ror = %.1f | ", res[i].rotate);
			printf("slo = %.1f | ", res[i].slope);
			printf("roll = %.1f | ", res[i].roll);
			printf("f = %.2f | ", res[i].focal);
			printf("x = %.2f | ", res[i].x);
			printf("z = %.2f\n", res[i].z);
		//}

	}

	return 0;
}


void main(void) {

	P4pf estimator;

	std::vector<double> focals;
	Eigen::Matrix<double, 2, 4> ip;
	Eigen::Matrix<double, 3, 4> op;
	std::vector<Eigen::Matrix3d> rotateMatrixs;
	std::vector<Eigen::Vector3d> translateVectors;
	std::vector<Eigen::Matrix<double, 2, 4>> ip_vector;
	std::vector<PoseResult> result_vec;
	PoseResult result;
	double w = 520; // !!!!!!!!!!!!!!!!!!!
	double h = 112;

	double offset = 450;

	//op << 0, w, w, 0,
	//	offset, offset, offset + h, offset + h,
	//	0, 0, 0, 0;

	//op << 0, 1, 1, 0,
	//	0, 0, h/w, h/w,
	//	0, 0, 0, 0;

	//op <<	0,	w/w,	w/w,	0,
	//		0,	0,		h/w,	h/w,
	//		0,	0,		0,		0;


	//op <<	0,     0,    4.0,    4.0,
	//		0,     0,      0,      0,
	//		0,  40.0,      0,   40.0;

	////горизонтальная плоскость
	//h = 210;
	//w = 297;
	//op << 0, w, w, 0,
	//	0, 0, 0, 0,
	//	0, 0, h, h;

	//вертикальная плоскость
	h = 210;
	w = 297;
	op << 0, w, w, 0,
	   	0, 0, h, h,
	  	0, 0, 0, 0;


		

	if (ReadIpFromFile("data4.txt", &ip_vector) < 0)
		printf("File error!\n");

	double height_avr = 0.0;
	double rotate_avr = 0.0;
	double slope_avr = 0.0;
	double x_avr = 0.0;
	double z_avr = 0.0;
	double focal_avr = 0.0;
	int count = 0;

	for (int i = 0; i < ip_vector.size(); i++) {

		printf("****** i = %i ******** \n", i);

		if (EstimatePose(ip_vector[i], op, &result, 2048, 1536) < 0)
		//if (EstimatePose(ip_vector[i], op, &result, 2448, 2050) < 0)
			printf("Estimate error!\n");

		result_vec.push_back(result);

		
		//printf("height = %.2f | ", result.height);
		//printf("focal = %.2f | ", result.focal);
		//printf("rotate = %.2f | ", result.rotate);
		//printf("slope = %.2f\n", result.slope);

		//printf("x = %.5f | ", result.x);
		//printf("z = %.5f \n ", result.z);
		
		printf("\n\n*****************\n");

		

		if (result_vec[i].focal != -1 && result_vec[i].height > 0) {

			height_avr = height_avr + result.height;
			rotate_avr = rotate_avr + abs(result.rotate);
			slope_avr = slope_avr + abs(result.slope);
			x_avr = x_avr + result.x;
			z_avr = z_avr + result.z;
			focal_avr = focal_avr + result.focal;

			count++;
		}
	}

	//PrintHist(result_vec);

	height_avr = height_avr / count;
	printf("\n\nAverage height = %f\n", height_avr);

	rotate_avr = rotate_avr / count;
	printf("Average rotate = %f\n", rotate_avr);

	slope_avr = slope_avr / count;
	printf("Average slope = %f\n", slope_avr);

	x_avr = x_avr / count;
	printf("Average x = %f\n", x_avr);

	z_avr = z_avr / count;
	printf("Average z = %f\n", z_avr);

	focal_avr = focal_avr / count;
	printf("Average focal  = %f\n", focal_avr);

	printf("size = %i\n", count);
	




	system("pause");
}