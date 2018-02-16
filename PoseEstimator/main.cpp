#include "P4pf.h"

#include <stdio.h>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#define deg (180/3.14)

void main(void) {

	P4pf estimator;

	Eigen::Matrix<double, 2, 4> ip;
	/*ip << 428, 677, 834, 566,
		870, 855, 1032, 1051;*/

	//ip << 566, 428, 640, 498,
	//	1051, 870, 1045, 867;
	//ip << 988, 1037, 1037, 988,
	//	  570, 566,  566,  560;

	//********** хорошие *************
	//ip << 861, 906, 906, 861,
	//	492, 489, 478, 481;

	ip << 852, 897, 897, 852,
		494, 491, 482, 485;

	//ip << 1048, 1109, 1109, 1048,
	//	809, 801, 787, 795;

	ip << 1080, 1143, 1143, 1080,
		842, 833, 819, 828;
	//********************************

	//ip << 897, 947, 947, 897,
	//	587, 583, 570, 574;

	//ip << 1318, 1397, 1397, 1318,
	//		736, 732, 717, 721;

	Eigen::Matrix<double, 3, 4> op;
	//op << 1.0, 2.4, 2.4, 1.0,
	//	0.0, 0.0, 0.0, 0.0,
	//	1.0, 1.0, 5.0, 5.0;

	double width  = 520;
	double height = 112;
	double offset = 450;

	op <<	offset,		offset + width,		offset + width,		offset,
			offset,		offset,				offset + height,	offset + height,
			offset,		offset,				offset,				offset;



	std::vector<double> focals;
	std::vector<Eigen::Matrix3d> rotateMatrixs;
	std::vector<Eigen::Vector3d> translateVectors;

	int res = estimator.P4Pf(ip, op, &focals, &rotateMatrixs, &translateVectors);

	//if (res < 0) { printf("error!\n"); system("pause"); return; }

	printf("size = %i\n", translateVectors.size());
	int index = 1;

	for (int index = 0; index < translateVectors.size(); index++) {

		Eigen::Matrix3d tmp1 = rotateMatrixs[index].transpose();
		tmp1 = tmp1 * -1;
		Eigen::Vector3d C = tmp1 * translateVectors[index];

		//for(int i = 0; i < focals.size(); i++) {
		//
		//	printf("focal[%i] = %f\n", i, focals[i]);
		//}

		for (int i = 0; i < 3; i++) {

			printf("C[%i] = %f\n", i, C[i]);
		}

		double x = C[0];
		double y = C[1];
		double z = C[2];




		double rotate = deg * atan(x / z);
		printf("rotate = %f", rotate);

		double slope = deg * atan(y / z);
		printf("	slope = %f", slope);

		printf("	height = %f", abs(y));

		printf("\n\n");
	}

	system("pause");
}