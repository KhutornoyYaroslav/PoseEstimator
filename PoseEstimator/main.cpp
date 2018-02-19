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

int EstimatePose(Eigen::Matrix<double, 2, 4> ip, Eigen::Matrix<double, 3, 4> op, PoseResult* result) {

	P4pf estimator;
	std::vector<double> focals;
	std::vector<Eigen::Matrix3d> rotateMatrixs;
	std::vector<Eigen::Vector3d> translateVectors;


	//for (int i = 0; i < 4; i++) {
	//	printf("ip[%i][%i] = %.2f,", 0, i, ip(0,i));
	//	printf("ip[%i][%i] = %.2f\n", 1, i, ip(1, i));
	//}

	for (int i = 0; i < 4; i++) {
		ip(1, i) = (ip(1, i) / 2050) - 0.5;
		ip(0, i) = (ip(0, i) / 2448) - 0.5;
	}

	int res = estimator.P4Pf(ip, op, &focals, &rotateMatrixs, &translateVectors);
	if (res < 0)
		return -1;

	result->rotate = -1;
	result->slope = -1;
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
		//Eigen::Vector3d lol = rotateMatrixs[i].row(1);
		//rotateMatrixs[i].row(1) = rotateMatrixs[i].row(2);
		//rotateMatrixs[i].row(2) = lol;
		// ?!?!?!?!

		double x = C[0];
		double y = C[1];
		double z = C[2];

		double slope = 0.0;
		double rotate = 0.0;
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





		////https://www.learnopencv.com/rotation-matrix-to-euler-angles/
		//double sy = sqrt(rotateMatrixs[i](0, 0) * rotateMatrixs[i](0, 0) + rotateMatrixs[i](1, 0) * rotateMatrixs[i](1, 0));
		//rotate = deg * atan2(-rotateMatrixs[i](2, 0), sy);
		//slope = deg * atan2(rotateMatrixs[i](2, 1), rotateMatrixs[i](2, 2));
		//slope = deg * atan2(rotateMatrixs[i](1, 2), rotateMatrixs[i](1, 1));
		//printf("SLOPE = %.2f!!!!!!\n", slope);
		//printf("ROTATE = %.2f!!!!!!\n", rotate);


		////http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.371.6578&rep=rep1&type=pdf
		////
		//if (abs(rotateMatrixs[i](2, 0)) != 1) {
		//	double ay_1 = -asin(rotateMatrixs[i](2, 0));
		//	double ay_2 = PI - ay_1;

		//	double ax_1 = atan2(rotateMatrixs[i](2, 1) / cos(ay_1), rotateMatrixs[i](2, 2) / cos(ay_1));
		//	double ax_2 = atan2(rotateMatrixs[i](2, 1) / cos(ay_2), rotateMatrixs[i](2, 2) / cos(ay_2));

		//	double az_1 = atan2(rotateMatrixs[i](1, 0) / cos(ay_1), rotateMatrixs[i](0, 0) / cos(ay_1));
		//	double az_2 = atan2(rotateMatrixs[i](1, 0) / cos(ay_2), rotateMatrixs[i](0, 0) / cos(ay_2));

		//	printf("ay_1 = %.2f | ", deg * ay_1);
		//	printf("ay_2 = %.2f | ", deg *  ay_2);

		//	printf("ax_1 = %.2f | ", deg * ax_1);
		//	printf("ax_2 = %.2f | ", deg * ax_2);

		//	printf("az_1 = %.2f | ", deg * az_1);
		//	printf("az_2 = %.2f \n", deg * az_2);
		//}
		
		
		////https://gamedev.stackexchange.com/questions/50963/how-to-extract-euler-angles-from-transformation-matrix
		//double ay = atan2(-rotateMatrixs[i](2, 0), rotateMatrixs[i](0, 0));
		//double ax = asin(rotateMatrixs[i](1, 0));
		//double az = atan2(-rotateMatrixs[i](1, 2), rotateMatrixs[i](1, 1));
		//printf("ay = %.2f | ", deg * ay);
		//printf("ax = %.2f | ", deg * ax);
		//printf("az = %.2f \n ", deg * az);




		//https://gamedev.stackexchange.com/questions/50963/how-to-extract-euler-angles-from-transformation-matrix
		//
		// (X -> Y -> Z) Верно для пешеходного перехода
		//
		double rotXangle = atan2(-rotateMatrixs[i](1, 2), rotateMatrixs[i](2, 2));
		double cosYangle = sqrt(pow(rotateMatrixs[i](0,0), 2) + pow(rotateMatrixs[i](0, 1), 2));
		double rotYangle = atan2(rotateMatrixs[i](0, 2), cosYangle);

		double sinXangle = sin(rotXangle);
		double cosXangle = cos(rotXangle);
		double rotZangle = atan2(cosXangle * rotateMatrixs[i](1, 0) + sinXangle * rotateMatrixs[i](2, 0), 
								cosXangle * rotateMatrixs[i](1, 1) + sinXangle * rotateMatrixs[i](2, 1));

		printf("ay = %.2f | ", deg * rotYangle);
		printf("ax = %.2f | ", deg * rotXangle);
		printf("az = %.2f \n ", deg * rotZangle);
		//--------------------------------------------------------------------------------------------------------


		////http://quabr.com/22709671/the-uniqueness-of-rotation-matrix
		////
		//// X -> Z -> Y
		////
		//double rotXangle = atan2(rotateMatrixs[i](2, 1), rotateMatrixs[i](1, 1)); // A
		////double cosXangle = cos(rotXangle);
		//double rotYangle = atan2(rotateMatrixs[i](0, 2), rotateMatrixs[i](0, 0)); // B

		//double sinYangle = sin(rotYangle); //sinB

		//double rotZangle = acos(rotateMatrixs[i](0, 2) / sinYangle);

		////double cosYangle = sqrt(pow(rotateMatrixs[i](0,0), 2) + pow(rotateMatrixs[i](0, 1), 2));
		////double rotYangle = atan2(rotateMatrixs[i](0, 2), cosYangle);
		//printf("ay = %.2f | ", deg * rotYangle);
		//printf("ax = %.2f | ", deg * rotXangle);
		//printf("az = %.2f \n ", deg * rotZangle);
		////--------------------------------------------------------------------------------------------------------

		






		////Из лекций Алексея
		////
		//double rotXangle = asin(rotateMatrixs[i](1,2));
		//double cosXangle = cos(rotXangle);
		//double rotYangle = acos(rotateMatrixs[i](2, 2) / cosXangle);
		//double rotZangle = acos(rotateMatrixs[i](1, 1) / cosXangle);


		//printf("ay (rotate) = %.2f | ", deg * rotYangle);
		//printf("ax (slope) = %.2f | ", deg * rotXangle);
		//printf("az = %.2f \n ", deg * rotZangle);
		////----------------------------

		//https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
		//
		//double rotXangle = atan2(rotateMatrixs[i](2, 1), rotateMatrixs[i](2,2));
		//double cosXangle = cos(rotXangle);
		//double rotYangle = acos(rotateMatrixs[i](2, 2) / cosXangle);
		//double rotZangle = acos(rotateMatrixs[i](1, 1) / cosXangle);

		//printf("ay (rotate) = %.2f | ", deg * rotYangle);
		//printf("ax (slope) = %.2f | \n", deg * rotXangle);
		//printf("az = %.2f \n ", deg * rotZangle);



		////http://nghiaho.com/?page_id=846
		////
		//double rotXangle = atan2(rotateMatrixs[i](2, 1), rotateMatrixs[i](2, 2));
		//double rotYangle = atan2(-rotateMatrixs[i](2, 0), sqrt(rotateMatrixs[i](2, 1)*rotateMatrixs[i](2, 1) + rotateMatrixs[i](2, 2)*rotateMatrixs[i](2, 2)));
		//double rotZangle = atan2(rotateMatrixs[i](1, 0), rotateMatrixs[i](0, 0));
		//printf("ay (rotate) = %.2f | ", deg * rotYangle);
		//printf("ax (slope) = %.2f | ", deg * rotXangle);
		//printf("az = %.2f \n ", deg * rotZangle);





	






		//slope = deg * atan2(rotateMatrixs[i](2, 1) / cos(-asin(rotateMatrixs[i](2, 0))),
		//				rotateMatrixs[i](2, 2) / cos(-asin(rotateMatrixs[i](2, 0))));
		//rotate = deg * a_y;

		//rotate = abs(rotate);
		//if (rotate >= 90 && rotate < 90 + 45)
		//	rotate = rotate - 90;

		//if (rotate >= 90 + 45)
		//	rotate = 180 - rotate;


		//printf("SLOPE = %.2f!!!!!!\n", slope);
		//printf("ROTATE = %.2f!!!!!!\n", rotate);
		//-----------------
		//printf("height = %f\n", y);
		//printf("focal = %f\n", focals[i]);
		//printf("rotate = %f\n", deg * atan(x / z));
		//printf("slope = %f\n", deg * atan(y / z));
		//printf("\n\n*****************\n");



		rotate = deg * rotYangle;
		slope = deg * rotXangle;
		//if (abs(y) >= 0) {
		//if (abs(slope) <= 25 && abs(slope) >= 5) {
			//if (rotate >= 10 && rotate <= 30) {
		if(abs(deg * rotZangle) < 1.0) {

				//printf("SLOPE = %.2f!!!!!!\n", slope);
				//printf("ROTATE = %.2f!!!!!!\n", rotate);


				result->rotate = rotate;
				result->slope = slope;
				result->focal = focals[i];

				result->height = y;
				result->x = C[0];
				result->y = C[1];
				result->z = C[2];
			//}
		}
	}

	return 0;
}


int PrintHist(std::vector<PoseResult> res) {

	int size = res.size();
	
	for (int i = 0; i < size; i++) {


		if (res[i].focal != -1.0 ) {//&& res[i].height > 0) {
			//double h = round(res[i].height / 100);
			//double r = round(res[i].rotate * 10);
			//double s = round(res[i].slope * 10);
			//double f = round(res[i].focal);

			//printf("h = %.1f | ", h/10);
			//printf("r = %.1f | ", r/10);
			//printf("s = %.1f | ", s / 10);
			//printf("f = %.1f | ", f);
			//printf("x = %.2f | ", res[i].x);
			//printf("z = %.2f\n", res[i].z);

			printf("h = %.2f | ", res[i].height);
			printf("r = %.1f | ", res[i].rotate);
			printf("s = %.1f | ", res[i].slope);
			printf("f = %.2f | ", res[i].focal);
			printf("x = %.2f | ", res[i].x);
			printf("z = %.2f\n", res[i].z);
		}

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
	double w = 520;
	double h = 112;

	double offset = 450;

	//op << 0, w, w, 0,
	//	offset, offset, offset + h, offset + h,
	//	0, 0, 0, 0;

	//op << 0, 1, 1, 0,
	//	0, 0, h/w, h/w,
	//	0, 0, 0, 0;

	op <<	0,     0,    4.0,    4.0,
			0,     0,      0,      0,
			0,  40.0,      0,   40.0;

	if (ReadIpFromFile("data2.txt", &ip_vector) < 0)
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

		if (EstimatePose(ip_vector[i], op, &result) < 0)
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

	PrintHist(result_vec);

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