/**
 * @file ex1.cpp
 * @author Can Erdogan
 * @date June 08, 2013
 * @brief This file is a part of the tutorial for new students and is written to demonstrate the 
 * importance of commenting. To do so, it carries out an algorithm, hopefully familiar to the
 * reader, in a very convoluted and uncommented way, and there is a slight problem with it
 * NOTE If you see a Nan or Inf in the results, re-run the program - just bad random values.
 */

#include <stdio.h>
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include </usr/include/eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main () {
	//Create 3x3 matrix m and m2
	MatrixXd m, m2;
	while(true){
		// Populate m and m2 with random number
		m = MatrixXd::Random(3,3);
		m2 = MatrixXd::Random(3,3);
		double d = m.determinant();
		double d2 = m2.determinant();
		// if determinant of d and d2 are not zero, break out.
		// m and m2 are invertible matrix
		if( (d != 0) && (d2 != 0) ) break;
	}
	Matrix3d p = m * m2;
	Matrix3d r = m2* ( p.inverse()* m );
	for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				if( i == j) assert(fabs(r(i,j) - 1) < 1e-5);
				else assert(fabs(r(i,j)) < 1e-5);
			}
	}
}
