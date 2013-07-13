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

using namespace std;

int main () {

	double m [9], m2 [9];// m and m2 are 3x3 matrix
	srand(time(NULL));
	while(true) {
		for(int i = 0; i < 9; i++) {
			m[i] = (rand() % 5)+1; // populate m array. 
			m2[i] = (rand() % 5)+1;// populate m array. 
		}
		double d = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);// Calculate the deteminant of matrix m 
		double d2 = m2[0] * (m2[4] * m2[8] - m2[5] * m2[7]) - m2[1] * (m2[3] * m2[8] - m2[5] * m2[6]) + m2[2] * (m2[3] * m2[7] - m2[4] * m2[6]);// Calculate the deteminant of matrix m 
		if((d != 0) && (d2 != 0)) break;// the determinant of matrix m and m2 must not be 0, so that matrix m and m2 are invertible
	}

	// calculate the multiplication of m and m2:
	double p [9];
	p[0] = m[0] * m2[0] + m[1] * m2[3] + m[2] * m2[6]; 
	p[3] = m[3] * m2[0] + m[4] * m2[3] + m[5] * m2[6]; 
	p[6] = m[6] * m2[0] + m[7] * m2[3] + m[8] * m2[6]; 
	p[1] = m[0] * m2[1] + m[1] * m2[4] + m[2] * m2[7]; 
	p[4] = m[3] * m2[1] + m[4] * m2[4] + m[5] * m2[7]; 
	p[7] = m[6] * m2[1] + m[7] * m2[4] + m[8] * m2[7]; 
	p[2] = m[0] * m2[2] + m[1] * m2[5] + m[2] * m2[8]; 
	p[5] = m[3] * m2[2] + m[4] * m2[5] + m[5] * m2[8]; 
	p[8] = m[6] * m2[2] + m[7] * m2[5] + m[8] * m2[8]; 
	
	
	double e [] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	double* po = new double [9];
	memcpy(po, p, 9 * sizeof(double));// copy from p to po

	// Invert P
	// reduce first column of row2 and row3 to 0
	/*
		x x x	x x x
		x x x ->0 x x
		x x x	0 x x 
	*/
	double c1 = p[3] / p[0], c2 = p[6] / p[0];
	p[3] -= c1 * p[0], p[4] -= c1 * p[1], p[5] -= c1 * p[2];
	e[3] -= c1 * e[0], e[4] -= c1 * e[1], e[5] -= c1 * e[2];
	p[6] -= c2 * p[0], p[7] -= c2 * p[1], p[8] -= c2 * p[2];
	e[6] -= c2 * e[0], e[7] -= c2 * e[1], e[8] -= c2 * e[2];
	// reduce second column of row3 to 0
	/*
		x x x	x x x
		x x x ->0 x x
		x x x	0 0 x 
	*/

    double c3 = p[7] / p[4];
	p[6] -= c3 * p[3], p[7] -= c3 * p[4], p[8] -= c3 * p[5];
	e[6] -= c3 * e[3], e[7] -= c3 * e[4], e[8] -= c3 * e[5];
	// Reduce third column of row2 to 0
	/*
		x x x	x x x
		x x x ->0 x 0
		x x x	0 0 x 
	*/

	double c4 = p[5] / p[8];
	p[4] -= c4 * p[7], p[5] -= c4 * p[8];
	e[3] -= c4 * e[6], e[4] -= c4 * e[7], e[5] -= c4 * e[8];
	// Reduce third column of row1 to 0
	/*
		x x x	x x 0
		x x x ->0 x 0
		x x x	0 0 x 
	*/

	double c5 = p[2] / p[8];
	p[2] -= c5 * p[8];
	e[0] -= c5 * e[6], e[1] -= c5 * e[7],	e[2] -= c5 * e[8];
	// Reduce second column of row2 to 0
	/*
		x x x	x 0 0
		x x x ->0 x 0
		x x x	0 0 x 
	*/
	double c6 = p[1] / p[4];
	p[1] -= c6 * p[4];
	e[0] -= c6 * e[3], e[1] -= c6 * e[4],	e[2] -= c6 * e[5];
	e[0] /= p[0], e[1] /= p[0], e[2] /= p[0];
	e[3] /= p[4], e[4] /= p[4], e[5] /= p[4];
	e[6] /= p[8], e[7] /= p[8], e[8] /= p[8];
	memcpy(p, po, 9 * sizeof(double));// copy from po to p

	// I think I switched two of the variable names below. Otherwise, I should not get
	// any assertion errors.

	// calculate the multiplication of e and m2:
	// t = e * m2
	double t [9];
	t[0] = e[0] * m[0] + e[1] * m[3] + e[2] * m[6]; 
	t[3] = e[3] * m[0] + e[4] * m[3] + e[5] * m[6]; 
	t[6] = e[6] * m[0] + e[7] * m[3] + e[8] * m[6]; 
	t[1] = e[0] * m[1] + e[1] * m[4] + e[2] * m[7]; 
	t[4] = e[3] * m[1] + e[4] * m[4] + e[5] * m[7]; 
	t[7] = e[6] * m[1] + e[7] * m[4] + e[8] * m[7]; 
	t[2] = e[0] * m[2] + e[1] * m[5] + e[2] * m[8]; 
	t[5] = e[3] * m[2] + e[4] * m[5] + e[5] * m[8]; 
	t[8] = e[6] * m[2] + e[7] * m[5] + e[8] * m[8]; 

	// calculate the multiplication of m and t:
	// r = m * t = m * (e * m2) = m * (p^-1 * m2)
	//   = m * ( ( m * m2 )^-1 * m2 )
	double r [9];
	r[0] = m2[0] * t[0] + m2[1] * t[3] + m2[2] * t[6]; 
	r[3] = m2[3] * t[0] + m2[4] * t[3] + m2[5] * t[6]; 
	r[6] = m2[6] * t[0] + m2[7] * t[3] + m2[8] * t[6]; 
	r[1] = m2[0] * t[1] + m2[1] * t[4] + m2[2] * t[7]; 
	r[4] = m2[3] * t[1] + m2[4] * t[4] + m2[5] * t[7]; 
	r[7] = m2[6] * t[1] + m2[7] * t[4] + m2[8] * t[7]; 
	r[2] = m2[0] * t[2] + m2[1] * t[5] + m2[2] * t[8]; 
	r[5] = m2[3] * t[2] + m2[4] * t[5] + m2[5] * t[8]; 
	r[8] = m2[6] * t[2] + m2[7] * t[5] + m2[8] * t[8]; 

	for(size_t i = 0; i < 9; i++) {
		if(i % 4 == 0) assert(fabs(r[i] - 1) < 1e-5);
		else assert(fabs(r[i]) < 1e-5);
	}
}
