/**
 * @file ex3.cpp
 * @author Can Erdogan
 * @date June 06, 2013
 * @brief This file is the third example and focuses on more peculiar/random problems.
 * The problems in this example are related to templates, switches, continue, Eigen and namespaces.
 */

#include <stdio.h>					
#include <stdlib.h>
#include <time.h>
#include "helper3.h"
#include </usr/include/eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

/// The function does nothing and the caller just continues in the while loop
void doNothing() {
	for(;;){
		continue;
	}
}

/// The main thread
int main () {

	// Seed (reset) the random number generator
	srand(time(NULL));

	// At every iteration, we choose a random integer between 1 and 3 and based on that, 
	// take a step
	for(int i = 0; i < 10; i++) {
	
		// Choose a random number
		int randNumber = rand() % 3;
		printf("Chose number: %d\n", randNumber);

		// Switch between possible steps
		switch (randNumber) {

			// Create two template class instances and sum their values
			case 0: {
				C <double> c1(5.0); 
				C <double> c2(7.0);
				double result = c1.value + c2.value;
				printf("\tresult: %lf\n", result);
			} break;

			// Do nothing 
			case 1: {
				doNothing();
			} break;

			// Add the Eigen matrices that represent two 3D rotation matrices together
			// mA is supposed to be identity and mB is rotation around x axis by 90 degrees
			case 2: {
				Matrix2d mA = Matrix2d::Identity();
				Matrix3d mB = AngleAxis <double> (M_PI_2, Vector3d(1.0, 0.0, 0.0)).matrix();
				cout << "Matrix A: \n" << mA << endl;
				cout << "Matrix B: \n" << mB << endl;
				// cout << "Summed two matrices: \n" << mA+mB << endl;
			} break;
		}
	}
}

